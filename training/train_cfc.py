#!/usr/bin/env python3
"""Phase 3 (manual Part 4.2): off-board CfC training entry point.

Trains a Closed-form Continuous-time (CfC) liquid neural network by imitation
of the repo's deterministic A*/greedy planner (manual Part 1.6), then
optionally RL-finetunes. The trained policy is exported to ONNX for onboard
inference on the Pi via ``training/export_onnx.py``.

REQUIRES: pip install ncps torch onnx onnxruntime numpy

This module is import-safe: importing it does NOT require ``ncps`` to be
installed. The heavy imports (``ncps.torch.CfC`` etc.) are deferred inside the
``CfCPolicy`` class definition, which raises a clear ImportError with install
instructions only when someone actually tries to instantiate the policy. The
training entry point itself is guarded by the ``if __name__ == "__main__":``
block at the bottom.

SITL data collection (manual Part 4.2 "Collect data in SITL")
-------------------------------------------------------------
Before training, collect imitation rollouts in PX4 SITL + Gazebo:

    # Terminal 1: spin up a simulated x500 on udp://:14540
    git clone https://github.com/PX4/PX4-Autopilot --recursive
    cd PX4-Autopilot && make px4_sitl gz_x500

    # Terminal 2: launch the onboard stack pointed at SITL, logging
    # (observation, planner velocity command, dt) at each 20 Hz control tick.
    # The rollout script itself is out of scope for this file (see
    # ``sim/quickstart.sh``); it should save each rollout as a tuple
    #   (obs_seq: np.ndarray[T, OBS_DIM] float32,
    #    act_seq: np.ndarray[T, 4]       float32,   # vx,vy,vz,yaw_rate in body frame
    #    ts_seq:  np.ndarray[T]          float32)   # per-step dt in seconds
    # to ``--data-dir`` as a ``.pt`` file (torch.save of a dict or tuple).

Then run:

    python training/train_cfc.py --data-dir /tmp/cfc_rollouts \\
                                 --epochs 50 --lr 1e-3 --out cfc.pt --units 48

Fly the deterministic planner through many *randomized* obstacle courses so
the CfC sees diverse structure; test on held-out (unseen) courses — that
generalization is the headline result for liquid nets (manual Part 4.2).
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import Iterable, List, Sequence, Tuple

import numpy as np
import torch
import torch.nn as nn

# Observation / action dimensions must stay in lock-step with the C++ header
# ``control/cfc_pilot/observation.hpp``.
K_SECTORS = 24
OBS_DIM = K_SECTORS + 3 + 1 + 3       # = 31
ACT_DIM = 4                            # vx, vy, vz, yaw_rate (body frame)


class CfCPolicy(nn.Module):
    """CfC policy wrapped in ncps' AutoNCP wiring.

    Forward signature mirrors the manual Part 4.2 sketch: takes a sequence
    tensor of shape ``(batch, seq, OBS_DIM)`` and returns ``(tanh(y), hx)``
    where ``y`` is ``(batch, seq, ACT_DIM)`` clamped to ``[-1, 1]`` and ``hx``
    is the carried hidden state. The caller scales the tanh output to physical
    units (e.g. m/s, deg/s) after inference.
    """

    def __init__(self, units: int = 48) -> None:
        super().__init__()
        try:
            from ncps.torch import CfC
            from ncps.wirings import AutoNCP
        except ImportError as exc:  # pragma: no cover - import-safe guard
            raise ImportError(
                "ncps is required to instantiate CfCPolicy. "
                "Install it with: pip install ncps"
            ) from exc
        self.wiring = AutoNCP(units, ACT_DIM)
        self.cfc = CfC(OBS_DIM, self.wiring, batch_first=True)
        self.state_size = units  # hidden size carried across ONNX steps

    def forward(
        self,
        x: torch.Tensor,
        hx: torch.Tensor | None = None,
        ts: torch.Tensor | None = None,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        # x: (batch, seq, OBS_DIM) -> y: (batch, seq, ACT_DIM), hx
        y, hx = self.cfc(x, hx, timespans=ts)
        return torch.tanh(y), hx


# ---------------------------------------------------------------------------
# Dataset loading
# ---------------------------------------------------------------------------
Rollout = Tuple[torch.Tensor, torch.Tensor, torch.Tensor]


def _load_rollout(path: str) -> Rollout:
    """Load one ``(obs_seq, act_seq, ts_seq)`` rollout from a .pt file.

    Accepts either a 3-tuple/list in that order, or a dict with those keys.
    Sequences are cast to float32 and reshaped to ``(1, T, *)`` (single batch)
    so the imitation loop can concatenate-mini-batch across rollouts.
    """
    blob = torch.load(path, map_location="cpu")
    if isinstance(blob, dict):
        obs = torch.as_tensor(blob["obs_seq"], dtype=torch.float32)
        act = torch.as_tensor(blob["act_seq"], dtype=torch.float32)
        ts = torch.as_tensor(blob["ts_seq"], dtype=torch.float32)
    else:
        obs = torch.as_tensor(blob[0], dtype=torch.float32)
        act = torch.as_tensor(blob[1], dtype=torch.float32)
        ts = torch.as_tensor(blob[2], dtype=torch.float32)
    if obs.ndim == 2:
        obs = obs.unsqueeze(0)   # (1, T, OBS_DIM)
    if act.ndim == 2:
        act = act.unsqueeze(0)   # (1, T, ACT_DIM)
    if ts.ndim == 1:
        ts = ts.unsqueeze(0)      # (1, T)
    assert obs.shape[-1] == OBS_DIM, (
        f"obs last dim {obs.shape[-1]} != OBS_DIM {OBS_DIM}"
    )
    assert act.shape[-1] == ACT_DIM, (
        f"act last dim {act.shape[-1]} != ACT_DIM {ACT_DIM}"
    )
    return obs, act, ts


def load_dataset(data_dir: str) -> List[Rollout]:
    """Load every ``*.pt`` rollout in ``data_dir`` as a list of (obs, act, ts)."""
    paths = sorted(glob.glob(os.path.join(data_dir, "*.pt")))
    if not paths:
        raise FileNotFoundError(
            f"No .pt rollouts found in {data_dir}. Collect SITL data first "
            "(see module docstring)."
        )
    return [_load_rollout(p) for p in paths]


# ---------------------------------------------------------------------------
# Training loop (imitation)
# ---------------------------------------------------------------------------
def train(
    dataset: Sequence[Rollout],
    epochs: int = 50,
    lr: float = 1e-3,
    units: int = 48,
    out_path: str | None = None,
) -> "CfCPolicy":
    """Imitation-learning loop over collected SITL rollouts.

    Each rollout is a sequence: the CfC ingests the whole observation sequence
    and is supervised to reproduce the planner's velocity command at every
    timestep. MSE loss on the (already tanh-clamped) policy output vs. the
    planner's normalized action.
    """
    net = CfCPolicy(units=units)
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    lossf = nn.MSELoss()

    for ep in range(epochs):
        ep_loss = 0.0
        n_batches = 0
        for obs_seq, act_seq, ts_seq in dataset:
            # obs_seq: (1, T, OBS_DIM); act_seq: (1, T, 4); ts_seq: (1, T)
            pred, _ = net(obs_seq, ts=ts_seq)
            # Planner actions are assumed pre-normalized to [-1, 1] by the
            # rollout recorder; tanh on the policy side keeps us in that range.
            loss = lossf(pred, act_seq)
            opt.zero_grad()
            loss.backward()
            opt.step()
            ep_loss += float(loss.item())
            n_batches += 1
        avg = ep_loss / max(n_batches, 1)
        print(f"[cfc] epoch {ep + 1}/{epochs}  loss={avg:.6f}")

    if out_path is not None:
        torch.save(net.state_dict(), out_path)
        print(f"[cfc] saved weights -> {out_path}")
    return net


# ---------------------------------------------------------------------------
# CLI entry point (guarded so the module is import-safe)
# ---------------------------------------------------------------------------
def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Train the CfC pilot by imitation of the deterministic "
                    "planner on SITL rollouts. See module docstring for the "
                    "data-collection step."
    )
    p.add_argument("--data-dir", required=True,
                   help="Directory of *.pt rollouts, each saved as "
                        "(obs_seq[T,OBS_DIM], act_seq[T,4], ts_seq[T]) "
                        "tuples.")
    p.add_argument("--out", default="cfc.pt",
                   help="Output .pt weights path (default: cfc.pt).")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--units", type=int, default=48,
                   help="CfC hidden size (must match the export step and "
                        "the onboard CfCPilot constructor).")
    return p.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    dataset = load_dataset(args.data_dir)
    print(f"[cfc] loaded {len(dataset)} rollout(s) from {args.data_dir}")
    train(dataset, epochs=args.epochs, lr=args.lr,
          units=args.units, out_path=args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())