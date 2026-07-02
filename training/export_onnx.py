#!/usr/bin/env python3
"""Phase 3 (manual Part 4.3): export the trained CfC as a single-step ONNX model.

The Pi runs ONE inference step per control tick and carries the hidden state
itself. We therefore wrap the sequence-oriented ``CfCPolicy`` in a
``CfCStep`` module that adds the (length-1) sequence dimension on the way in
and removes it on the way out, so the ONNX graph has fixed single-step shapes
``obs: (1, OBS_DIM)``, ``hx: (1, U)``, ``ts: (1,)`` -> ``act: (1, ACT_DIM)``,
``hx_out: (1, U)``. Fixed shapes are the fastest path on the Pi's ARM core.

REQUIRES: pip install ncps torch onnx onnxruntime numpy

This script is the actual export entry point so it eagerly imports torch /
ncps; it is NOT import-safe (training/train_cfc.py is the import-safe module).
Run it from the command line:

    python training/export_onnx.py \\
        --model /path/to/cfc.pt \\
        --out  /opt/dronectl/cfc_policy.onnx \\
        --units 48

The resulting ``cfc_policy.onnx`` is what ``CfCPilot`` loads on the Pi
(manual Part 4.4 / Part 8.2).
"""

from __future__ import annotations

import argparse
import sys
from typing import Sequence

import torch

# Reuse the policy definition from the training module so OBS_DIM/ACT_DIM stay
# in sync with the C++ header automatically.
from train_cfc import CfCPolicy, OBS_DIM, ACT_DIM  # noqa: F401  (ACT_DIM exercised below)


class CfCStep(torch.nn.Module):
    """Single-step wrapper around the sequence-oriented CfCPolicy.

    Adds a length-1 sequence dimension for the forward pass, then squeezes it
    back out so the ONNX graph is single-step with explicit hidden-state I/O.
    """

    def __init__(self, net: "CfCPolicy") -> None:
        super().__init__()
        self.net = net

    def forward(
        self,
        obs: torch.Tensor,   # (1, OBS_DIM)
        hx: torch.Tensor,    # (1, U)
        ts: torch.Tensor,    # (1,)
    ):
        y, hx2 = self.net(obs.unsqueeze(1), hx, ts.unsqueeze(1))  # add seq dim
        return y.squeeze(1), hx2  # (1, ACT_DIM), (1, U)


def export(model_path: str, onnx_path: str, units: int) -> None:
    net = CfCPolicy(units=units)
    net.load_state_dict(torch.load(model_path, map_location="cpu"))
    net.eval()

    step = CfCStep(net).eval()
    U = net.state_size
    # Dummy inputs with the exact fixed shapes the Pi will pass each tick.
    dummy = (
        torch.zeros(1, OBS_DIM, dtype=torch.float32),
        torch.zeros(1, U, dtype=torch.float32),
        torch.ones(1, dtype=torch.float32),
    )

    torch.onnx.export(
        step,
        dummy,
        onnx_path,
        opset_version=17,
        input_names=["obs", "hx", "ts"],
        output_names=["act", "hx_out"],
        dynamic_axes=None,  # fixed shapes = fastest on the Pi
    )
    print(
        f"[onnx] exported {model_path} (units={U}) -> {onnx_path} "
        f"(obs: [1,{OBS_DIM}], hx: [1,{U}], ts: [1] -> "
        f"act: [1,{ACT_DIM}], hx_out: [1,{U}])"
    )


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Export a trained CfC policy as a single-step ONNX model "
                    "with explicit hidden-state I/O for onboard Pi inference."
    )
    p.add_argument("--model", required=True,
                   help="Path to the trained cfc.pt weights (torch state_dict).")
    p.add_argument("--out", default="cfc_policy.onnx",
                   help="Output ONNX path (default: cfc_policy.onnx). On "
                        "the Pi this is /opt/dronectl/cfc_policy.onnx "
                        "(manual Part 8.2).")
    p.add_argument("--units", type=int, default=48,
                   help="CfC hidden size used during training (must match).")
    return p.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    export(args.model, args.out, args.units)
    return 0


if __name__ == "__main__":
    sys.exit(main())