# CfC training (manual Part 4)

This directory holds the off-board training + ONNX export entry points for the
Phase 3 CfC pilot. The trained policy is exported to ONNX and deployed to the
Pi, where `control/cfc_pilot/cfc_pilot.hpp` runs it one step per 20 Hz control
tick via onnxruntime.

## Pipeline

```
SITL rollouts (.pt)  ──►  train_cfc.py  ──►  cfc.pt  ──►  export_onnx.py  ──►  cfc_policy.onnx
                                                          (deploy to Pi)
```

## 1. Install training deps

```bash
# REQUIRES: pip install ncps torch onnx onnxruntime numpy
pip install ncps torch onnx onnxruntime numpy
```

`ncps` (the Closed-form Continuous-time cell) is the differentiator — it
provides `ncps.torch.CfC` + `ncps.wirings.AutoNCP` used by `CfCPolicy` in
`train_cfc.py`.

## 2. Collect SITL rollouts

Fly the deterministic planner (the A*/greedy fallback in
`core/navigation.hpp`) through randomized Gazebo obstacle courses via PX4 SITL,
logging one tuple per rollout:

| field     | shape       | dtype   | notes                                   |
|-----------|-------------|---------|-----------------------------------------|
| `obs_seq` | `(T, 31)`   | float32 | `build_observation` output per tick      |
| `act_seq` | `(T, 4)`    | float32 | `(vx, vy, vz, yaw_rate)` body frame, pre-normalized to `[-1, 1]` |
| `ts_seq`  | `(T,)`      | float32 | per-step `dt` seconds                    |

Save each rollout as a `.pt` file (torch.save of a 3-tuple in that order, or a
dict with those keys) into a data directory:

```bash
python sim/quickstart.sh            # spins up PX4 SITL + onboard stack
# (Phase 5 todo: the rollout recorder itself — see sim/quickstart.sh)
```

The action must be **pre-normalized to `[-1, 1]`** by the rollout recorder
(divide velocities by `max_velocity` and yaw-rate by `yaw_max_dps`) since the
policy output is `tanh`-clamped to `[-1, 1]`. The onboard `CfCPilot` re-scales
the policy output back to m/s and deg/s on the Pi.

Test on **held-out (unseen)** courses — generalization to unseen maps is the
headline result for liquid nets and the Phase 3 exit criterion (manual Part 4.5).

## 3. Train

```bash
python training/train_cfc.py \
    --data-dir /tmp/cfc_rollouts \
    --epochs 50 \
    --lr 1e-3 \
    --units 48 \
    --out cfc.pt
```

`--units` must match what you pass to `export_onnx.py` and to the onboard
`CfCPilot` constructor (`/etc/dronectl/config.yaml` `cfc.hidden_units`, manual
Part 8.2). `OBS_DIM = 31` and `ACT_DIM = 4` are defined in `train_cfc.py` and
must stay in lock-step with `control/cfc_pilot/observation.hpp`.

`train_cfc.py` is import-safe: importing it does not require `ncps` to be
installed (the heavy imports are deferred into `CfCPolicy.__init__`). The
training entry point is guarded by `if __name__ == "__main__":`.

## 4. Export to ONNX

```bash
python training/export_onnx.py \
    --model cfc.pt \
    --out  cfc_policy.onnx \
    --units 48
```

The export wraps the sequence-oriented `CfCPolicy` in a `CfCStep` module that
adds/removes a length-1 sequence dimension, so the ONNX graph has fixed
single-step shapes `obs: [1,31]`, `hx: [1,48]`, `ts: [1]` -> `act: [1,4]`,
`hx_out: [1,48]`. Fixed shapes are the fastest path on the Pi's ARM core.
Opset 17; input names `["obs","hx","ts"]`, output names `["act","hx_out"]`
(matched verbatim by `CfCPilot::step` in `control/cfc_pilot/cfc_pilot.hpp`).

## 5. Deploy to the Pi

Copy the ONNX onto the Pi at the path the onboard `CfCPilot` reads (manual
Part 8.2):

```
/opt/dronectl/cfc_policy.onnx
```

Permissions: `0644` owned by `root:dronectl` (or whatever the
`dronectl-pilot.service` run-as user is). The `/etc/dronectl/config.yaml`
`cfc.model` key (Part 8.2) points `pilot_main` at this path; the default CLI
value is already `/opt/dronectl/cfc_policy.onnx`.

## 6. Re-training / iteration

- Want to add RL fine-tuning (manual Part 4.2)? Wrap the SITL env in
  Gymnasium + PPO with `reward = progress - collision - jerk`. Keep the
  safety filter active during and after training. Left as a Phase 5+ task.
- Changed `OBS_DIM` / `ACT_DIM` in `observation.hpp`? Update `K_SECTORS` in
  `train_cfc.py` too and re-collect rollouts — the observation shape must
  match exactly across training, export, and onboard inference.
- The deterministic safety filter (`CfCPilot::safety_filter`) is a hardcoded
  geometric veto, never learned — keep it active even after RL fine-tuning.