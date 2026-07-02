# Tests — the ladder mapped to repo binaries

The repo's test ladder mirrors manual Part 9.5 (see
[docs/SAFETY.md](../docs/SAFETY.md) for the verbatim ladder + the
pre-flight card). This page is the mapping from each ladder rung to
the actual test binary / harness that lives in the repo, and which
rungs are automated in CI vs run by hand.

## Rung 1 — Unit

These are pure-C++ unit tests; they run on every push in CI.

| Binary | Source | Gate |
| --- | --- | --- |
| `smoke_test` | `tests/smoke_test.cpp` | Cross-header compile + behavior smoke (math / grid / pose / motion / ray / cluster / nav). |
| `odr_two_tu` | `tests/odr_two_tu_main.cpp` + `tests/odr_two_tu_other.cpp` | **Two-TU ODR regression.** Two translation units that both include every header must link cleanly. Before the Phase 0 `inline` fixes this failed to link (multiply-defined free functions). This binary is the gate that keeps that whole class of regressions closed. |
| `failsafe_test` | `tests/failsafe_test.cpp` (against header-only `control/failsafe.hpp`) | State-machine transitions: BOOT -> IDLE -> ARMED -> AUTONOMOUS, link-loss, kill, low-battery, geofence, fault reset. |
| `perception_compile_test` | `tests/perception_compile_test.cpp` + `perception/drone_pose_provider.cpp` | Phase 1 perception compile/link gate. Links the perception TU against a small test driver so the new headers + `DronePoseProvider` are exercised. Implicitly also a small two-TU ODR check for the perception code. |
| `observation_test` | `tests/observation_test.cpp` (against header-only `control/cfc_pilot/observation.hpp`) | Observation builder (Part 4.1): sector distances, normalization, goal vector. Pure-C++ asserts on hand-crafted grids. |
| `cfc_safety_test` | `tests/cfc_safety_test.cpp` (against header-only `control/cfc_pilot/cfc_pilot.hpp` `safety_filter`) | The deterministic geometric veto (Part 4.4): command-into-obstacle is clamped to HOLD; clear commands pass through. Pure-C++, **no onnxruntime needed** — the safety filter is deliberately decoupled from inference so it stands on its own. |

## Rung 2 — SITL

| Harness | Path | Gate |
| --- | --- | --- |
| `sim/quickstart.sh` | `sim/quickstart.sh` | Spins up PX4 SITL + Gazebo and points the full onboard stack (`pilot_main`, CfC, `FcBridge`, `tether-agent`) at `udp://:14540`. Fly a simulated drone with zero hardware. |

The SITL rung is partially automatable (assert sim drone moves when a
velocity setpoint is sent; assert telemetry streams back into the
mapper; assert killing the tether does not change flight behavior) but
the harness itself is **run-by-hand** today. CI does not run SITL.

Manual acceptance for rung 2 (manual Part 3.4 / Part 4 exit criterion):
command a velocity setpoint from code and the simulated drone moves;
FC pose streams back into the mapper; kill-link tests leave the drone
flying.

## Rungs 3–6 — Manual (gated)

These rungs involve real hardware and real airspace. They cannot be
automated; they are gated by the pre-flight card in
[docs/SAFETY.md](../docs/SAFETY.md) and by the regulatory gate in
[docs/REGULATORY.md](../docs/REGULATORY.md).

| Rung | What | Gate |
| --- | --- | --- |
| 3. HITL | Real Pi + real FC, **props removed**, sim world. | Operator + bench. |
| 4. Tethered low hover | Open area, < 2 m, tether attached, kill-switch in hand. First real props. | Operator + airframe + pre-flight card. |
| 5. Open-area autonomy | Progressively larger goals; CfC drives; log everything. | Operator + airframe + Part 107 + LAANC. |
| 6. Backhaul / fleet | Add Starlink + a second drone **only after single-drone is boring.** | Operator + airframe × N + BVLOS waiver + fleet authorization. |

## What runs in CI

Automated on every push (`.github/workflows/build.yml`):

- **Linux Ubuntu** (the gate): `make` builds `embedded_voxel_mapper`,
  `smoke_test`, `odr_two_tu`, `perception_compile_test`,
  `failsafe_test`, `observation_test`, `cfc_safety_test`; `make test`
  runs the mapper smoke + the six unit binaries in sequence; anything
  failing stops the pipeline.
- A separate job (matrix `ubuntu-latest`) configures CMake with
  `DRONECTL_ENABLE_FC_BRIDGE=OFF` (and `DRONECTL_ENABLE_CFC=OFF` by
  default) and attempts a configure + build. The CMake run is the
  reference-configurate check; MAVSDK / onnxruntime are not installed
  on the CI machine so the option gates keep the bare configure green.
- (Existing) `rpi-cross` job cross-compiles the mapper for aarch64 to
  catch Pi build breakage early.

Not in CI:

- SITL (rung 2) — needs a Gazebo host and the PX4 source tree.
- HITL onward (rungs 3–6) — needs hardware + airspace authorization.

## Building the unit binaries

The Makefile is the gate; the CMakeLists.txt is the reference build.

```bash
# Make (the gate):
make              # builds embedded_voxel_mapper (+ smoke binaries via `test`)
make test         # runs the mapper smoke + the four unit binaries

# CMake (the reference build):
cmake -B build .
cmake --build build
ctest --test-dir build --output-on-failure
```

See the main [README](../README.md) for the build matrix on Pi / macOS
/ WSL and [docs/SAFETY.md](../docs/SAFETY.md) for the rungs themselves.