# Architecture — one-page diagram (Option A)

This is a single-page view of the two stacks in Option A — the onboard
Pi pilot (top) and the ground / server stack (bottom) — joined by the
fiber tether. It mirrors the manual Part 0 diagram, with pointers to
the actual repo paths so you can read the source behind each box.

The headline properties:

- **The flight-critical path is on the drone.** Losing the fiber does
  not stop flight; the onboard CfC keeps flying toward the last goal
  and runs the configured link-loss action (see `control/failsafe.hpp`,
  `on_tether_loss`).
- **The expensive brain is on the ground.** The Starlink terminal, the
  ground box coordinator, and the LLM mission brain are all shared
  across drones and stay on the ground — a crashed drone costs a
  Pi and an airframe, never the expensive kit.
- **The LLM emits goals, never velocities.** The deterministic validator
  rejects anything outside the geofence / altitude band / separation
  rules. The CfC turns goals into flight. The tether carries goals
  down + telemetry + map deltas up. The Starlink carries the LLM
  uplink only.

## Onboard stack (one Pi)

```
libcamera ───► DronePoseProvider ──────────────────────────────────────┐
   perception/drone_pose_provider.{hpp,cpp}                            │
                                                                      ▼
MAVSDK (pose from FC EKF/VIO) ──► LiveVoxelMapper ──► occupancy grid (VoxelGrid)
control/fc_bridge/fc_bridge.cpp        core/live_voxel_mapper.cpp        core/grid.hpp
                                                │
                                                │ grid
                                                ▼
                                  ObservationBuilder
                                  control/cfc_pilot/observation.hpp
                                                │ (sector distances + state + goal)
                                                ▼
                                  CfCPilot (onnxruntime, carries hidden state)
                                  control/cfc_pilot/cfc_pilot.hpp
                                                │ (NavigationCommand)
                                                ▼
                                  SafetyFilter (deterministic geometric veto)
                                  control/cfc_pilot/cfc_pilot.hpp        ──── cfc_pilot.hpp
                                                │
                                                ▼
                                  FcBridge (NavigationCommand -> MAVSDK offboard)
                                  control/fc_bridge/fc_bridge.cpp
                                                │
                                                ▼
                                          PX4 / ArduPilot on Pixhawk

   failsafe machine (control/failsafe.hpp) vets every offboard setpoint:
       pose stale > 100 ms -> reject, HOLD / LAND
       CfC NaN or safety-filter veto sustained -> deterministic planner, then HOLD
       tether loss -> keep flying onboard, run link-loss action
       operator kill over fiber -> EMERGENCY_STOP, never gated

   tether-agent (link/tether_agent/tether_agent.{hpp,cpp}) runs alongside:
       up:   DroneState (pose, battery, map_delta, failsafe_state) at 5–20 Hz
       down: GroundCommand (goal | teleop | kill | mode change)
       heartbeat 200 ms; 1 s of silence -> LINK_LOSS (see docs/TETHER.md)
```

## Ground / server stack

```
            fiber (per drone, see docs/TETHER.md, link/tether_agent/README.md)
   drone-01 ──────────────┐
   drone-02 ──────────────┼──► ground box switch
   drone-03 ──────────────┘          │
                                     ▼
                          ground-side converters (one/drone)
                                     │
                                     ▼
                                  switch
                                     │
                                     ▼
                          coordinator (one process per drone)
                          ground/coordinator/ (Phase 5)
                                     │
       ┌─────────────────────────────┼─────────────────────────────┐
       ▼                              ▼                             ▼
  fleet view (operator)        deterministic validator         per-drone
       │                        (geofence + separation)        coordinator proc
       │                              │
       │                              │ goals (LLM-emitted; never velocities)
       │                              ▼
       │                        mission brain (self-hosted LLM + map fusion)
       │                        server/ (Phase 5)
       │                              │
       │                              │ WireGuard over Starlink
       │                              ▼
       │                        Starlink terminal (shared, Ku/Ka-band uplink)
       │                              │
       ▼                              ▼
   operator UI            remote / public endpoint (server holds it,
   (fleet view)           ground box dials out — Tailscale/Headscale OK)
```

The two scaling rules (see [FLEET.md](FLEET.md)) sit on top of this
diagram: **telemetry + map deltas scale** to many drones; **simultaneous
HD video does not** (video is on-demand). The Starlink terminal is the
one shared uplink — that's the "many drones, one terminal" property.

## Reference

- Onboard diagram: manual Part 0 (live version, this repo).
- Failsafe machine: `control/failsafe.hpp`.
- Tether contract: [`link/tether_agent/README.md`](../link/tether_agent/README.md),
  [`docs/TETHER.md`](TETHER.md).
- BOM: [`docs/HARDWARE.md`](HARDWARE.md).
- Fleet rules: [`docs/FLEET.md`](FLEET.md).
- Regulatory gate: [`docs/REGULATORY.md`](REGULATORY.md).
- Test ladder: [`docs/SAFETY.md`](SAFETY.md), [`tests/README.md`](../tests/README.md).