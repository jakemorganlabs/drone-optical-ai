# ground/coordinator (Phase 5)

The per-fleet ground coordinator (Option A manual Part 6.3). One process
holds N drone sessions, fans re-tasked goal plans out to per-drone
tether-agent ports, fuses per-drone map deltas back into a shared world
model the LLM can re-reason over, and runs an explicit **deterministic
`Validator`** between the LLM and the drones so no out-of-geofence /
out-of-altitude-band / too-close goal ever reaches a drone — regardless of
what the LLM said.

## Topology (manual Part 6.1)

```
                                                                 Starlink WAN
   drone-01 ──fiber──►┐                                                      
   drone-02 ──fiber──►├──[switch]── ground box ── wg0 ──────────────► self-hosted AI server
   drone-NN ──fiber──►┘              [coordinator +        (Tier-3 VPN     [LLM mission brain]
                                     tether clients]       over Starlink)  server/mission_brain.py
                                     :7400 HTTP]                            :7000 HTTP
```

- **Tier-3 (non-flight-critical)** traffic rides `wg0`: goal plans down,
  telemetry + map deltas + re-task context up.
- **Nothing in the closed flight loop rides `wg0`.** If Starlink or `wg0`
  drops, every drone continues its last goal on the onboard CfC; the operator
  still has full fiber teleop. No freeze, no fall (manual Part 6.4).
- Each drone's fiber + converter + switch port is its own `/30`; the shared
  resource is the **one Starlink terminal** (manual Part 6.4 / docs/FLEET.md).

## Files

- `coordinator.hpp` — public `Validator`, `WorldModel`, `TetherClient`,
  `Coordinator` classes + POJOs (`CoordPose`, `Goal`, `Geofence`,
  `GoalPlan`, `DroneTelemetry`).
- `coordinator.cpp` — stdlib + pthreads implementation; tiny hand-rolled
  HTTP/1.0 listener + JSON-line TCP tether client. No Boost / grpc / protobuf.
- `../wireguard/wg0.example.conf` — annotated WireGuard config (Part 6.2).

## Contract

Tether-agent wire protocol: see `link/tether_agent/README.md` +
`link/tether_agent/tether.proto`. The coordinator does **not** include any
Phase 4 header — it speaks the JSON-line TCP protocol over its own
`TetherClient`.

Listening HTTP endpoints (bound on `127.0.0.1:7400`, Phase 5 scaffold):

| Method | Path        | Body                                                                 | Returns |
|--------|-------------|----------------------------------------------------------------------|---------|
| POST   | `/plan`     | `{"goals":[...],"geofence":{...}}`                                   | `{"ok":true,"goals":N}` or `422 {"ok":false,"reason":"..."}` |
| GET    | `/world`    | —                                                                    | `{"drones":[{"drone_id":...,"pose":...,"battery":...,"state":...,"has_map_delta":bool}]}` |
| POST   | `/kill`     | —                                                                    | `{"ok":true,"killed":true}` — sends a kill to **every** drone; bypasses the validator (Part 7: highest priority, never gated) |

The `/plan` handler runs the C++ `Validator` (geofence polygon via ray-cast,
altitude band, drone-separation) and only forwards goals that pass. The
server-side `server/validator.py` is a second, independent gate — both must
pass for a goal to ship.

## Build

Wired through `ground/Makefile` (NOT the root Makefile — this is Python + JSON
config + a small C++ binary outside the root `make`/`make test` flow):

```bash
cd ground && make coordinator      # builds ground/coordinator/coordinator
./coordinator/coordinator --help
./coordinator/coordinator --http-port 7400 --drone-id drone-01 --drone-port 52000
```

The binary is `-Wall -Wextra` clean and depends only on stdlib + pthreads.

## Backhaul-loss behavior (manual Part 6.4)

The coordinator **never blocks a drone on a Starlink round-trip.** Goals are
pushed to a session the instant they are validated; telemetry is pulled
asynchronously. If the LLM server (or the coordinator's HTTP socket to it)
is unreachable, the coordinator simply stops emitting NEW goals — each drone
keeps its last goal on the onboard CfC. `make coordinator` + `python3
mission_brain.py` is the full Starlink-up path; killing `wg0` mid-mission is
the degradation test.

## TODOs for Phase 7 packaging

- Swap the hand-rolled HTTP server for a real framework (fastapi / crow /
  cpp-httplib) once deps are vendored.
- Swap the JSON-line `TetherClient` for a gRPC bidi client against the
  upgraded tether-agent once `protobuf` + `grpc` are vendored.
- Replace the opaque `map_delta_b64` "hold-and-timestamp" fusion with a real
  voxel merge.
- systemd unit + first-boot wizard wiring (Part 8).