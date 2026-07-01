# tether-agent (Phase 4 scaffold)

The fiber-optic tether datalink between the onboard Pi pilot and the
ground box / coordinator. Per the Option A build manual (Part 5), this is
the **jam-immune leash**: telemetry up, goals + teleop overrides down. It
is **not** in the flight-critical loop — if it drops, the onboard CfC
pilot (Part 4) keeps flying toward the last goal, then runs the configured
link-loss action. The drone does not fall.

This is the **Phase 4 scaffold**: a single-file C++17 server using POSIX
sockets and a JSON-line transport. It deliberately has **no protobuf /
gRPC dependency** so the link can be exercised today with zero extra
libraries. The `.proto` contract (`tether.proto`) lives next to this file
and is the path forward.

- `tether.proto` — eventual gRPC bidi contract (manual Part 5.2)
- `tether_agent.hpp` — public `TetherAgent` class + POJOs mirroring the proto
- `tether_agent.cpp` — POSIX socket server; compiles to a self-test binary

```
   TODO(phase5): swap to gRPC + protobuf once deps are vendored.
```

## Wire format (JSON lines)

All messages are single-line JSON objects terminated by `\n`. No
trailing newlines within a frame; no binary framing in the scaffold.

### Up: DroneState (drone → ground), ~5–20 Hz

```json
{"drone_id":"drone-01",
 "pose":{"x":1.5,"y":-2.0,"z":3.25,"yaw":45.0,"pitch":0.0,"roll":0.0},
 "battery":0.87,
 "map_delta":"",
 "state":4}
```

Field mapping (`tether.proto` field number  → JSON key):

| # | proto field  | JSON key    | type    | notes |
|---|--------------|-------------|---------|-------|
| 1 | `drone_id`   | `drone_id`  | string  | |
| 2 | `pose`       | `pose`      | object  | fields `x,y,z,yaw,pitch,roll` (doubles) |
| 3 | `battery`    | `battery`   | number  | 0.0–1.0 |
| 4 | `map_delta`  | `map_delta` | string  | opaque; empty string = no delta this tick (base64 round-trips in the gRPC upgrade) |
| 5 | `state`      | `state`     | int     | `FailsafeState` enum value (see below) |

### Down: GroundCommand (ground → drone)

A `oneof c` — exactly one of the four bodies is present. Recognized by
the **presence of its key**; absent keys are ignored.

```json
{"goal":{"pose":{"x":10,"y":2,"z":3,"yaw":0,"pitch":0,"roll":0},"id":"g-001"}}
{"teleop":{"vx":0.5,"vy":-0.2,"vz":0.0,"yaw_rate":10.0}}
{"kill":{"reason":"operator"}}
{"mode":{"mode":1}}
```

| tag key | body fields | notes |
|---------|-------------|-------|
| `goal`   | `pose` (object) + `id` (string) | idempotent goal id for dedup on reconnect |
| `teleop` | `vx, vy, vz` (m/s body) + `yaw_rate` (deg/s) | operator override |
| `kill`   | `reason` (string, optional) | **highest priority, never gated** — maps to `EMERGENCY_STOP` |
| `mode`   | `mode` (int) | `1`=AUTONOMOUS, `2`=TELEOP, `3`=IDLE |

Unknown / malformed lines are dropped silently by the parser; only
well-formed commands are dispatched to the `on_ground_command` callback.

### Enum values

`FailsafeState` (`DroneState.state`):

```
BOOT=1 IDLE=2 ARMED=3 AUTONOMOUS=4 TELEOP=5 LINK_LOSS=6 RTL=7 LANDING=8 FAULT=9
```

(0 is reserved/unused; proto3 enum zero is `FAILSAFE_UNSPECIFIED` and is
not emitted by the agent.)

## Environment variables

| var          | default | notes |
|--------------|---------|-------|
| `TETHER_PORT` | `52000` | TCP port the agent binds on `127.0.0.1` |

The bind address is fixed at `127.0.0.1` per the scaffold — the fiber
converter puts the drone on a private `/30` (`10.8.0.0/30` per manual
Part 5.1) and a future phase moves the bind address configurable. For the
Phase 4 scaffold, ground clients connect to the loopback forwarder.

## Link-loss contract (manual Part 5.3)

- Heartbeat cadence: **200 ms** telemetry frames.
- If the agent sees **1 s of silence** from the ground (no inbound line),
  it fires `on_tether_loss()` **once** and enters `LINK_LOSS` mode.
- **It does not disconnect or stop.** It keeps the listener bound, keeps
  emitting heartbeat telemetry, and keeps the session socket open so the
  ground can re-attach on the same TCP connection if the silence was a
  transient stall. If the peer resets, it closes the dead socket and
  loops back to `accept()` for a fresh reconnect.
- On the first good inbound frame after a loss, it fires
  `on_tether_recover()` **once** and resumes normal mode.
- Callers (the failsafe machine, Part 7) wire `on_tether_loss` /
  `on_tether_recover` into the state machine. The agent itself never
  imports `control/failsafe.hpp` — the hook is a plain `std::function<void()>`
  so Phase 6 can land the `Failsafe` class without recompiling this file.

## API (`tether_agent.hpp`)

```cpp
dronectl::TetherAgent agent;
agent.set_telemetry_source([]{ return drone_state_now(); });
agent.on_ground_command([](const dronectl::GroundCommand& c){ dispatch(c); });
agent.on_tether_loss([]{ failsafe.on_tether_loss(); });
agent.on_tether_recover([]{ failsafe.on_tether_recover(); });
agent.start(52000);   // binds 127.0.0.1:52000, spawns worker thread
// ...
agent.stop();
```

The class is pimpl-bodied so POSIX headers do not leak into the public
header; callers in `control/`, `ground/`, and `server/` can include it
without picking up `<sys/socket.h>`.

## Build

This is **not** wired into the main build targets (`all`, `test`) — the
manual's build order (Part 9.6 step 5) puts link plumbing *after*
autonomy. There is an optional Makefile target:

```bash
make tether-test     # builds build/tether_agent (self-test binary)
./build/tether_agent # listens 3s on 127.0.0.1:52000 then stops
TETHER_PORT=52001 ./build/tether_agent
```

`tether-test` is order-only-prereq'd on `| $(BUILD_DIR)` and never breaks
`make` or `make test`.

## gRPC upgrade path (Phase 5+)

The JSON-line transport is intentionally field-identical to
`tether.proto`. The upgrade is mechanical:

1. Vendor `protobuf` + `grpc` into `third_party/` and wire a `protoc`
   step that emits `tether.pb.cc` / `tether.grpc.pb.cc`.
2. Replace the JSON serializer/parser bodies in `tether_agent.cpp` with
   proto encode/decode; the `DroneState` / `GroundCommand` POJOs become
   thin wrappers over the generated types (or are deleted in favor of the
   generated types).
3. Swap the POSIX `accept` loop for a `grpc::Server` with a bidi
   `Service` implementing `rpc Telemetry(stream DroneState) returns
   (stream GroundCommand)`.
4. Keep the same `on_*` callback surface so `control/pilot_main.cpp`
   and the Phase 6 failsafe wiring do not change.

Until that swap, the scaffold is the contract: any client speaking these
JSON lines will keep working through the upgrade because the field names
and semantics are pinned by `tether.proto`.