// Phase 4 scaffold: fiber-optic tether datalink agent.
//
// Single-file C++17 stub using POSIX sockets + a JSON-line transport.
// It is NOT a real gRPC server yet; the manual (Part 5.2) explicitly
// permits "gRPC bidi OR plain UDP + protobuf" — we use JSON-line TCP
// so this compiles with zero extra deps.
//
//   TODO(phase5): swap to gRPC + protobuf once deps are vendored.
//
// Link-loss contract (manual Part 5.3):
//   - heartbeat every 200 ms
//   - 1 s of silence from the ground -> on_tether_loss() fires; agent
//     stays bound and listening in LINK_LOSS mode (does NOT stop)
//   - on reconnect, resync (idempotent goal ids handled by callers)
//
// The failsafe hook signature on_tether_loss(std::function<void()>) is
// intentionally decoupled from the Failsafe class being defined by the
// Phase 6 agent — do NOT include control/failsafe.hpp here.

#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace dronectl {

// ---- POJOs mirroring tether.proto -------------------------------------------------
// Kept as plain structs (no protobuf dep) so callers in control/ and
// server/ can hold/pass them without wiring grpc. Field names track the
// .proto for an easy mechanical swap later.

struct Pose {
    double x = 0.0, y = 0.0, z = 0.0;
    double yaw = 0.0, pitch = 0.0, roll = 0.0;  // deg
};

enum class FailsafeState : int {
    BOOT = 1,
    IDLE,
    ARMED,
    AUTONOMOUS,
    TELEOP,
    LINK_LOSS,
    RTL,
    LANDING,
    FAULT,
};

struct DroneState {
    std::string  drone_id;
    Pose         pose;
    float        battery = 1.0f;     // 0..1
    // map_delta omitted from the JSON scaffold by default (bytes); callers
    // may set it but it round-trips as a base64-ish opaque string.
    std::string  map_delta_b64;      // empty = no delta this tick
    FailsafeState state = FailsafeState::BOOT;
};

enum class CmdKind : int {
    Goal = 1,
    Teleop,
    Kill,
    Mode,
};

enum class FlightMode : int {
    AUTONOMOUS = 1,
    TELEOP,
    IDLE,
};

struct Waypoint {
    Pose        pose;
    std::string id;                  // idempotent goal id
};

struct TeleopVelocity {
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;   // m/s, body frame
    float yaw_rate = 0.0f;                     // deg/s
};

struct Kill {
    std::string reason;             // informational only
};

struct ModeChange {
    FlightMode mode = FlightMode::IDLE;
};

struct GroundCommand {
    CmdKind kind = CmdKind::Kill;   // discriminator; tagged-union-ish
    Waypoint       goal;
    TeleopVelocity teleop;
    Kill           kill;
    ModeChange     mode;
};

// ---- TetherAgent -----------------------------------------------------------------

class TetherAgent {
public:
    TetherAgent();
    ~TetherAgent();

    TetherAgent(const TetherAgent&)            = delete;
    TetherAgent& operator=(const TetherAgent&) = delete;

    // Telemetry source the agent polls each heartbeat tick (thread-safe
    // wrt the agent's worker thread; the source must be thread-safe).
    void set_telemetry_source(std::function<DroneState()> src);

    // Ground-command dispatcher: every parsed GroundCommand from the wire
    // is handed to this callback on the agent's worker thread.
    void on_ground_command(std::function<void(const GroundCommand&)> cb);

    // Link-loss / recovery hooks. on_tether_loss fires once on the
    // transition into LINK_LOSS (1 s of silence); on_tether_recover fires
    // once on the first good frame after a loss. Neither disconnects the
    // listener; the agent keeps running.
    void on_tether_loss(std::function<void()> cb);
    void on_tether_recover(std::function<void()> cb);

    // Bind + listen on 127.0.0.1:port (default 52000) and start the
    // worker thread. Returns false if the socket cannot be bound.
    bool start(uint16_t port = 52000);

    // Signal the worker to stop and join it. Idempotent.
    void stop();

private:
    struct Impl;
    Impl* impl_;   // pimpl keeps POSIX headers out of this public header
    void worker_loop(uint16_t port);  // accept/dispatch loop
};

}  // namespace dronectl