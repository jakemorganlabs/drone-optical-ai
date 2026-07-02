// Phase 5 (manual Part 6.1) — ground coordinator.
//
// One process per drone. Lives on the ground box and:
//   - dials ONE tether-agent (Phase 4) on the drone side over the fiber IP
//     (default 10.8.0.2:<TETHER_PORT>) and speaks the JSON-line wire format
//     defined by link/tether_agent/README.md (field-identical to tether.proto),
//   - receives drone telemetry (Pose, battery, failsafe state, map_delta),
//     one DroneState per JSON line,
//   - forwards goals / teleop / kill / mode down to the tether-agent as
//     JSON-line GroundCommand frames (oneof c: goal | teleop | kill | mode),
//   - exposes a tiny read-only fleet-view HTTP/JSON scrape on
//     127.0.0.1:8090/fleet (POSIX sockets + a hand-written JSON writer; no
//     HTTP framework dep).
//
// Pure C++17, POSIX sockets only. The wire format is pinned by the Phase 4
// tether-agent; see coordinator.cpp for the exact JSON shapes and
// link/tether_agent/tether.proto for the canonical field names.
//
//   TODO(phase7): systemd unit "dronectl-coordinator@.service" (one instance
//   per drone via %i -> --drone-id %i --tether 10.8.0.2:52000).

#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace dronectl {

// ---- POJOs mirroring the tether-agent wire schema (field-identical) ------------
// Duplicated from link/tether_agent/tether_agent.hpp on purpose: the
// coordinator is a *client* of the tether link and must not include the
// agent's server header (which would pull POSIX headers via its pimpl anyway).
// Keeping these standalone also lets the server-side Python emit the same
// field names without a C++ dep.

struct CoordPose {
    double x = 0.0, y = 0.0, z = 0.0;
    double yaw = 0.0, pitch = 0.0, roll = 0.0;  // deg
};

enum class FailsafeState : int {
    BOOT = 1, IDLE, ARMED, AUTONOMOUS, TELEOP, LINK_LOSS, RTL, LANDING, FAULT,
};

struct DroneState {
    std::string  drone_id;
    CoordPose    pose;
    float        battery = 1.0f;       // 0..1
    std::string  map_delta_b64;        // opaque; empty = no delta this tick
    FailsafeState state = FailsafeState::BOOT;
};

enum class CmdKind : int { Goal = 1, Teleop, Kill, Mode };
enum class FlightMode : int { AUTONOMOUS = 1, TELEOP, IDLE };

struct Waypoint   { CoordPose pose; std::string id; };
struct TeleopVel  { float vx=0, vy=0, vz=0, yaw_rate=0; };
struct KillCmd    { std::string reason; };
struct ModeChange { FlightMode mode = FlightMode::IDLE; };

struct GroundCommand {
    CmdKind kind = CmdKind::Kill;
    Waypoint   goal;
    TeleopVel  teleop;
    KillCmd    kill;
    ModeChange mode;
};

// ---- Coordinator --------------------------------------------------------------

struct CoordinatorConfig {
    std::string drone_id   = "drone-01";
    std::string tether_ip  = "10.8.0.2";   // drone-side fiber IP (manual Part 5.1)
    uint16_t    tether_port = 52000;        // Phase 4 default (link/tether_agent)
    uint16_t    fleet_port  = 8090;        // 127.0.0.1:8090/fleet scrape
    // Reconnect backoff for the tether dialer.
    int         reconnect_ms = 1000;
};

class Coordinator {
public:
    Coordinator();
    ~Coordinator();

    Coordinator(const Coordinator&)            = delete;
    Coordinator& operator=(const Coordinator&) = delete;

    // Inbound telemetry callback (worker thread). Called for every
    // DroneState JSON line received from the tether-agent.
    void on_telemetry(std::function<void(const DroneState&)> cb);

    // Start the tether-client worker + the fleet-view HTTP listener.
    // Returns false if the fleet socket cannot be bound.
    bool start(const CoordinatorConfig& cfg);

    // Signal the worker to stop and join. Idempotent.
    void stop();

    // ---- Synchronous command senders (thread-safe) ----------------------
    // Each enqueues a single JSON-line GroundCommand on the tether socket.
    // Returns true if the frame was handed to the kernel (send_all ok);
    // false on hard socket error / not currently connected (the caller may
    // retry; the worker keeps trying to reconnect).
    bool send_goal(const Waypoint& w);
    bool send_teleop(const TeleopVel& v);
    bool send_kill(const std::string& reason);
    bool send_mode(FlightMode m);

    // ---- Fleet view (read by the HTTP worker) ----------------------------
    // Returns a snapshot of this coordinator's last-seen drone state. Safe
    // to call from any thread.
    DroneState last_state() const;
    bool       have_state() const;

private:
    struct Impl;
    Impl* impl_;
    void tether_worker();
    void fleet_worker();
};

}  // namespace dronectl