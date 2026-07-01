#pragma once

// Phase 2: flight-controller bridge (MAVSDK).
//
// FcBridge streams FC EKF pose/attitude into the perception provider
// (DronePoseProvider::push_pose) and accepts NavigationCommand outputs from the
// onboard pilot, translating them to MAVSDK offboard setpoints.
//
// Safety interlocks (Part 3.3 + Part 7):
//   * VELOCITY_SETPOINT is rejected unless the Failsafe gate approves AND the
//     last pose is fresher than 100 ms. The command collapses to HOLD otherwise.
//   * EMERGENCY_STOP is never gated (operator kill must always go through).
//
// This header intentionally keeps MAVSDK types out of the public surface
// (PIMPL via MavState) so it can be included where mavsdk is unavailable; the
// .cpp pulls in MAVSDK and implements the bridge.

#include <memory>
#include <mutex>
#include <string>
#include <chrono>

#include "pose.hpp"
#include "navigation.hpp"
#include "perception/drone_pose_provider.hpp"
#include "control/failsafe.hpp"   // Phase 6/7 Failsafe::allow_autonomy(const Pose&)

class FcBridge {
public:
    // `provider` receives EKF pose updates; `failsafe` gates velocity commands
    // (may be nullptr to disable the gate during early bring-up). The caller
    // owns both and must keep them alive for the lifetime of the bridge.
    FcBridge(DronePoseProvider& provider, const Failsafe* failsafe = nullptr);

    // Defined in the .cpp so the incomplete MavState (holds MAVSDK plugins) can
    // be destroyed safely; keeps MAVSDK types out of this header.
    ~FcBridge();

    // Connect to the FC (default SITL / mavlink-router onboard endpoint). Returns
    // false if no autopilot is discovered within the timeout.
    bool connect(const std::string& url = "udp://:14540");

    // Issue one control tick's command to the FC. Safe to call from the pilot
    // control thread only (not re-entrant with itself; telemetry callbacks run
    // on MAVSDK's own thread and only touch provider_ / pose state under mutex).
    void send(const NavigationCommand& cmd);

    // True after a successful connect().
    bool connected() const { return connected_; }

private:
    // Merge the latest position + attitude telemetry into a Pose and push it
    // into the perception provider. Called on the MAVSDK telemetry thread
    // (caller holds pm_).
    void push_if_ready();

    // Start offboard mode if it isn't already active. Seeds a zero-velocity
    // setpoint first (MAVSDK requires a setpoint be armed before start()).
    void ensure_offboard();

    DronePoseProvider& provider_;
    const Failsafe* failsafe_;   // optional; may be nullptr

    // PIMPL-style MAVSDK state. Defined in the .cpp so the MAVSDK headers stay
    // out of this public header.
    struct MavState;
    std::unique_ptr<MavState> mav_;
    bool connected_ = false;

    // Latest pose fragments, written from MAVSDK telemetry callbacks.
    mutable std::mutex pm_;
    Vec3 last_pos_{};
    struct Att { float yaw = 0.0f, pitch = 0.0f, roll = 0.0f; } last_att_{};
    bool pos_ok_ = false;
    bool att_ok_ = false;
    std::chrono::steady_clock::time_point last_pose_time_{};
};