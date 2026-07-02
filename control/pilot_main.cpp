// Phase 3 (manual Part 4.5): the onboard pilot main loop.
//
// Occupancy grid -> observation -> CfC (ONNX) -> safety filter ->
// NavigationCommand -> FcBridge. Ticks at 20 Hz.
//
// Composition:
//   DronePoseProvider   (perception)  -- camera frames + FC pose
//   DualVoxelGrid       (core)         -- occupancy surface the CfC reasons over
//   FcBridge            (control)      -- MAVSDK offboard (Phase 2; optional)
//   CfCPilot            (control)      -- ONNX + safety filter (Phase 3; optional)
//   Failsafe            (control)      -- gates offboard setpoints (Phase 7)
//
// Both MAVSDK and onnxruntime are OPTIONAL. A `make` without either must still
// build and run. The CfC block is gated with `__has_include(<onnxruntime_cxx_api.h>)`;
// when ORT is absent, the loop runs full-tilt but the pilot returns HOLD
// every tick and logs "CfC model unavailable; sending HOLD". The FcBridge TU
// is linked in by the build system only when MAVSDK is available; without it,
// the bridge TU is not linked and `bridge` is not constructed (see the
// `#ifdef DRONECTL_HAVE_MAVSDK` guards below).
//
// The model path is configurable via the CFC_MODEL env var (default
// /opt/dronectl/cfc_policy.onnx) so the same binary can be pointed at a
// trained model without rebuilding.
//
// NOTE on LiveVoxelMapper: the manual Part 4.5 sketch composes LiveVoxelMapper.
// In this repo LiveVoxelMapper is defined in core/live_voxel_mapper.cpp
// alongside that file's own main() (it's the embedded-mapper demo TU). To
// reuse the class here without a duplicate-main ODR violation we rename its
// main out of the way with a macro before including the .cpp (see below).
// No edits are made to the Phase 0 file. LiveVoxelMapper owns the DualVoxelGrid
// the CfC + safety filter consume each tick.

#include "grid.hpp"                 // GridConfig, DualVoxelGrid
#include "pose.hpp"                 // Pose, Vec3
#include "navigation.hpp"          // NavigationCommand
#include "perception/drone_pose_provider.hpp"
#include "control/failsafe.hpp"
#include "control/cfc_pilot/observation.hpp"
#include "control/cfc_pilot/cfc_pilot.hpp"
#ifdef DRONECTL_HAVE_MAVSDK
#include "control/fc_bridge/fc_bridge.hpp"
#endif

// Bring the LiveVoxelMapper class into this TU. core/live_voxel_mapper.cpp is
// a single-TU module that ALSO defines main(); we rename that main out of the
// way with a macro so pilot_main's own main is the program entry point. This
// keeps us off the Phase 0 file (no edits to live_voxel_mapper.cpp) while
// letting pilot_main instantiate LiveVoxelMapper directly per manual Part 4.5.
#define main live_voxel_mapper_main_unused
#include "core/live_voxel_mapper.cpp"
#undef main

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_running{true};

void handle_signal(int sig) {
    (void)sig;
    g_running.store(false);
}

using clock = std::chrono::steady_clock;

NavigationCommand hold_command() {
    NavigationCommand cmd;
    cmd.type = NavigationCommand::HOLD;
    cmd.timestamp = clock::now();
    return cmd;
}
}  // namespace

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const std::string model_path = []() -> std::string {
        const char* env = std::getenv("CFC_MODEL");
        return env ? std::string(env) : std::string("/opt/dronectl/cfc_policy.onnx");
    }();

    // Grid sized for a ~30 m horizon at 0.4 m resolution (manual Part 4.5).
    // Smaller than the production 96^3 so the dev-Pi mapper stays light.
    GridConfig cfg(48, 0.4f, 30.0f);
    LiveVoxelMapper mapper(cfg);
    auto provider = std::make_shared<DronePoseProvider>(640, 480, 60.0f);
    Failsafe failsafe;
#ifdef DRONECTL_HAVE_MAVSDK
    FcBridge bridge(*provider, &failsafe);
#endif

    CfCPilot pilot(model_path, /*hidden_units=*/48, /*max_v=*/5.0f);

    std::cout << "[pilot] CFC_MODEL=" << model_path
              << "  has_model=" << (pilot.has_model() ? "yes" : "no")
              << "\n";
    if (!pilot.has_model()) {
        std::cout << "[pilot] CfC model unavailable; sending HOLD every tick\n";
    }

#ifdef DRONECTL_HAVE_MAVSDK
    if (bridge.connect("udp://:14540")) {
        std::cout << "[pilot] FC bridge connected\n";
    } else {
        std::cout << "[pilot] FC bridge not connected (MAVSDK unavailable or no FC); "
                     "commands will be dropped\n";
    }
#else
    std::cout << "[pilot] built without MAVSDK; FcBridge omitted (no offboard "
                 "setpoints will be sent)\n";
#endif

    if (!provider->start()) {
        std::cerr << "[pilot] failed to start perception provider\n";
        return 1;
    }
    if (!mapper.start()) {
        std::cerr << "[pilot] failed to start voxel mapper\n";
        provider->stop();
        return 1;
    }

    // Goal: held at the current pose until the tether/LLM (Parts 5-6) pushes
    // a new one. The CfC therefore steers "nowhere" until a goal arrives.
    Vec3 goal(0.0f, 0.0f, 0.0f);
    Pose pose;
    if (provider->get_latest_pose(pose)) {
        goal = pose.position;
    }

    auto last = clock::now();
    while (g_running.load()) {
        auto now = clock::now();
        float dt = std::chrono::duration<float>(now - last).count();
        last = now;

        if (!provider->get_latest_pose(pose)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        NavigationCommand cmd;
        if (pilot.has_model() && failsafe.allow_autonomy(pose)) {
            const DualVoxelGrid* grid = mapper.get_voxel_grid();
            if (grid) {
                Vec3 vel(0.0f, 0.0f, 0.0f);
                cmd = pilot.step(*grid, pose, vel, goal, dt);
            } else {
                cmd = hold_command();
            }
        } else if (!pilot.has_model()) {
            // No ORT in this build: emit HOLD and keep the loop alive so the
            // perception/failsafe wiring is exercised. Logged once at startup.
            cmd = hold_command();
        } else {
            // Autonomy not allowed: HOLD/LAND. The FcBridge will map HOLD to
            // a zero-velocity setpoint, or refuse it if pose is stale.
            cmd = hold_command();
        }

#ifdef DRONECTL_HAVE_MAVSDK
        bridge.send(cmd);
#else
        // No FcBridge in this build; nothing to send. Touch `cmd` to keep the
        // optimizer from eliding the whole branch in debug builds.
        (void)cmd;
#endif

        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20 Hz
    }

    std::cout << "[pilot] shutting down\n";
    mapper.stop();
    provider->stop();
#ifdef DRONECTL_HAVE_MAVSDK
    // FcBridge destructor drops the MAVSDK connection cleanly.
#endif
    return 0;
}