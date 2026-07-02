#pragma once

// Phase 3 (manual Part 4.4): the onboard CfC pilot + deterministic safety
// filter.
//
// Per control tick:
//   1. build_observation(grid, pose, vel, goal)  -> compact state vector
//   2. ONNX inference (single step) with explicit hidden-state carry
//   3. map tanh outputs to a velocity/yaw-rate NavigationCommand
//   4. safety_filter(cmd, grid, pose)            -> deterministic geometric veto
//
// Build gate: onnxruntime (ORT) is OPTIONAL. The ORT C++ headers are detected
// with `#if __has_include(<onnxruntime_cxx_api.h>)`, which the compiler
// evaluates per-TU before parsing anything below it. When found, the macro
// DRONECTL_HAVE_ORT is defined for the rest of this TU and the full onnxruntime
// C++ inference path is compiled in, exactly as the manual shows. The build
// may also pass -DDRONECTL_HAVE_ORT explicitly (Make ORT_PROBE / CMake
// find_package) to force the path on even when the header isn't on the default
// search path. When ORT is absent, this header still parses cleanly with
// -Wall -Wextra: step() returns a HOLD NavigationCommand, reset() is a no-op,
// has_model() is false. The onboard loop (Part 4.5) is the same binary either
// way; it just holds in place until an ORT build is dropped in.
//
// `safety_filter` is a public static helper so it can be unit-tested without
// an ONNX session (tests/cfc_safety_test.cpp). The net proposes, geometry
// vetoes: if the commanded direction hits an occupied voxel within the
// braking distance, the command collapses to HOLD.
//
// Manual sketch used `VoxelGrid`; the repo's grid class is `DualVoxelGrid`
// (core/grid.hpp), so step()/safety_filter take `const DualVoxelGrid&` and
// use get_static_voxel + get_dynamic_voxel, world_to_voxel, voxel_to_index,
// get_config. NavigationCommand fields (type/velocity/yaw_rate/yaw/timestamp)
// and the enum (VELOCITY_SETPOINT/HOLD/...) come from core/navigation.hpp.

#include "observation.hpp"   // build_observation, Observation, OBS_DIM
#include "navigation.hpp"     // NavigationCommand, Vec3
#include "grid.hpp"           // DualVoxelGrid, GridConfig
#include "pose.hpp"           // Pose

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

// Auto-detect onnxruntime via __has_include unless the build already forced
// the macro on (e.g. ORT headers are on a non-default search path the build
// added). This keeps the header self-contained and -Wall -Wextra clean either
// way.
#if !defined(DRONECTL_HAVE_ORT) && __has_include(<onnxruntime_cxx_api.h>)
#define DRONECTL_HAVE_ORT 1
#endif

#if defined(DRONECTL_HAVE_ORT)
#include <onnxruntime_cxx_api.h>
#endif

class CfCPilot {
public:
    // `max_v` scales the tanh policy output to physical m/s; `yaw_max_dps`
    // scales the yaw-rate output to deg/s. Both must match the training-time
    // normalization (manual Part 4.1/4.2).
    CfCPilot(const std::string& model_path, int hidden_units, float max_v,
             float yaw_max_dps = 45.0f)
#if defined(DRONECTL_HAVE_ORT)
        : env_(ORT_LOGGING_LEVEL_WARNING, "cfc"),
          session_(env_, model_path.c_str(), Ort::SessionOptions{}),
          U_(hidden_units),
#else
        : U_(hidden_units),
#endif
          max_v_(max_v),
          yaw_max_dps_(yaw_max_dps),
          hx_(static_cast<size_t>(hidden_units), 0.0f),
          model_path_(model_path) {
        (void)model_path;  // held in model_path_ for diagnostics; ORT ctor used it
    }

    // One control tick: observation -> ONNX -> velocity command -> safety filter.
    NavigationCommand step(const DualVoxelGrid& grid, const Pose& pose,
                           const Vec3& vel, const Vec3& goal, float dt) {
#if defined(DRONECTL_HAVE_ORT)
        Observation o = build_observation(grid, pose, vel, goal);

        // --- ONNX inference (single step, explicit hidden-state carry) ---
        Ort::MemoryInfo mem =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::array<int64_t, 2> obs_shape{1, OBS_DIM};
        std::array<int64_t, 2> hx_shape{1, static_cast<int64_t>(U_)};
        std::array<int64_t, 1> ts_shape{1};
        float ts = dt;
        std::vector<Ort::Value> inputs;
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem, o.data.data(), OBS_DIM, obs_shape.data(), 2));
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem, hx_.data(), U_, hx_shape.data(), 2));
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem, &ts, 1, ts_shape.data(), 1));
        const char* in_names[]  = {"obs", "hx", "ts"};
        const char* out_names[] = {"act", "hx_out"};
        auto out = session_.Run(Ort::RunOptions{nullptr},
                                in_names, inputs.data(), 3,
                                out_names, 2);

        const float* act = out[0].GetTensorData<float>();
        const float* hxn = out[1].GetTensorData<float>();
        std::copy(hxn, hxn + U_, hx_.begin());   // carry hidden state

        // Map tanh outputs to a velocity + yaw-rate command.
        NavigationCommand cmd;
        cmd.type = NavigationCommand::VELOCITY_SETPOINT;
        cmd.velocity = Vec3(act[0] * max_v_, act[1] * max_v_, act[2] * max_v_);
        cmd.yaw_rate = act[3] * yaw_max_dps_;
        cmd.timestamp = std::chrono::steady_clock::now();

        // Deterministic geometric veto.
        return safety_filter(cmd, grid, pose);
#else
        // No ORT in this build: hold in place. Keeps the loop runnable on a
        // dev machine so the bring-up / failsafe wiring is exercised
        // without a trained model. The pilot main loop logs this case.
        (void)grid;
        (void)pose;
        (void)vel;
        (void)goal;
        (void)dt;
        NavigationCommand cmd;
        cmd.type = NavigationCommand::HOLD;
        cmd.timestamp = std::chrono::steady_clock::now();
        return cmd;
#endif
    }

    // Clear the carried hidden state (e.g. on a goal change or after a fault).
    void reset() {
#if defined(DRONECTL_HAVE_ORT)
        std::fill(hx_.begin(), hx_.end(), 0.0f);
#else
        // No-op in the no-ORT path; hx_ isn't used.
#endif
    }

    // Whether this pilot is actually running ONNX inference (true only when
    // ORT was found at build time AND a model was loaded). Used by the pilot
    // main loop to decide whether to fall back to the deterministic planner.
    bool has_model() const {
#if defined(DRONECTL_HAVE_ORT)
        return true;
#else
        return false;
#endif
    }

    const std::string& model_path() const { return model_path_; }

    // Deterministic geometric veto (manual Part 4.4). Public + static so it
    // can be unit-tested without an ONNX session (tests/cfc_safety_test.cpp).
    // If the commanded direction hits an occupied voxel within the braking
    // distance, the command collapses to HOLD. The net proposes; geometry
    // vetoes. Occupancy test:
    //   grid.get_static_voxel(idx) + grid.get_dynamic_voxel(idx) > 0.5f
    static NavigationCommand safety_filter(NavigationCommand cmd,
                                           const DualVoxelGrid& grid,
                                           const Pose& pose) {
        Vec3 v = cmd.velocity;
        float speed = length(v);
        if (speed < 1e-3f) return cmd;

        Vec3 dir = v * (1.0f / speed);
        const GridConfig& cfg = grid.get_config();
        // Braking distance assuming ~2 m/s^2 max decel, plus one voxel of margin.
        float brake = 0.5f * speed * speed / 2.0f + cfg.voxel_size;

        for (float d = cfg.voxel_size; d < brake; d += cfg.voxel_size) {
            Vec3 wp = pose.position + dir * d;
            int ix, iy, iz;
            if (!grid.world_to_voxel(wp, ix, iy, iz)) break;
            int idx = grid.voxel_to_index(ix, iy, iz);
            if (grid.get_static_voxel(idx) + grid.get_dynamic_voxel(idx) > 0.5f) {
                cmd.velocity = Vec3(0.0f, 0.0f, 0.0f);
                cmd.type = NavigationCommand::HOLD;
                return cmd;
            }
        }
        return cmd;
    }

#if defined(DRONECTL_HAVE_ORT)
private:
    Ort::Env env_;
    Ort::Session session_;
#endif
    int U_;
    float max_v_;
    float yaw_max_dps_;
    std::vector<float> hx_;
    std::string model_path_;
};

// Drop the build-gate macro so it doesn't leak into including TUs.
#ifdef DRONECTL_HAVE_ORT
#undef DRONECTL_HAVE_ORT
#endif