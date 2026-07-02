// Phase 3 unit tests for CfCPilot::safety_filter (manual Part 4.4).
//
// exercise the geometric veto against a tiny DualVoxelGrid both with and
// without onnxruntime installed in the build environment. Build it by hand:
//   g++ -std=c++17 -Wall -Wextra -I. -o build/cfc_safety_test \
//       tests/cfc_safety_test.cpp

#include "control/cfc_pilot/cfc_pilot.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

static int failures = 0;
#define CHECK(cond) do { \
    if(!(cond)) { std::cerr << "CHECK FAILED: " << #cond \
        << " at " << __FILE__ << ":" << __LINE__ << "\n"; ++failures; } \
} while(0)

// GridConfig(8, 0.5f, 4.0f): origin (0,0,0), half_size = 2.0 m, horizon 4.0 m.
// A drone at the origin has ~2 m of room before leaving the grid along any axis.
static void test_command_toward_obstacle_is_vetoed() {
    GridConfig cfg(8, 0.5f, 4.0f);
    DualVoxelGrid grid(cfg);

    // Seed an occupied voxel straight ahead (+x) at 1.0 m.
    int ix, iy, iz;
    bool ok = grid.world_to_voxel(Vec3(1.0f, 0.0f, 0.0f), ix, iy, iz);
    CHECK(ok);
    grid.accumulate_static(grid.voxel_to_index(ix, iy, iz), 1.0f);

    // Build a velocity command straight at the obstacle.
    NavigationCommand cmd;
    cmd.type = NavigationCommand::VELOCITY_SETPOINT;
    cmd.velocity = Vec3(3.0f, 0.0f, 0.0f);   // clearly nonzero + toward obstacle
    Pose pose(Vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f);

    NavigationCommand out = CfCPilot::safety_filter(cmd, grid, pose);

    // Channel 1 (endo-beta): geographic veto collapses the command to HOLD
    // (manual Part 4.4): direction hits an occupied voxel within braking
    // distance, so velocity -> 0 and type -> HOLD.
    CHECK(out.type == NavigationCommand::HOLD);
    CHECK(std::fabs(out.velocity.x) < 1e-5f);
    CHECK(std::fabs(out.velocity.y) < 1e-5f);
    CHECK(std::fabs(out.velocity.z) < 1e-5f);
}

// Command toward open space passes through unchanged.
static void test_command_toward_open_space_passes() {
    GridConfig cfg(8, 0.5f, 4.0f);
    DualVoxelGrid grid(cfg);   // empty grid: no occupied voxels anywhere

    NavigationCommand cmd;
    cmd.type = NavigationCommand::VELOCITY_SETPOINT;
    cmd.velocity = Vec3(2.0f, 0.0f, 0.0f);   // toward +x, which is open
    Pose pose(Vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f);

    NavigationCommand out = CfCPilot::safety_filter(cmd, grid, pose);

    // No obstacle in the braking cone -> command passes through unchanged.
    CHECK(out.type == NavigationCommand::VELOCITY_SETPOINT);
    CHECK(std::fabs(out.velocity.x - 2.0f) < 1e-5f);
    CHECK(std::fabs(out.velocity.y - 0.0f) < 1e-5f);
    CHECK(std::fabs(out.velocity.z - 0.0f) < 1e-5f);
}

// A near-zero command must never be touched by the filter (no divide by zero).
static void test_zero_command_passes_through() {
    GridConfig cfg(8, 0.5f, 4.0f);
    DualVoxelGrid grid(cfg);
    // Even with an obstacle adjacent, a zero command should be returned as-is.
    int ix, iy, iz;
    bool ok = grid.world_to_voxel(Vec3(0.25f, 0.0f, 0.0f), ix, iy, iz);
    CHECK(ok);
    grid.accumulate_static(grid.voxel_to_index(ix, iy, iz), 1.0f);

    NavigationCommand cmd;
    cmd.type = NavigationCommand::VELOCITY_SETPOINT;
    cmd.velocity = Vec3(0.0f, 0.0f, 0.0f);
    Pose pose(Vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f);

    NavigationCommand out = CfCPilot::safety_filter(cmd, grid, pose);
    CHECK(out.type == NavigationCommand::VELOCITY_SETPOINT);
    CHECK(std::fabs(out.velocity.x) < 1e-6f);
}

// Obstacle NOT on the commanded path: filter leaves the command intact.
static void test_off_path_obstacle_does_not_veto() {
    GridConfig cfg(8, 0.5f, 4.0f);
    DualVoxelGrid grid(cfg);

    // Obstacle at +y; command is +x. The +x march never visits the +y voxel.
    int ix, iy, iz;
    bool ok = grid.world_to_voxel(Vec3(0.0f, 1.0f, 0.0f), ix, iy, iz);
    CHECK(ok);
    grid.accumulate_static(grid.voxel_to_index(ix, iy, iz), 1.0f);

    NavigationCommand cmd;
    cmd.type = NavigationCommand::VELOCITY_SETPOINT;
    cmd.velocity = Vec3(2.0f, 0.0f, 0.0f);
    Pose pose(Vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f);

    NavigationCommand out = CfCPilot::safety_filter(cmd, grid, pose);
    CHECK(out.type == NavigationCommand::VELOCITY_SETPOINT);
    CHECK(std::fabs(out.velocity.x - 2.0f) < 1e-5f);
}

int main() {
    test_command_toward_obstacle_is_vetoed();
    test_command_toward_open_space_passes();
    test_zero_command_passes_through();
    test_off_path_obstacle_does_not_veto();

    if (failures == 0) {
        std::printf("[cfc_safety_test] all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "[cfc_safety_test] %d check(s) failed\n", failures);
    return 1;
}