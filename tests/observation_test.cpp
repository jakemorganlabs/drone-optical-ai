// Phase 3 unit tests for control/cfc_pilot/observation.hpp.
//
// Constructs a tiny DualVoxelGrid, seeds an occupied voxel straight ahead of a
// Pose, and asserts the corresponding sector reads near zero (obstacle) while
// an open sector reads near 1.0. Matches the manual's normalized-distance
// semantics (0 = obstacle at drone, 1 = clear to horizon).

#include "control/cfc_pilot/observation.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

static int failures = 0;
#define CHECK(cond) do { \
    if(!(cond)) { std::cerr << "CHECK FAILED: " << #cond \
        << " at " << __FILE__ << ":" << __LINE__ << "\n"; ++failures; } \
} while(0)

static bool approx(float a, float b, float eps) {
    return std::fabs(a - b) < eps;
}

// GridConfig(24, 0.5f, 4.0f): origin (0,0,0), half_size = 24*0.5/2 = 6.0,
// so the grid spans [-6, 6] in each axis and the horizon is 4.0 m. A drone at
// the origin can march the full 4 m horizon along any axis without leaving
// the grid, so an unobstructed sector reads ~1.0 (clear to horizon).
static void test_obstacle_ahead_obstructs_sector_zero() {
    GridConfig cfg(24, 0.5f, 4.0f);
    DualVoxelGrid grid(cfg);

    // Seed an occupied voxel straight ahead (+x) of the drone at the origin.
    // World (1.0, 0.0, 0.0) lands well inside the grid. The sector for k=0
    // sweeps ang=0 -> world_ang = yaw=0 + 0 = 0 -> dir = (+1, 0, 0), so k=0
    // must be the obstructed sector.
    int ix, iy, iz;
    Pose pose(Vec3(0.0f, 0.0f, 0.0f), 0.0f, 0.0f, 0.0f);
    bool ok = grid.world_to_voxel(Vec3(1.0f, 0.0f, 0.0f), ix, iy, iz);
    CHECK(ok);
    int idx = grid.voxel_to_index(ix, iy, iz);
    grid.accumulate_static(idx, 1.0f);

    Vec3 vel(0.0f, 0.0f, 0.0f);
    Vec3 goal(0.0f, 0.0f, 0.0f);
    Observation o = build_observation(grid, pose, vel, goal);

    // k=0 (forward +x) should be near zero (obstacle hit at ~1 step ahead).
    CHECK(o.data[0] < 0.4f);

    // k=12 (backward -x) is unobstructed and stays inside the grid for the
    // full 4 m horizon, so it must read ~1.0 (clear to horizon).
    CHECK(approx(o.data[12], 1.0f, 1e-3f));

    // k=6 (90deg to the left, +y) is also unobstructed and clear to horizon.
    CHECK(approx(o.data[6], 1.0f, 1e-3f));

    // Drone at the origin, goal at the origin -> goal vector is zero. The
    // builder returns a zero unit vector and distance 0. The remaining
    // non-sector fields (velocity, height) should be zero too.
    CHECK(approx(o.data[K_SECTORS + 0], 0.0f, 1e-5f));   // vel.x
    CHECK(approx(o.data[K_SECTORS + 1], 0.0f, 1e-5f));   // vel.y
    CHECK(approx(o.data[K_SECTORS + 2], 0.0f, 1e-5f));   // vel.z
    CHECK(approx(o.data[K_SECTORS + 3], 0.0f, 1e-5f));   // height z/maxd
    CHECK(approx(o.data[K_SECTORS + 4], 0.0f, 1e-5f));   // gu.x
    CHECK(approx(o.data[K_SECTORS + 5], 0.0f, 1e-5f));   // gu.y
    CHECK(approx(o.data[K_SECTORS + 6], 0.0f, 1e-5f));   // gd/maxd
}

// Drone yawed 90deg: k=0's world angle becomes yaw+0 = 90deg, so k=0 now
// sweeps +y. Verify the obstruction at +y shows up at k=0 under yaw=90.
static void test_yaw_rotates_sector_sweep() {
    GridConfig cfg(24, 0.5f, 4.0f);
    DualVoxelGrid grid(cfg);

    int ix, iy, iz;
    Pose pose(Vec3(0.0f, 0.0f, 0.0f), /*yaw=*/90.0f, 0.0f, 0.0f);
    bool ok = grid.world_to_voxel(Vec3(0.0f, 1.0f, 0.0f), ix, iy, iz);
    CHECK(ok);
    grid.accumulate_static(grid.voxel_to_index(ix, iy, iz), 1.0f);

    Vec3 vel(0.0f, 0.0f, 0.0f);
    Vec3 goal(0.0f, 0.0f, 0.0f);
    Observation o = build_observation(grid, pose, vel, goal);

    // With yaw=90deg, k=0 sweeps world direction (cos90, sin90, 0) = (+y),
    // so the +y obstacle obstructs k=0.
    CHECK(o.data[0] < 0.4f);

    // The opposite body sector (k=12, body ang=pi -> world ang=pi/2+pi=3pi/2
    // -> world dir (-y)) is unobstructed and clear to horizon.
    CHECK(approx(o.data[12], 1.0f, 1e-3f));
}

int main() {
    test_obstacle_ahead_obstructs_sector_zero();
    test_yaw_rotates_sector_sweep();

    if (failures == 0) {
        std::printf("[observation_test] all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "[observation_test] %d check(s) failed\n", failures);
    return 1;
}