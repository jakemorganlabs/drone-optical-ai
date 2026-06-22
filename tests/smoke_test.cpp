// Minimal smoke test that exercises every public header so CI can catch
// compilation regressions across math/grid/pose/motion/ray_marching/
// clustering/navigation headers.
#include "math.hpp"
#include "grid.hpp"
#include "pose.hpp"
#include "motion.hpp"
#include "ray_marching.hpp"
#include "clustering.hpp"
#include "navigation.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

static int failures = 0;
#define CHECK(cond) do { \
    if(!(cond)) { std::cerr << "CHECK FAILED: " << #cond \
        << " at " << __FILE__ << ":" << __LINE__ << "\n"; ++failures; } \
} while(0)

void test_math() {
    Vec3 a(1, 2, 3), b(4, 5, 6);
    Vec3 c = a + b;
    CHECK(c.x == 5 && c.y == 7 && c.z == 9);
    CHECK(std::fabs(length(a) - std::sqrt(14.0f)) < 1e-4f);
    Vec3 n = normalize(a);
    CHECK(std::fabs(length(n) - 1.0f) < 1e-4f);
    Mat3 r = rotation_matrix_yaw_pitch_roll(0, 0, 0);
    CHECK(r.m[0] == 1.0f && r.m[4] == 1.0f && r.m[8] == 1.0f);
    CHECK(std::fabs(compute_focal_length(100, 90.0f) - 50.0f) < 1e-3f);
}

void test_grid() {
    GridConfig cfg(8, 0.5f, 10.0f);
    DualVoxelGrid grid(cfg);
    CHECK(grid.get_total_voxels() == 8 * 8 * 8);
    grid.accumulate_dynamic(0, 3.0f);
    CHECK(std::fabs(grid.get_dynamic_voxel(0) - 3.0f) < 1e-4f);
    grid.apply_decay();
    CHECK(grid.get_dynamic_voxel(0) < 3.0f);   // Decay reduces the value
    int idx = grid.voxel_to_index(1, 2, 3);
    CHECK(idx == 1 * 64 + 2 * 8 + 3);

    // Tile accumulation
    auto tile = grid.create_thread_tile();
    grid.accumulate_dynamic_tile(0, 1.5f, *tile);
    grid.commit_tile(*tile);
    CHECK(grid.get_dynamic_voxel(0) > 0.0f);
}

void test_motion() {
    ImageGray a(4, 4), b(4, 4);
    for(int i = 0; i < 16; ++i) {
        a.pixels[i] = (i % 2) ? 1.0f : 0.0f;
        b.pixels[i] = (i % 2) ? 5.0f : 0.0f;
    }
    MotionMask mm = detect_motion(a, b, 1.0f);
    CHECK(mm.count_changed() == 8);
    MotionStats stats = compute_motion_stats(mm);
    CHECK(stats.max_diff == 4.0f);
}

void test_clustering_tracking() {
    GridConfig cfg(8, 0.5f, 10.0f);
    DualVoxelGrid grid(cfg);
    grid.accumulate_dynamic(grid.voxel_to_index(2, 2, 2), 5.0f);
    grid.accumulate_dynamic(grid.voxel_to_index(2, 2, 3), 5.0f);

    VoxelClusterer clusterer(2.0f, 1);
    auto clusters = clusterer.find_clusters(grid, 0.1f);
    CHECK(!clusters.empty());

    VoxelTracker tracker(10.0f);
    tracker.update_tracks(clusters);
    CHECK(tracker.get_tracks().size() >= 1);
}

void test_navigation() {
    GridConfig cfg(8, 0.5f, 10.0f);
    DualVoxelGrid grid(cfg);
    VoxelClusterer clusterer(2.0f, 1);
    auto clusters = clusterer.find_clusters(grid, 0.1f);

    VoxelCostmapGenerator costgen(0.3f, 0.7f, 1.0f, 2.0f);
    Vec3 drone_pos(0, 0, 10);
    auto costmap = costgen.generate_costmap(grid, clusters, drone_pos);
    CHECK(costmap.get_width() > 0);
    CHECK(costmap.get_height() > 0);

    NavigationPlanner planner(5.0f, 2.0f, 10.0f, 5.0f);
    Vec3 start(0, 0, 10);
    Vec3 goal(5, 5, 10);
    auto waypoints = planner.plan_path(costmap, start, goal);
    CHECK(!waypoints.empty());

    auto cmd = planner.generate_velocity_command(start, goal, Vec3(0, 0, 0), costmap);
    CHECK(cmd.type == NavigationCommand::VELOCITY_SETPOINT);
}

int main() {
    test_math();
    test_grid();
    test_motion();
    test_clustering_tracking();
    test_navigation();
    if(failures == 0) {
        std::cout << "All smoke tests passed.\n";
        return 0;
    }
    std::cerr << failures << " checks failed.\n";
    return 1;
}