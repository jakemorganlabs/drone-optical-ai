// Phase 1 compile-only unit test. Includes the new perception TUs and calls a
// couple of methods so the new translation units (drone_pose_provider.cpp /
// depth_occupancy.hpp) are exercised. The two-TU ODR gate is implicitly served
// by linking the perception TU alongside the main the mapper TU in the
// pilot_main / embedded_voxel_mapper_full targets; this test makes the
// percepton headers themselves a compile gate too.
#include "perception/drone_pose_provider.hpp"
#include "perception/depth_occupancy.hpp"

#include "grid.hpp"
#include "pose.hpp"
#include "math.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <utility>

static int failures = 0;
#define CHECK(cond) do { \
    if(!(cond)) { std::cerr << "CHECK FAILED: " << #cond \
        << " at " << __FILE__ << ":" << __LINE__ << "\n"; ++failures; } \
} while(0)

static void test_drone_pose_provider() {
    DronePoseProvider provider(4, 4, 60.0f);
    CHECK(!provider.is_active());

    bool got_pose = false;
    bool got_frame = false;
    provider.register_pose_callback([&](const Pose&) { got_pose = true; });
    provider.register_frame_callback([&](const CameraFrame&) { got_frame = true; });

    // push_pose before start() must NOT fire the callback (active_ is false),
    // but must store the pose for get_latest_pose().
    Pose pushed(Vec3(1, 2, 3), 10, 0, 0);
    provider.push_pose(pushed);
    CHECK(!got_pose);

    Pose out;
    CHECK(provider.get_latest_pose(out));
    CHECK(std::fabs(out.position.x - 1.0f) < 1e-4f);
    CHECK(std::fabs(out.position.y - 2.0f) < 1e-4f);
    CHECK(std::fabs(out.position.z - 3.0f) < 1e-4f);

    CameraFrame fout;
    CHECK(!provider.get_latest_frame(fout));   // no frame pushed yet

    // start() spawns the capture thread; on a dev box rpicam-vid is absent so
    // capture_loop logs and returns immediately. Either way the provider is
    // now active and a subsequent push_pose must fire the callback.
    provider.start();
    CHECK(provider.is_active());
    provider.push_pose(pushed);
    CHECK(got_pose);
    provider.stop();
    CHECK(!provider.is_active());
}

static void test_depth_occupancy() {
    GridConfig cfg(8, 0.5f, 10.0f);
    DualVoxelGrid grid(cfg);

    CameraFrame frame(4, 4, 60.0f);
    Pose pose(Vec3(0, 0, 0), 0, 0, 0);   // identity rotation, at origin

    // A 4x4 depth image: everything 0.5m in front of the camera (z=0.5 in
    // world since identity rotation). Cheapness: just make sure some voxels
    // accumulate > 0 without crashing.
    std::vector<float> depth(4 * 4, 0.5f);
    DepthOccupancyWriter::write_depth_frame(grid, frame, depth.data(),
                                            4, 4, pose, 5.0f);

    // Confirm at least one static voxel got bumped by the accumulation pass.
    bool any_static = false;
    for (int i = 0; i < grid.get_total_voxels(); ++i) {
        if (grid.get_static_voxel(i) > 0.0f) { any_static = true; break; }
    }
    CHECK(any_static);

    // Callback-pattern API: set a callback returning the same depth+pose, then
    // write_latest() must also accumulate.
    DepthOccupancyWriter writer;
    bool cb_called = false;
    writer.set_depth_callback([&]() {
        cb_called = true;
        return std::make_pair(depth.data(), pose);
    });
    int before = 0;
    for (int i = 0; i < grid.get_total_voxels(); ++i)
        before += (grid.get_static_voxel(i) > 0.0f) ? 1 : 0;
    writer.write_latest(grid, frame, 5.0f);
    CHECK(cb_called);
    int after = 0;
    for (int i = 0; i < grid.get_total_voxels(); ++i)
        after += (grid.get_static_voxel(i) > 0.0f) ? 1 : 0;
    CHECK(after >= before);
}

int main() {
    test_drone_pose_provider();
    test_depth_occupancy();
    if (failures == 0) {
        std::cout << "perception_compile_test passed.\n";
        return 0;
    }
    std::cerr << failures << " checks failed.\n";
    return 1;
}