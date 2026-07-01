// Second translation unit for the ODR regression test. Includes every header
// exactly the way the main binary does, then references one symbol from each
// so the compiler cannot discard anything. Must link cleanly against
// odr_two_tu_main.cpp once the header functions are marked inline.
#include "math.hpp"
#include "grid.hpp"
#include "pose.hpp"
#include "motion.hpp"
#include "ray_marching.hpp"
#include "clustering.hpp"
#include "navigation.hpp"

#include <vector>

// Pull one symbol from each header-defined free function so they are emitted.
int other_tu_force_emit() {
    Mat3 r = rotation_matrix_yaw_pitch_roll(1.0f, 2.0f, 3.0f);
    float f = compute_focal_length(100, 90.0f);
    ImageGray a(2, 2), b(2, 2);
    MotionMask mm = detect_motion(a, b, 0.1f);
    MotionStats st = compute_motion_stats(mm);
    float t = compute_adaptive_threshold(a, b, 1.0f, 95.0f);
    MotionMask mf = detect_motion_filtered(a, b, 0.1f, 2);
    MotionMask ma = detect_motion_adaptive(a, b, 1.0f, 95.0f);

    GridConfig cfg(4, 0.5f, 5.0f);
    DualVoxelGrid grid(cfg);
    Vec3 origin(0, 0, 0), dir(1, 0, 0);
    auto steps = cast_ray_into_grid(origin, dir, grid, 2.0f);
    auto steps2 = cast_ray_into_grid_optimized(origin, dir, grid, 2.0f, 16);
    float w = compute_distance_weight(1.0f, 50.0f, 0.5f);
    std::vector<std::pair<int,int>> px = {{0,0}};
    std::vector<float> pv = {1.0f};
    auto batch = cast_rays_batch(px, pv, origin, r, 50.0f, grid, 2.0f);
    bool ok = batch.empty() || true;
    (void)steps; (void)steps2; (void)w; (void)st; (void)t;
    (void)mf; (void)ma; (void)mm; (void)ok;
    return static_cast<int>(r.m[0] + f);
}