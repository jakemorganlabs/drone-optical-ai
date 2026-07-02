#pragma once

// Phase 3 (manual Part 4.1): compact observation builder for the CfC pilot.
//
// We do NOT feed the CfC the raw 160^3 voxel grid. We feed a compact, physical
// state vector: K horizontal sector clearances + vertical clearance + drone
// velocity + height proxy + goal vector. This is the state representation the
// liquid net reasons over at each 20 Hz control tick.
//
// Manual signature used `VoxelGrid`; the repo's grid class is `DualVoxelGrid`
// (core/grid.hpp), so the signature here takes `const DualVoxelGrid&` and uses
// its `get_static_voxel` / `get_dynamic_voxel` / `world_to_voxel` /
// `voxel_to_index` / `get_config` accessors. `deg2rad` and `length` come from
// core/math.hpp (pulled in transitively by grid.hpp).

#include "grid.hpp"   // DualVoxelGrid, GridConfig
#include "pose.hpp"   // Pose
#include "math.hpp"   // deg2rad, length, Vec3

#include <array>
#include <cmath>

// K horizontal sectors spaced evenly around the yaw plane.
static constexpr int K_SECTORS = 24;
// OBS_DIM = K_SECTORS (sector clearances)
//         + 3 (velocity xyz)
//         + 1 (height proxy)
//         + 3 (goal unit xy + normalized distance)
static constexpr int OBS_DIM = K_SECTORS + 3 + 1 + 3;

struct Observation {
    std::array<float, OBS_DIM> data{};
};

// March each sector direction through the grid; record normalized distance to
// the first occupied voxel (1.0 = clear to the grid horizon). Sectors are a
// body-frame yaw sweep rotated into world frame by pose.yaw so the net sees a
// drone-centric polar occupancy scan regardless of heading.
inline Observation build_observation(const DualVoxelGrid& grid,
                                     const Pose& pose,
                                     const Vec3& velocity,
                                     const Vec3& goal_world) {
    Observation o{};
    o.data.fill(1.0f);
    const GridConfig& cfg = grid.get_config();
    const float step = cfg.voxel_size * 0.5f;
    const float maxd = cfg.horizon_distance;

    for (int k = 0; k < K_SECTORS; ++k) {
        float ang = (2.0f * static_cast<float>(M_PI) * static_cast<float>(k)) /
                    static_cast<float>(K_SECTORS);        // body-frame yaw sweep
        float world_ang = deg2rad(pose.yaw) + ang;
        Vec3 dir(std::cos(world_ang), std::sin(world_ang), 0.0f);
        float hit = maxd;
        for (float d = step; d < maxd; d += step) {
            Vec3 wp = pose.position + dir * d;
            int ix, iy, iz;
            if (!grid.world_to_voxel(wp, ix, iy, iz)) { hit = d; break; }
            int idx = grid.voxel_to_index(ix, iy, iz);
            if (grid.get_static_voxel(idx) + grid.get_dynamic_voxel(idx) > 0.5f) {
                hit = d;
                break;
            }
        }
        o.data[k] = hit / maxd;   // 0 = obstacle at drone, 1 = clear to horizon
    }

    int i = K_SECTORS;
    // Velocity (body/world frame, normalized by a 5 m/s max).
    o.data[i++] = velocity.x / 5.0f;
    o.data[i++] = velocity.y / 5.0f;
    o.data[i++] = velocity.z / 5.0f;
    // Height proxy (relative to grid horizon; AGL would be better but needs
    // a terrain reference — left for Phase 5).
    o.data[i++] = pose.position.z / maxd;
    // Goal: unit direction xy + normalized distance.
    Vec3 g = goal_world - pose.position;
    float gd = length(g);
    Vec3 gu = gd > 1e-3f ? g * (1.0f / gd) : Vec3(0.0f, 0.0f, 0.0f);
    o.data[i++] = gu.x;
    o.data[i++] = gu.y;
    o.data[i++] = std::min(gd / maxd, 1.0f);
    return o;
}