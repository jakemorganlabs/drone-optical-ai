#pragma once

#include "math.hpp"
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <mutex>

//----------------------------------------------
// Grid Management
//----------------------------------------------
struct GridConfig {
    int N;                    // Grid size (N^3)
    float voxel_size;         // Voxel size in meters
    float horizon_distance;   // Maximum ray marching distance
    Vec3 origin;              // Current grid origin in world coordinates

    GridConfig() : N(160), voxel_size(0.5f), horizon_distance(40.0f), origin(0,0,0) {}
    GridConfig(int N_, float voxel_size_, float horizon_) : N(N_), voxel_size(voxel_size_), horizon_distance(horizon_), origin(0,0,0) {}
};

//----------------------------------------------
// Dual Voxel Grids
//----------------------------------------------
// Voxel storage is plain `float` rather than `std::atomic<float>` so the
// grid can be grown/copied/moved freely. Concurrent writes from OpenMP
// parallel sections are guarded by `grid_mutex` on the commit/decay paths,
// and the per-thread accumulation tiles avoid contention during ray casting.
class DualVoxelGrid {
private:
    GridConfig config;
    std::vector<float> dynamic_vox;
    std::vector<float> static_vox;
    mutable std::mutex grid_mutex;

    // Decay parameters
    float dynamic_decay_rate;  // per-frame decay factor
    float static_decay_rate;   // per-frame decay factor

    // Thread-local accumulation tiles to reduce atomic pressure
    struct ThreadTile {
        std::vector<float> dynamic_accum;
        std::vector<float> static_accum;
        ThreadTile(int size) : dynamic_accum(size, 0.0f), static_accum(size, 0.0f) {}
    };

public:
    DualVoxelGrid(const GridConfig& cfg) : config(cfg) {
        int total_voxels = config.N * config.N * config.N;
        dynamic_vox.assign(total_voxels, 0.0f);
        static_vox.assign(total_voxels, 0.0f);

        // Set decay rates (half-life based)
        // For 30 FPS: half-life of 2s = decay_rate = 0.5^(1/(30*2)) ~= 0.988
        dynamic_decay_rate = 0.988f;  // ~2 second half-life
        static_decay_rate = 0.997f;   // ~60 second half-life
    }

    // Get grid configuration
    const GridConfig& get_config() const { return config; }

    // Get voxel value (thread-safe read under lock)
    float get_dynamic_voxel(int idx) const {
        std::lock_guard<std::mutex> lk(grid_mutex);
        return dynamic_vox[idx];
    }
    float get_static_voxel(int idx) const {
        std::lock_guard<std::mutex> lk(grid_mutex);
        return static_vox[idx];
    }

    // Accumulate into voxels (thread-safe)
    void accumulate_dynamic(int idx, float value) {
        std::lock_guard<std::mutex> lk(grid_mutex);
        dynamic_vox[idx] += value;
    }

    void accumulate_static(int idx, float value) {
        std::lock_guard<std::mutex> lk(grid_mutex);
        static_vox[idx] += value;
    }

    // Thread-local accumulation for better performance
    void accumulate_dynamic_tile(int idx, float value, ThreadTile& tile) {
        tile.dynamic_accum[idx] += value;
    }

    void accumulate_static_tile(int idx, float value, ThreadTile& tile) {
        tile.static_accum[idx] += value;
    }

    // Commit thread-local tiles to main grid
    void commit_tile(const ThreadTile& tile) {
        std::lock_guard<std::mutex> lk(grid_mutex);
        int total_voxels = config.N * config.N * config.N;
        for(int i = 0; i < total_voxels; i++) {
            if(tile.dynamic_accum[i] != 0.0f) {
                dynamic_vox[i] += tile.dynamic_accum[i];
            }
            if(tile.static_accum[i] != 0.0f) {
                static_vox[i] += tile.static_accum[i];
            }
        }
    }

    // Apply decay to all voxels
    void apply_decay() {
        std::lock_guard<std::mutex> lk(grid_mutex);
        int total_voxels = config.N * config.N * config.N;
        for(int i = 0; i < total_voxels; i++) {
            if(dynamic_vox[i] > 0.0f) {
                dynamic_vox[i] *= dynamic_decay_rate;
            }
            if(static_vox[i] > 0.0f) {
                static_vox[i] *= static_decay_rate;
            }
        }
    }

    // Recenter grid origin (sliding window)
    void recenter_origin(const Vec3& new_origin) {
        std::lock_guard<std::mutex> lk(grid_mutex);
        // For now, just update the origin. In a full implementation, you'd
        // translate the voxel data and zero newly revealed regions.
        config.origin = new_origin;
    }

    // Check if drone is near grid edge
    bool needs_recentering(const Vec3& drone_pos) const {
        float half_size = 0.5f * config.N * config.voxel_size;
        Vec3 grid_min = config.origin - Vec3(half_size, half_size, half_size);
        Vec3 grid_max = config.origin + Vec3(half_size, half_size, half_size);

        // Recenter if drone is within 20% of grid edge
        float margin = 0.2f * half_size;
        return (drone_pos.x < grid_min.x + margin || drone_pos.x > grid_max.x - margin ||
                drone_pos.y < grid_min.y + margin || drone_pos.y > grid_max.y - margin ||
                drone_pos.z < grid_min.z + margin || drone_pos.z > grid_max.z - margin);
    }

    // Get AABB bounds for ray intersection
    void get_bounds(Vec3& min, Vec3& max) const {
        float half_size = 0.5f * config.N * config.voxel_size;
        min = config.origin - Vec3(half_size, half_size, half_size);
        max = config.origin + Vec3(half_size, half_size, half_size);
    }

    // Convert world position to voxel indices
    bool world_to_voxel(const Vec3& world_pos, int& ix, int& iy, int& iz) const {
        Vec3 grid_min, grid_max;
        get_bounds(grid_min, grid_max);

        if(world_pos.x < grid_min.x || world_pos.x > grid_max.x ||
           world_pos.y < grid_min.y || world_pos.y > grid_max.y ||
           world_pos.z < grid_min.z || world_pos.z > grid_max.z) {
            return false;
        }

        ix = static_cast<int>((world_pos.x - grid_min.x) / config.voxel_size);
        iy = static_cast<int>((world_pos.y - grid_min.y) / config.voxel_size);
        iz = static_cast<int>((world_pos.z - grid_min.z) / config.voxel_size);

        // Clamp to valid range
        ix = std::max(0, std::min(ix, config.N - 1));
        iy = std::max(0, std::min(iy, config.N - 1));
        iz = std::max(0, std::min(iz, config.N - 1));

        return true;
    }

    // Convert voxel indices to linear index
    int voxel_to_index(int ix, int iy, int iz) const {
        return ix * config.N * config.N + iy * config.N + iz;
    }

    // Get total voxel count
    int get_total_voxels() const { return config.N * config.N * config.N; }

    // Create thread tile
    std::unique_ptr<ThreadTile> create_thread_tile() const {
        return std::make_unique<ThreadTile>(get_total_voxels());
    }
};