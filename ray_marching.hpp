#pragma once

#include "math.hpp"
#include "grid.hpp"
#include <vector>
#include <limits>

//----------------------------------------------
// Ray Marching Structures
//----------------------------------------------
struct RayStep {
    int ix, iy, iz;
    int step_count;
    float distance;
    float t_entry, t_exit;  // Entry/exit times for AABB intersection
    
    RayStep() : ix(0), iy(0), iz(0), step_count(0), distance(0.0f), t_entry(0.0f), t_exit(0.0f) {}
    RayStep(int x, int y, int z, int step, float dist, float t_in, float t_out) 
        : ix(x), iy(y), iz(z), step_count(step), distance(dist), t_entry(t_in), t_exit(t_out) {}
};

//----------------------------------------------
// Ray-AABB Intersection
//----------------------------------------------
bool ray_aabb_intersection(
    const Vec3& ray_origin,
    const Vec3& ray_direction,
    const Vec3& box_min,
    const Vec3& box_max,
    float& t_entry,
    float& t_exit)
{
    float tmin = -std::numeric_limits<float>::infinity();
    float tmax = std::numeric_limits<float>::infinity();

    for (int i = 0; i < 3; ++i)
    {
        float origin = (i==0)? ray_origin.x : ((i==1)? ray_origin.y : ray_origin.z);
        float d = (i==0)? ray_direction.x : ((i==1)? ray_direction.y : ray_direction.z);
        float mn = (i==0)? box_min.x : ((i==1)? box_min.y : box_min.z);
        float mx = (i==0)? box_max.x : ((i==1)? box_max.y : box_max.z);

        if (std::abs(d) > 1e-8f)
        {
            float t1 = (mn - origin) / d;
            float t2 = (mx - origin) / d;

            if (t1 > t2) std::swap(t1, t2);

            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);

            if (tmin > tmax)
                return false;
        }
        else
        {
            // Ray is parallel to the slab. If the origin is not within the slab, no intersection
            if (origin < mn || origin > mx)
                return false;
        }
    }

    t_entry = tmin;
    t_exit = tmax;
    return true;
}

//----------------------------------------------
// Distance-Aware Weighting
//----------------------------------------------
float compute_distance_weight(float distance, float focal_length, float voxel_size, float min_weight = 0.1f) {
    // Solid angle model: weight ∝ 1/distance²
    // But we need to account for voxel size and focal length
    float solid_angle_weight = 1.0f / (1.0f + distance * distance);
    
    // Normalize by focal length and voxel size
    float normalized_weight = solid_angle_weight * (focal_length * focal_length) / (voxel_size * voxel_size);
    
    // Apply lower bound to prevent overweighting
    return std::max(normalized_weight, min_weight);
}

//----------------------------------------------
// Ray Casting into Grid
//----------------------------------------------
std::vector<RayStep> cast_ray_into_grid(
    const Vec3& camera_pos, 
    const Vec3& dir_normalized, 
    const DualVoxelGrid& grid,
    float max_distance = -1.0f)
{
    std::vector<RayStep> steps;
    steps.reserve(64);

    const GridConfig& config = grid.get_config();
    
    // Use grid's horizon distance if max_distance not specified
    if(max_distance < 0.0f) {
        max_distance = config.horizon_distance;
    }

    Vec3 grid_min, grid_max;
    grid.get_bounds(grid_min, grid_max);

    float t_min = 0.f;
    float t_max = max_distance;

    // 1) Ray-box intersection
    if(!ray_aabb_intersection(camera_pos, dir_normalized, grid_min, grid_max, t_min, t_max)) {
        return steps; // no intersection
    }

    if(t_min < 0.f) t_min = 0.f;
    if(t_max > max_distance) t_max = max_distance;
    
    if(t_min >= t_max) {
        return steps; // no valid intersection range
    }

    // 2) Start voxel
    Vec3 start_world = camera_pos + dir_normalized * t_min;
    int ix, iy, iz;
    if(!grid.world_to_voxel(start_world, ix, iy, iz)) {
        return steps;
    }

    // 3) Step direction
    int step_x = (dir_normalized.x >= 0.f)? 1 : -1;
    int step_y = (dir_normalized.y >= 0.f)? 1 : -1;
    int step_z = (dir_normalized.z >= 0.f)? 1 : -1;

    auto boundary_in_world_x = [&](int i_x){ return grid_min.x + i_x * config.voxel_size; };
    auto boundary_in_world_y = [&](int i_y){ return grid_min.y + i_y * config.voxel_size; };
    auto boundary_in_world_z = [&](int i_z){ return grid_min.z + i_z * config.voxel_size; };

    int nx_x = ix + (step_x>0?1:0);
    int nx_y = iy + (step_y>0?1:0);
    int nx_z = iz + (step_z>0?1:0);

    float next_bx = boundary_in_world_x(nx_x);
    float next_by = boundary_in_world_y(nx_y);
    float next_bz = boundary_in_world_z(nx_z);

    float t_max_x = safe_div(next_bx - camera_pos.x, dir_normalized.x);
    float t_max_y = safe_div(next_by - camera_pos.y, dir_normalized.y);
    float t_max_z = safe_div(next_bz - camera_pos.z, dir_normalized.z);

    float t_delta_x = safe_div(config.voxel_size, std::fabs(dir_normalized.x));
    float t_delta_y = safe_div(config.voxel_size, std::fabs(dir_normalized.y));
    float t_delta_z = safe_div(config.voxel_size, std::fabs(dir_normalized.z));

    float t_current = t_min;
    int step_count = 0;

    // 4) Walk through voxels
    while(t_current <= t_max && step_count < config.N * 2) { // Safety limit
        RayStep rs(ix, iy, iz, step_count, t_current, t_min, t_max);
        steps.push_back(rs);

        if(t_max_x < t_max_y && t_max_x < t_max_z){
            ix += step_x;
            t_current = t_max_x;
            t_max_x += t_delta_x;
        } else if(t_max_y < t_max_z){
            iy += step_y;
            t_current = t_max_y;
            t_max_y += t_delta_y;
        } else {
            iz += step_z;
            t_current = t_max_z;
            t_max_z += t_delta_z;
        }
        step_count++;
        
        // Check bounds
        if(ix<0 || ix>=config.N || iy<0 || iy>=config.N || iz<0 || iz>=config.N){
            break;
        }
    }

    return steps;
}

//----------------------------------------------
// Optimized Ray Casting with Early Exit
//----------------------------------------------
std::vector<RayStep> cast_ray_into_grid_optimized(
    const Vec3& camera_pos, 
    const Vec3& dir_normalized, 
    const DualVoxelGrid& grid,
    float max_distance = -1.0f,
    int max_steps = -1)
{
    std::vector<RayStep> steps;
    steps.reserve(32); // Smaller reserve for optimization

    const GridConfig& config = grid.get_config();
    
    if(max_distance < 0.0f) {
        max_distance = config.horizon_distance;
    }
    
    if(max_steps < 0) {
        max_steps = config.N; // Default to grid size
    }

    Vec3 grid_min, grid_max;
    grid.get_bounds(grid_min, grid_max);

    float t_min, t_max;
    if(!ray_aabb_intersection(camera_pos, dir_normalized, grid_min, grid_max, t_min, t_max)) {
        return steps;
    }

    t_min = std::max(t_min, 0.0f);
    t_max = std::min(t_max, max_distance);
    
    if(t_min >= t_max) {
        return steps;
    }

    // Start voxel
    Vec3 start_world = camera_pos + dir_normalized * t_min;
    int ix, iy, iz;
    if(!grid.world_to_voxel(start_world, ix, iy, iz)) {
        return steps;
    }

    // Step direction
    int step_x = (dir_normalized.x >= 0.f)? 1 : -1;
    int step_y = (dir_normalized.y >= 0.f)? 1 : -1;
    int step_z = (dir_normalized.z >= 0.f)? 1 : -1;

    // Precompute step parameters
    float t_delta_x = safe_div(config.voxel_size, std::fabs(dir_normalized.x));
    float t_delta_y = safe_div(config.voxel_size, std::fabs(dir_normalized.y));
    float t_delta_z = safe_div(config.voxel_size, std::fabs(dir_normalized.z));

    auto boundary_in_world_x = [&](int i_x){ return grid_min.x + i_x * config.voxel_size; };
    auto boundary_in_world_y = [&](int i_y){ return grid_min.y + i_y * config.voxel_size; };
    auto boundary_in_world_z = [&](int i_z){ return grid_min.z + i_z * config.voxel_size; };

    int nx_x = ix + (step_x>0?1:0);
    int nx_y = iy + (step_y>0?1:0);
    int nx_z = iz + (step_z>0?1:0);

    float t_max_x = safe_div(boundary_in_world_x(nx_x) - camera_pos.x, dir_normalized.x);
    float t_max_y = safe_div(boundary_in_world_y(nx_y) - camera_pos.y, dir_normalized.y);
    float t_max_z = safe_div(boundary_in_world_z(nx_z) - camera_pos.z, dir_normalized.z);

    float t_current = t_min;
    int step_count = 0;

    // Walk through voxels with early exit
    while(t_current <= t_max && step_count < max_steps) {
        steps.emplace_back(ix, iy, iz, step_count, t_current, t_min, t_max);

        // Find next boundary
        if(t_max_x < t_max_y && t_max_x < t_max_z){
            ix += step_x;
            t_current = t_max_x;
            t_max_x += t_delta_x;
        } else if(t_max_y < t_max_z){
            iy += step_y;
            t_current = t_max_y;
            t_max_y += t_delta_y;
        } else {
            iz += step_z;
            t_current = t_max_z;
            t_max_z += t_delta_z;
        }
        step_count++;
        
        // Check bounds
        if(ix<0 || ix>=config.N || iy<0 || iy>=config.N || iz<0 || iz>=config.N){
            break;
        }
    }

    return steps;
}

//----------------------------------------------
// Batch Ray Casting for Multiple Pixels
//----------------------------------------------
struct BatchRayResult {
    std::vector<RayStep> steps;
    int pixel_u, pixel_v;
    float pixel_value;
    
    BatchRayResult() : pixel_u(0), pixel_v(0), pixel_value(0.0f) {}
    BatchRayResult(int u, int v, float val) : pixel_u(u), pixel_v(v), pixel_value(val) {}
};

std::vector<BatchRayResult> cast_rays_batch(
    const std::vector<std::pair<int, int>>& pixel_coords,
    const std::vector<float>& pixel_values,
    const Vec3& camera_pos,
    const Mat3& camera_rot,
    float focal_length,
    const DualVoxelGrid& grid,
    float max_distance = -1.0f)
{
    std::vector<BatchRayResult> results;
    results.reserve(pixel_coords.size());

    for(size_t i = 0; i < pixel_coords.size(); i++) {
        int u = pixel_coords[i].first;
        int v = pixel_coords[i].second;
        float val = pixel_values[i];

        // Compute ray direction for this pixel
        float x = (float(u) - 0.5f * grid.get_config().N);
        float y = -(float(v) - 0.5f * grid.get_config().N);
        float z = -focal_length;

        Vec3 ray_cam = {x, y, z};
        ray_cam = normalize(ray_cam);

        // Transform to world coordinates
        Vec3 ray_world = mat3_mul_vec3(camera_rot, ray_cam);
        ray_world = normalize(ray_world);

        // Cast ray
        std::vector<RayStep> steps = cast_ray_into_grid_optimized(
            camera_pos, ray_world, grid, max_distance);

        if(!steps.empty()) {
            BatchRayResult result(u, v, val);
            result.steps = std::move(steps);
            results.push_back(std::move(result));
        }
    }

    return results;
}
