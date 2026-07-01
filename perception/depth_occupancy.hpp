#pragma once

// Phase 1 perception backend (b): depth-camera occupancy writer.
//
// A depth camera (OAK-D / RealSense / DepthAI) provides a per-pixel metric
// depth buffer. Rather than cast rays from monocular intensity (fragile under
// ego-motion), we back-project each depth pixel to a world point and
// accumulate it into the static occupancy grid via DualVoxelGrid::accumulate_static.
// This is the recommended path for real obstacle avoidance (manual Part 0,
// reality check #1).
//
// This header is intentionally generic: it takes a `float* depth_m` buffer and
// a CameraFrame + Pose, and knows nothing about any specific depth SDK. The
// integrator wires the depth SDK's frame callback to write_depth_frame() and
// to DronePoseProvider::on_camera_frame (so the mapper's frame timing/stats
// still flow). No libcamera / librealsense / depthai headers are included.

#include "grid.hpp"
#include "pose.hpp"

#include <functional>
#include <utility>

class DepthOccupancyWriter {
public:
    // Callback giving the writer access to a depth buffer + the pose it was
    // captured with. Registered by the depth SDK integrator.
    using DepthSampleCallback =
        std::function<std::pair<const float*, Pose>()>;

    DepthOccupancyWriter() = default;
    explicit DepthOccupancyWriter(DepthSampleCallback cb) : cb_(std::move(cb)) {}

    void set_depth_callback(DepthSampleCallback cb) { cb_ = std::move(cb); }

    // Back-project every depth pixel to a world point and accumulate into the
    // grid's static occupancy. Matches the manual Part 2.2(b) pseudocode:
    //   for each pixel (u,v): z = depth_m(v*W+u); reject z<=0 or z>horizon;
    //   ray_cam = frame.get_pixel_direction(u,v);          // unit dir in cam frame
    //   p_cam  = ray_cam * (z / -ray_cam.z);                // metric point, cam frame
    //   p_world= pose.position + R * p_cam;                 // R = pose.get_rotation_matrix()
    //   if grid.world_to_voxel(p_world, ix,iy,iz):
    //       grid.accumulate_static(grid.voxel_to_index(ix,iy,iz), 1.0f)
    //
    // NOTE: the manual writes the signature with `const DualVoxelGrid&`, but
    // DualVoxelGrid::accumulate_static is a non-const method (it takes a
    // mutex and mutates the static occupancy buffer). The grid parameter is
    // therefore non-const here so the body compiles; this is a deliberate,
    // minimal deviation from the pseudocode.
    static void write_depth_frame(DualVoxelGrid& grid,
                                  const CameraFrame& frame,
                                  const float* depth_m,
                                  int W, int H,
                                  const Pose& pose,
                                  float horizon) {
        if (!depth_m || W <= 0 || H <= 0) return;

        const Mat3 R = pose.get_rotation_matrix();

        for (int v = 0; v < H; ++v) {
            for (int u = 0; u < W; ++u) {
                float z = depth_m[v * W + u];
                if (z <= 0.0f || z > horizon) continue;

                Vec3 ray_cam = frame.get_pixel_direction(u, v);   // unit, cam frame
                if (ray_cam.z >= -1e-6f) continue;                // at/behind camera

                Vec3 p_cam = ray_cam * (z / -ray_cam.z);           // metric point, cam frame
                Vec3 p_world = pose.position + mat3_mul_vec3(R, p_cam);

                int ix, iy, iz;
                if (!grid.world_to_voxel(p_world, ix, iy, iz)) continue;
                grid.accumulate_static(grid.voxel_to_index(ix, iy, iz), 1.0f);
            }
        }
    }

    // Convenience: pull the latest depth buffer + pose via the registered
    // callback and write into `grid`. The integrator calls this on each depth
    // callback tick.
    void write_latest(DualVoxelGrid& grid, const CameraFrame& frame, float horizon) {
        if (!cb_) return;
        auto sample = cb_();
        const float* depth = sample.first;
        if (!depth) return;
        write_depth_frame(grid, frame, depth, frame.width, frame.height,
                          sample.second, horizon);
    }

private:
    DepthSampleCallback cb_;
};