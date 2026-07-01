# perception/

Real-time perception backends for the onboard stack. Both implement the
`PoseProvider` contract from `pose.hpp` so `LiveVoxelMapper` is used unchanged.

## Backend (a) — mono libcamera (`drone_pose_provider.{hpp,cpp}`)

- Pipes `rpicam-vid ... --codec yuv420 --nopreview -o -` through `popen` and
  takes the Y (luma) plane as a grayscale frame.
- Dependency-light: no libcamera / librealsense / depthai headers. Compiles on
  a dev machine without a camera; `rpicam-vid` is only invoked at runtime on the
  Pi. If the binary is missing it logs an error and returns rather than spinning.
- Use for development / structure-from-motion experiments. **Fragile for
  real-time avoidance** (a single camera cannot recover metric depth).

## Backend (b) — depth camera (`depth_occupancy.hpp`)

- For OAK-D / RealSense / other depth sensors that emit a per-pixel metric
  depth buffer (`float* depth_m`, meters).
- `DepthOccupancyWriter::write_depth_frame()` back-projects each depth pixel to
  a world point via `CameraFrame::get_pixel_direction()` + `Pose::get_rotation_matrix()`
  and accumulates it into the `DualVoxelGrid` static occupancy via
  `accumulate_static`. Replaces monocular ray-casting for avoidance.
- Generic `float* depth_m` interface; the integrator wires the depth SDK's
  frame callback to `write_depth_frame()` (and to
  `DronePoseProvider::on_camera_frame` so frame timing/stats still flow).

## Which to choose

Use **(b) depth** for a real avoider — this is the manual's recommendation
(Part 0, reality check #1). Use **(a) mono** only for dev bring-up or
structure-from-motion mapping where you accept fragility. Set the camera type
in `/etc/dronectl/config.yaml` (`camera.type: depth_oakd` vs `mono_libcamera`).