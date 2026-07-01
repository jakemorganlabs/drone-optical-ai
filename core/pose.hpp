#pragma once

#include "math.hpp"
#include <chrono>
#include <functional>
#include <vector>

//----------------------------------------------
// Pose Structures
//----------------------------------------------
struct Pose {
    Vec3 position;           // Camera/drone position in world coordinates
    float yaw;               // Yaw angle in degrees
    float pitch;             // Pitch angle in degrees  
    float roll;              // Roll angle in degrees
    std::chrono::steady_clock::time_point timestamp;  // Timestamp
    
    Pose() : position(0,0,0), yaw(0), pitch(0), roll(0) {}
    Pose(const Vec3& pos, float y, float p, float r) 
        : position(pos), yaw(y), pitch(p), roll(r) {}
    
    // Get rotation matrix
    Mat3 get_rotation_matrix() const {
        return rotation_matrix_yaw_pitch_roll(yaw, pitch, roll);
    }
    
    // Get forward direction (negative Z axis in camera frame)
    Vec3 get_forward_direction() const {
        Mat3 rot = get_rotation_matrix();
        Vec3 forward = {0, 0, -1};  // Camera looks down negative Z
        return mat3_mul_vec3(rot, forward);
    }
    
    // Get up direction (positive Y axis in camera frame)
    Vec3 get_up_direction() const {
        Mat3 rot = get_rotation_matrix();
        Vec3 up = {0, 1, 0};  // Camera up is positive Y
        return mat3_mul_vec3(rot, up);
    }
    
    // Get right direction (positive X axis in camera frame)
    Vec3 get_right_direction() const {
        Mat3 rot = get_rotation_matrix();
        Vec3 right = {1, 0, 0};  // Camera right is positive X
        return mat3_mul_vec3(rot, right);
    }
};

//----------------------------------------------
// Camera Frame Information
//----------------------------------------------
struct CameraFrame {
    int width;
    int height;
    std::vector<float> pixels;  // Grayscale float values
    Pose camera_pose;
    float fov_degrees;
    std::chrono::steady_clock::time_point timestamp;
    
    CameraFrame() : width(0), height(0), fov_degrees(60.0f) {}
    CameraFrame(int w, int h, float fov) : width(w), height(h), fov_degrees(fov) {
        pixels.resize(static_cast<size_t>(w) * static_cast<size_t>(h), 0.0f);
    }

    // Get pixel index (row-major)
    int get_index(int u, int v) const {
        return v * width + u;
    }
    
    // Get focal length
    float get_focal_length() const {
        return compute_focal_length(width, fov_degrees);
    }
    
    // Get pixel direction in camera frame
    Vec3 get_pixel_direction(int u, int v) const {
        float focal_len = get_focal_length();
        float x = (float(u) - 0.5f * width);
        float y = -(float(v) - 0.5f * height);
        float z = -focal_len;
        
        Vec3 ray_cam = {x, y, z};
        return normalize(ray_cam);
    }
    
    // Get pixel direction in world frame
    Vec3 get_pixel_direction_world(int u, int v) const {
        Vec3 ray_cam = get_pixel_direction(u, v);
        Mat3 rot = camera_pose.get_rotation_matrix();
        Vec3 ray_world = mat3_mul_vec3(rot, ray_cam);
        return normalize(ray_world);
    }
};

//----------------------------------------------
// Pose Callback Interface
//----------------------------------------------
using PoseCallback = std::function<void(const Pose&)>;
using FrameCallback = std::function<void(const CameraFrame&)>;

//----------------------------------------------
// VIO/FCU Interface (Abstract)
//----------------------------------------------
class PoseProvider {
public:
    virtual ~PoseProvider() = default;
    
    // Register callbacks
    virtual void register_pose_callback(PoseCallback callback) = 0;
    virtual void register_frame_callback(FrameCallback callback) = 0;
    
    // Start/stop pose streaming
    virtual bool start() = 0;
    virtual void stop() = 0;
    
    // Check if provider is active
    virtual bool is_active() const = 0;
    
    // Get latest pose (non-blocking)
    virtual bool get_latest_pose(Pose& pose) const = 0;
    
    // Get latest frame (non-blocking)
    virtual bool get_latest_frame(CameraFrame& frame) const = 0;
};

//----------------------------------------------
// Mock Pose Provider (for testing/development)
//----------------------------------------------
// Dev provider that synthesizes a moving camera and a few bright "pillars"
// at fixed world spots so the ray-caster has real structure to accumulate.
// Future hardware providers (DronePoseProvider) implement the same interface
// but source frames from libcamera/depth SDKs and pose from the FC bridge.
class MockPoseProvider : public PoseProvider {
private:
    PoseCallback pose_callback;
    FrameCallback frame_callback;
    bool active;
    Pose current_pose;
    CameraFrame current_frame;
    float angle_ = 0.0f;   // instance state (no function-local static)

public:
    MockPoseProvider() : active(false) {
        current_pose = Pose(Vec3(0, 0, 10), 0, 0, 0);
        current_frame = CameraFrame(640, 480, 60.0f);
        current_frame.camera_pose = current_pose;
    }

    void register_pose_callback(PoseCallback callback) override {
        pose_callback = callback;
    }

    void register_frame_callback(FrameCallback callback) override {
        frame_callback = callback;
    }

    bool start() override {
        active = true;
        return true;
    }

    void stop() override {
        active = false;
    }

    bool is_active() const override {
        return active;
    }

    bool get_latest_pose(Pose& pose) const override {
        if(!active) return false;
        pose = current_pose;
        return true;
    }

    bool get_latest_frame(CameraFrame& frame) const override {
        if(!active) return false;
        frame = current_frame;
        return true;
    }

    // Externally driven pose update (still supported for tests)
    void update_pose(const Pose& new_pose) {
        current_pose = new_pose;
        current_frame.camera_pose = new_pose;
        if(pose_callback && active) {
            pose_callback(new_pose);
        }
    }

    // Externally driven frame update (still supported for tests)
    void update_frame(const CameraFrame& new_frame) {
        current_frame = new_frame;
        if(frame_callback && active) {
            frame_callback(new_frame);
        }
    }

    // Advance the synthetic scene by dt seconds. The camera orbits at z=10
    // looking straight down world -Z (identity rotation, since this codebase's
    // yaw rotates around Z and doesn't steer the horizontal gaze). Pillars sit
    // on the ground (z=0) within the FOV, so the ray-caster accumulates real
    // occupancy and the motion detector sees frame-to-frame change as the
    // camera translates.
    void simulate_movement(float dt) {
        if(!active) return;
        angle_ += 0.3f * dt;

        // Small orbit in XY at z=10; identity rotation means forward = world -Z.
        current_pose.position = Vec3(2.0f * std::cos(angle_),
                                     2.0f * std::sin(angle_), 10.0f);
        current_pose.yaw = 0.0f;
        current_pose.pitch = 0.0f;
        current_pose.roll = 0.0f;
        current_pose.timestamp = std::chrono::steady_clock::now();
        current_frame.camera_pose = current_pose;

        // Splat a few bright pillars at fixed world spots on the ground. FOV is
        // 60deg, so at depth 10m the visible XY radius is 10*tan(30) ~= 5.8m;
        // all pillars below sit inside that disc and move in the image as the
        // camera orbits.
        std::fill(current_frame.pixels.begin(), current_frame.pixels.end(), 0.0f);
        const Vec3 pillars[] = {
            { 2.0f,  0.0f, 0.0f}, {-2.0f,  0.0f, 0.0f},
            { 0.0f,  2.0f, 0.0f}, { 0.0f, -2.0f, 0.0f},
            { 2.0f,  2.0f, 0.0f}, {-2.0f, -2.0f, 0.0f},
            { 3.0f, -1.0f, 0.0f}, {-1.0f,  3.0f, 0.0f}
        };
        Mat3 R = transpose(current_pose.get_rotation_matrix());
        const float f = current_frame.get_focal_length();
        for(const Vec3& p : pillars) {
            Vec3 rel = mat3_mul_vec3(R, p - current_pose.position);
            if(rel.z >= -0.5f) continue;                   // behind / at camera plane
            int u = int(0.5f * current_frame.width  + f * (rel.x / -rel.z));
            int v = int(0.5f * current_frame.height - f * (rel.y / -rel.z));
            for(int dv = -6; dv <= 6; ++dv)
              for(int du = -6; du <= 6; ++du) {
                int uu = u+du, vv = v+dv;
                if(uu>=0 && uu<current_frame.width && vv>=0 && vv<current_frame.height)
                  current_frame.pixels[current_frame.get_index(uu,vv)] = 1.0f;
              }
        }
        current_frame.timestamp = current_pose.timestamp;

        if(pose_callback)  pose_callback(current_pose);
        if(frame_callback) frame_callback(current_frame);   // <-- drives the pipeline
    }
};
