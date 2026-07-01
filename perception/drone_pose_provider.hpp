#pragma once

// Phase 1: real perception provider. Frames come from a camera thread
// (libcamera / rpicam-vid YUV420 path, or a depth-SDK callback wired in by
// the integrator); pose is pushed in by the flight-controller bridge
// (Part 3) via push_pose(). Implements the existing PoseProvider contract so
// LiveVoxelMapper is used unchanged.

#include "pose.hpp"
#include <atomic>
#include <thread>
#include <mutex>

class DronePoseProvider : public PoseProvider {
public:
    DronePoseProvider(int w, int h, float fov) : frame_(w, h, fov) {}

    void register_pose_callback(PoseCallback cb) override  { pose_cb_ = std::move(cb); }
    void register_frame_callback(FrameCallback cb) override { frame_cb_ = std::move(cb); }

    bool start() override {
        active_ = true;
        cam_thread_ = std::thread(&DronePoseProvider::capture_loop, this);
        return true;
    }
    void stop() override {
        active_ = false;
        if (cam_thread_.joinable()) cam_thread_.join();
    }
    bool is_active() const override { return active_; }

    bool get_latest_pose(Pose& p) const override {
        std::lock_guard<std::mutex> lk(m_);
        if (!have_pose_) return false;
        p = pose_;
        return true;
    }
    bool get_latest_frame(CameraFrame& f) const override {
        std::lock_guard<std::mutex> lk(m_);
        if (!have_frame_) return false;
        f = frame_;
        return true;
    }

    // Called by the FC bridge whenever a fresh EKF/VIO pose arrives.
    void push_pose(const Pose& p) {
        {
            std::lock_guard<std::mutex> lk(m_);
            pose_ = p;
            have_pose_ = true;
        }
        if (pose_cb_ && active_) pose_cb_(p);
    }

private:
    void capture_loop();               // implemented in .cpp (Part 2.2)
    void on_camera_frame(const CameraFrame& f) {
        {
            std::lock_guard<std::mutex> lk(m_);
            frame_ = f;
            have_frame_ = true;
        }
        if (frame_cb_ && active_) frame_cb_(f);
    }

    mutable std::mutex m_;
    PoseCallback pose_cb_;
    FrameCallback frame_cb_;
    std::atomic<bool> active_{false};
    std::thread cam_thread_;
    Pose pose_;
    CameraFrame frame_;
    bool have_pose_ = false, have_frame_ = false;
};