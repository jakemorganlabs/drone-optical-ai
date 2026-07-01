// Phase 1 perception backend (a): mono libcamera path via rpicam-vid YUV420.
// Dependency-light: pipes `rpicam-vid ... --codec yuv420 -o -` through popen and
// takes the Y (luma) plane as the grayscale image. No libcamera C++ SDK headers
// are included, so this TU compiles on dev machines without a camera; the
// rpicam-vid binary is only invoked at runtime on the Pi.

#include "drone_pose_provider.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <chrono>
#include <string>
#include <vector>

namespace {

// Check whether a binary is resolvable on PATH without spinning if it's gone.
bool have_binary(const char* name) {
    std::string which = "command -v ";
    which += name;
    which += " >/dev/null 2>&1";
    return std::system(which.c_str()) == 0;
}

} // namespace

void DronePoseProvider::capture_loop() {
    const int W = frame_.width;
    const int H = frame_.height;
    if (W <= 0 || H <= 0) {
        std::fprintf(stderr, "[DronePoseProvider] invalid frame dims %dx%d\n", W, H);
        return;
    }

    // Refuse to spin if rpicam-vid isn't installed (e.g. dev machine). Log once
    // and return, leaving the provider active but frame-less so the FC bridge
    // pose path still works.
    if (!have_binary("rpicam-vid")) {
        std::fprintf(stderr,
            "[DronePoseProvider] rpicam-vid not found on PATH; "
            "mono camera capture disabled (depth path is independent).\n");
        return;
    }

    std::string cmd = "rpicam-vid -t 0 --width " + std::to_string(W) +
                      " --height " + std::to_string(H) +
                      " --framerate 20 --codec yuv420 --nopreview -o -";

    // popen/pclose are POSIX (in <stdio.h>), not in namespace std::. Use the
    // global namespace so the TU builds on macOS/non-glibc toolchains too.
    FILE* p = ::popen(cmd.c_str(), "r");
    if (!p) {
        std::fprintf(stderr, "[DronePoseProvider] popen(rpicam-vid) failed\n");
        return;
    }

    const size_t frame_bytes = static_cast<size_t>(W) * static_cast<size_t>(H) * 3 / 2; // YUV420
    std::vector<uint8_t> raw(frame_bytes);

    while (active_ && std::fread(raw.data(), 1, frame_bytes, p) == frame_bytes) {
        CameraFrame f = frame_;                 // copies width/height/fov + pixel buffer
        for (int i = 0; i < W * H; ++i) {
            f.pixels[i] = raw[i] / 255.0f;      // Y plane = grayscale intensity
        }
        f.timestamp = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lk(m_);
            if (have_pose_) f.camera_pose = pose_;   // stamp with latest FC pose
        }
        on_camera_frame(f);
    }

    ::pclose(p);
}