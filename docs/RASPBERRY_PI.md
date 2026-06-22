# Raspberry Pi Build & Deployment Guide

This guide walks through building and running Drone Optical AI on
Raspberry Pi (Pi 4, Pi 5, Pi 3, and Pi Zero / Zero 2 W).

## Supported Hardware

| Board | Status | Notes |
|-------|--------|-------|
| Raspberry Pi 5 (aarch64) | Full | OpenMP supported, ~32 MB voxel grids OK |
| Raspberry Pi 4 (aarch64) | Full | Recommended minimum for 160³ grids |
| Raspberry Pi 3 (armv7hf) | Full | Reduce grid to N=96 for steady 30 FPS |
| Raspberry Pi Zero 2 W | OK | Use N=48–64, single-threaded |
| Raspberry Pi Zero (v1) | Slow | Use N=32, single-threaded, expect <5 FPS |

The project is pure C++17 with no required runtime libraries, so it runs
on any Pi that has a C++17 compiler (`g++ 7+` or `clang++ 5+`).

## 1. Install toolchain

```bash
sudo apt update
sudo apt install -y build-essential cmake git
# OpenMP is optional but recommended on Pi 3/4/5:
sudo apt install -y libomp-dev
```

Verify:

```bash
g++ --version    # expect 7.x or newer
cmake --version  # expect 3.16+ (skip if using make only)
```

## 2. Clone & build

```bash
git clone https://github.com/jakemorganlabs/drone-optical-ai.git
cd drone-optical-ai

# Make (recommended on Pi)
make

# Or CMake
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build
```

The build outputs `build/embedded_voxel_mapper` (Make) or
`build/bin/embedded_voxel_mapper` (CMake).

## 3. Run

```bash
# Quick 10-second smoke run with a small grid
./build/embedded_voxel_mapper --grid 64 --voxel 0.5 --duration 10

# Heavier mapping run (Pi 4+ recommended)
./build/embedded_voxel_mapper \
    --grid 128 \
    --voxel 0.25 \
    --horizon 25.0 \
    --duration 60 \
    --out /home/pi/voxel_map.bin
```

## 4. Tuning for performance

| Board | `--grid` | `--voxel` | `--horizon` | Notes |
|-------|---------|----------|-----------|-------|
| Pi 5  | 160 | 0.5 | 40 | Full quality |
| Pi 4  | 128 | 0.5 | 30 | Smooth |
| Pi 3  | 96  | 0.5 | 25 | OK |
| Zero 2 W | 64 | 0.5 | 20 | Single-threaded |
| Zero (v1) | 32 | 0.5 | 15 | <5 FPS |

Memory at N³ voxels with two float grids is roughly:
`N³ × 8 bytes`. For N=160 this is ~32 MB; for N=64 it is ~2 MB.

## 5. Attaching a live camera

To use a live camera instead of the mock `PoseProvider`, implement a
new `PoseProvider` subclass that:

- Streams `Pose` updates from your VIO/IMU source.
- Streams `CameraFrame` data (grayscale float pixels) from your camera.
- Calls the registered callbacks registered via
  `register_pose_callback()` / `register_frame_callback()`.

Minimal pattern:

```cpp
class PiCameraPoseProvider : public PoseProvider {
public:
    bool start() override {
        active_ = true;
        capture_thread_ = std::thread(&PiCameraPoseProvider::loop, this);
        return true;
    }
    void stop() override {
        active_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
    }
    // ...register/get methods...
private:
    void loop() {
        while (active_) {
            CameraFrame frame = /* read from /dev/video0 etc. */;
            if (frame_callback_) frame_callback_(frame);
            Pose pose = /* read from IMU / VIO */;
            if (pose_callback_) pose_callback_(pose);
        }
    }
};
```

For the Raspberry Pi Camera Module, the `libcamera` or `picamera2`
stack is recommended. For IMU/VIO, connect over UART/MAVLink to a
companion flight controller (PX4 / ArduPilot).

## 6. Run as a systemd service (optional)

`/etc/systemd/system/voxel-mapper.service`:

```ini
[Unit]
Description=Drone Optical AI Live Voxel Mapper
After=network.target

[Service]
ExecStart=/home/pi/drone-optical-ai/build/embedded_voxel_mapper \
    --grid 96 --duration 3600 --out /home/pi/maps/voxel_map.bin
WorkingDirectory=/home/pi/drone-optical-ai
Restart=on-failure
User=pi

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now voxel-mapper.service
journalctl -u voxel-mapper -f
```

## 7. Cross-compiling (optional)

If you develop on x86 Linux and deploy to Pi, you can cross-compile
with an `aarch64-linux-gnu` toolchain:

```bash
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
cd drone-optical-ai
make CXX=aarch64-linux-gnu-g++ CXXFLAGS="-std=c++17 -O3 -fopenmp"
```

Copy `build/embedded_voxel_mapper` to the Pi and run.