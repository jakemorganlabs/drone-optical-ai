# Drone Optical AI — Live Voxel Mapping System

A real-time, camera-based voxel mapping system with motion detection,
DBSCAN-style clustering, multi-frame tracking, and autonomous
navigation planning. Originally designed for drone applications requiring
real-time 3D environment understanding, and built to run on Linux (incl.
Raspberry Pi), macOS, and WSL.

> Status: **works out of the box on a fresh checkout** — `make` produces a
> working `embedded_voxel_mapper` binary with no external dependencies beyond
> a C++17 compiler and (optionally) OpenMP.

## Features

- **Live Camera Integration**: Real-time frame processing from camera streams
  via a `PoseProvider` interface (mock included; real VIO/FCU providers are
  pluggable).
- **Dual Voxel Grids**: Dynamic (motion-based) and static (dense mapping)
  voxel representations.
- **Motion Detection**: Adaptive-threshold motion detection with noise
  filtering.
- **3D Clustering**: DBSCAN-style 3D voxel clustering.
- **Object Tracking**: Multi-frame track association with velocity estimation.
- **Navigation Planning**: Costmap generation, A* path planning, and velocity
  command generation with obstacle avoidance.
- **Flight Control Interface**: Real-time setpoint streaming to flight
  controllers (PX4/ArduPilot/custom).
- **Sliding Window**: Dynamic grid recentering for large-scale mapping.

## File Structure

```
.
├── math.hpp              # Math utilities, rotations, vectors
├── grid.hpp              # Dual voxel grid management (mutex-protected)
├── pose.hpp              # Pose structures, CameraFrame, PoseProvider interface
├── motion.hpp            # Motion detection (fixed/adaptive/filtered)
├── ray_marching.hpp       # Ray casting & DDA traversal of voxel grid
├── clustering.hpp        # 3D DBSCAN clustering & multi-frame tracks
├── navigation.hpp        # Costmap, path planning, FCU interface
├── live_voxel_mapper.cpp # Main live mapping system + CLI entry point
├── process_image.cpp     # pybind11 module for offline processing (Python bindings)
├── ray_voxel.cpp         # Legacy offline demo (optional third_party deps)
├── setup.py              # Python bindings build
├── CMakeLists.txt        # CMake build
├── Makefile              # Make build (Pi-friendly)
├── spacevoxelviewer.py   # 3D visualization
├── voxelmotionviewer.py  # Motion visualization
├── tests/
│   └── smoke_test.cpp    # Cross-module compilation & behavior smoke tests
├── third_party/
│   └── README.md         # How to vendor optional legacy deps
└── docs/
    └── RASPBERRY_PI.md   # Raspberry Pi build & deployment guide
```

## Quick Start

### Build

```bash
# Easiest: Make
make
./build/embedded_voxel_mapper --help

# Or via CMake
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build
./build/bin/embedded_voxel_mapper --help
```

### Run

```bash
# Default: 30s run, 160^3 grid, 0.5m voxels, 40m horizon
./build/embedded_voxel_mapper

# Configure via CLI flags
./build/embedded_voxel_mapper \
    --grid 96 \
    --voxel 0.25 \
    --horizon 20.0 \
    --duration 10 \
    --out my_grid.bin
```

### Test

```bash
# 3-second smoke test of the binary
make test

# Compile + run cross-module smoke tests
cmake -B build . && cmake --build build --target smoke_test    # if a target is configured
# or directly:
clang++ -std=c++17 -O2 -I. -o build/smoke_test tests/smoke_test.cpp
./build/smoke_test
```

## Building on Raspberry Pi

See [docs/RASPBERRY_PI.md](docs/RASPBERRY_PI.md) for the full guide. TL;DR:

```bash
sudo apt update
sudo apt install -y build-essential cmake libomp-dev
git clone https://github.com/jakemorganlabs/drone-optical-ai.git
cd drone-optical-ai
make
./build/embedded_voxel_mapper --grid 64 --voxel 0.5 --duration 10
```

The project is pure C++17 with no required runtime libraries, so it
also runs on Pi Zero / Zero W (single-threaded, with a smaller grid).

## Python Bindings

The Python bindings expose the offline `process_image_cpp` module via
pybind11.

```bash
# Desktop (development/visualization)
pip install -e .

# Embedded target (live camera)
BUILD_EMBEDDED=1 pip install -e .
```

Requires `pybind11` (`pip install pybind11` or `apt install pybind11-dev`).

### Visualization

```bash
python spacevoxelviewer.py
python voxelmotionviewer.py
```

## Configuration

### Grid Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `N` | 160 | Grid size (N³ voxels) |
| `voxel_size` | 0.5m | Voxel resolution |
| `horizon_distance` | 40m | Maximum ray marching distance |

### Motion Detection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `motion_threshold` | 2.0 | Fixed difference threshold |
| `adaptive_percentile` | 95.0 | Percentile for adaptive threshold |
| `min_region_size` | 4 | Minimum pixels for valid motion |
| `noise_filtering` | true | Enable noise filtering |

### Decay Rates

| Grid Type | Half-life | Decay Rate |
|-----------|-----------|------------|
| Dynamic | 2s | 0.988 per frame |
| Static | 60s | 0.997 per frame |

### Navigation

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_velocity` | 5.0 m/s | Maximum drone velocity |
| `max_acceleration` | 2.0 m/s² | Maximum acceleration |
| `planning_horizon` | 10s | Path planning time horizon |
| `replan_rate` | 5Hz | Path replanning frequency |

## Flight Controller Integration

The `FlightControllerInterface` class is hardware-agnostic. Provide a
callback to bridge it to your FCU:

```cpp
FlightControllerInterface fcu(20.0f);  // 20 Hz

fcu.set_command_callback([](const NavigationCommand& cmd) {
    switch(cmd.type) {
        case NavigationCommand::VELOCITY_SETPOINT:
            send_velocity_setpoint(cmd.velocity, cmd.yaw);  // your MAVLink/PX4 call
            break;
        case NavigationCommand::EMERGENCY_STOP:
            send_emergency_stop();
            break;
    }
});

fcu.send_command(velocity_cmd);
```

Supported command types: `POSITION_SETPOINT`, `VELOCITY_SETPOINT`, `LAND`,
`HOLD`, `EMERGENCY_STOP`.

## Performance

| Operation | Performance | Notes |
|-----------|-------------|-------|
| Motion Detection | ~1ms @ 640x480 | Adaptive threshold |
| Ray Casting | ~5ms @ 1000 rays | DDA algorithm |
| Voxel Update | ~2ms @ 160³ grid | OpenMP parallel (when available) |
| Clustering | ~10ms @ 1000 voxels | DBSCAN algorithm |
| Costmap Gen | ~20ms @ 160x160 | 2D projection |

Memory: ~32 MB for voxel grids at 160³ (proportional to N³).

## Contributing

1. Fork the repository.
2. Create a feature branch: `git checkout -b feature/amazing-feature`.
3. Make sure `./build/smoke_test` passes.
4. Commit changes: `git commit -m 'Add amazing feature'`.
5. Push to branch: `git push origin feature/amazing-feature`.
6. Open a pull request.

Code style:
- C++: Google C++ Style Guide
- Python: PEP 8
- Headers: `#pragma once`
- Documentation: Doxygen-style comments

## License

MIT License — see [LICENSE](LICENSE). The optional `third_party/`
dependencies (nlohmann/json, stb) retain their own licenses.

## Acknowledgments

- OpenMP — parallel processing framework
- STB Image — image loading utilities (optional, for legacy demo)
- nlohmann/json — JSON parsing library (optional, for legacy demo)
- pybind11 — Python-C++ bindings (optional, for Python module)

---

**Note**: This system is designed for research and development. Always
test thoroughly in simulation before deploying to real hardware.