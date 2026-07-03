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
├── core/                  # Header-only voxel core (math/grid/pose/motion/ray/cluster/nav)
│   ├── math.hpp           # Math utilities, rotations, vectors
│   ├── grid.hpp           # Dual voxel grid management (mutex-protected)
│   ├── pose.hpp           # Pose structures, CameraFrame, PoseProvider interface
│   ├── motion.hpp         # Motion detection (fixed/adaptive/filtered)
│   ├── ray_marching.hpp   # Ray casting & DDA traversal of voxel grid
│   ├── clustering.hpp     # 3D DBSCAN clustering & multi-frame tracks
│   ├── navigation.hpp     # Costmap, path planning, FCU interface
│   ├── live_voxel_mapper.cpp # Main live mapping system + CLI entry point
│   └── ray_voxel.cpp      # Legacy offline demo (optional third_party deps)
├── CMakeLists.txt         # CMake build (defines the voxelcore INTERFACE library)
├── Makefile               # Make build (Pi-friendly)
├── voxelmotionviewer.py   # Motion visualization (reads the VXG1 .bin output)
├── tests/
│   ├── smoke_test.cpp       # Cross-module compilation & behavior smoke tests
│   ├── odr_two_tu_main.cpp # Two-TU ODR regression (links 2 TUs w/ all headers)
│   └── odr_two_tu_other.cpp
├── third_party/
│   └── README.md         # How to vendor optional legacy deps
├── docs/                 # Option A build manual pages
│   ├── RASPBERRY_PI.md     # Raspberry Pi build & deployment guide
│   ├── WIRING.md           # Flight controller <-> companion computer wiring
│   ├── HARDWARE.md         # Bill of materials (per-drone vs shared ground)
│   ├── TETHER.md           # Fiber-optic tether datalink
│   ├── FLEET.md            # One Starlink terminal, many drones
│   ├── SAFETY.md           # Test ladder + pre-flight card template
│   ├── REGULATORY.md       # US Part 107 / Remote ID / BVLOS / LAANC
│   └── ARCHITECTURE.md     # One-page stack diagram (onboard + ground)
└── tests/                 # Smoke + ODR + failsafe + perception + (planned) cfc
```

The `core/` directory is a header-only interface library (`voxelcore` in
CMake, `-Icore` in the Makefile): bare includes like `#include "math.hpp"`
resolve from any TU in the repo via that include path, so new code under
`perception/`, `control/`, etc. does not need path rewrites.

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
# Full smoke suite: mapper runs and reports >0 frames processed,
# then the behavior unit tests, then the two-TU ODR regression test.
make test

# Or via CMake
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build
ctest --test-dir build --output-on-failure
```

## Build manual pages

This repo is the partial implementation of an onboard
CfC-piloted drone with a fiber tether, a shared ground box, and a
remote LLM mission brain. The end-to-end design lives across these docs:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — one-page stack diagram
  (onboard + ground).
- [docs/HARDWARE.md](docs/HARDWARE.md) — the bill of materials
  (per-drone expendable vs shared ground).
- [docs/WIRING.md](docs/WIRING.md) — flight controller <-> companion
  computer wiring + MAVLink buzzword checklist.
- [docs/TETHER.md](docs/TETHER.md) — fiber-optic tether datalink +
  link-loss contract.
- [docs/FLEET.md](docs/FLEET.md) — one Starlink terminal, many drones.
- [docs/SAFETY.md](docs/SAFETY.md) — the test ladder + a pre-flight
  card template (print and sign before props spin).
- [docs/REGULATORY.md](docs/REGULATORY.md) — US Part 107 / Remote ID /
  tethered-UAS / BVLOS / LAANC / RF (verify for your region).

The CMake option gates are on by default OFF (manual Part 9.2):

```bash
# Bare configure, no external deps — must succeed:
cmake -B build .

# Wrapped builds (optional; require MAVSDK / onnxruntime):
cmake -B build -DDRONECTL_ENABLE_FC_BRIDGE=ON .
cmake -B build -DDRONECTL_ENABLE_CFC=ON .
```

The Make build is the gate; the CMake script is the reference / long-term build.
Single-TU `make` produces the embedded mapper by default; `make test` runs
the full unit ladder (see [tests/README.md](tests/README.md)).

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

### Visualization

```bash
python voxelmotionviewer.py        # reads live_voxel_grid.bin by default
# (Edit the path inside the script if you used --out my_grid.bin.)
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

## Packaging

The end-to-end, flashable turn-key artifact lives under [`image/`](image/):

- **`image/systemd/`** — the `dronectl-pilot`, `mavlink-router`,
  `tether-agent`, and first-boot `dronectl-wizard` systemd units (manual
  Part 8.1).
- **`image/config/config.yaml`** — the default `/etc/dronectl/config.yaml`
  schema (manual Part 8.2); the first-boot wizard rewrites it on first run.
- **`image/wizard/wizard.py`** — a stdlib-only first-boot AP web wizard
  (manual Part 8.4) that collects `drone_id`, camera type, FC URL, tether
  IPs, and an optional server endpoint, then writes the config, disables
  itself, and reboots.
- **`image/dronectl`** — the stdlib-only `dronectl status` /
  `dronectl preflight` CLI (manual Part 8.5); installed at
  `/usr/local/bin/dronectl` by the image stage. Both subcommands degrade to
  `MISSING` for any absent subsystem, so they are safe to run on a dev box
  with none of the onboard artifacts present.
- **`image/pi-gen/`** — a custom [pi-gen](https://github.com/RPi-Distro/pi-gen)
  stage (`stage-dronectl/`) that apt-installs `libcamera`, `onnxruntime`,
  `libmavsdk-dev`, and `mavlink-router`, drops the prebuilt `pilot_main` +
  `embedded_voxel_mapper` + `cfc_policy.onnx` into `/opt/dronectl/`, enables
  the `dronectl-*` units, and ships the default config. See
  [`image/pi-gen/README.md`](image/pi-gen/README.md) and the
  [`build.sh`](image/pi-gen/build.sh) skeleton.

### Building the image

```bash
# 1. Cross-compile (or build on a Pi) so that build/pilot_main,
#    build/embedded_voxel_mapper, build/tether_agent, and cfc_policy.onnx exist.
# 2. Stage + bake:
image/pi-gen/build.sh --repo-root . --artifacts build --release
# 3. (the --release flag xz -9s deploy/*.img for you)
```

### GitHub Release flow

Attach `deploy/dronectl-pi.img.xz` to a GitHub Release tagged e.g. `v0.X`;
users flash it with Raspberry Pi Imager, boot, join the `dronectl-<host>` AP,
run the wizard at `http://10.0.0.1/`, and fly. The same image works with
`sim/quickstart.sh` for zero-hardware SITL bring-up (manual Part 8.6).

## License

MIT License — see [LICENSE](LICENSE). The optional `third_party/`
dependencies (nlohmann/json, stb) retain their own licenses.

## Acknowledgments

- OpenMP — parallel processing framework
- STB Image — image loading utilities (optional, for legacy demo)
- nlohmann/json — JSON parsing library (optional, for legacy demo)

---

**Note**: This system is designed for research and development. Always
test thoroughly in simulation before deploying to real hardware.
