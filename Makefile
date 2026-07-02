# Makefile for Live Voxel Mapping System
# Compatible with Linux (incl. Raspberry Pi), macOS, and WSL.
# OpenMP is optional; the project compiles single-threaded when it is unavailable.

# Compiler and flags
CXX       ?= g++
CXXFLAGS  ?= -std=c++17 -Wall -Wextra -O3
DEBUG_FLAGS  = -g -O0 -DDEBUG
RELEASE_FLAGS = -O3 -DNDEBUG

# Try to enable OpenMP (silently fall back if not supported, e.g. Apple clang without libomp)
OPENMP_FLAGS := $(shell echo 'int main(){return 0;}' | $(CXX) -x c++ -fopenmp - -o /dev/null 2>/dev/null && echo '-fopenmp' || echo '')
ifneq ($(OPENMP_FLAGS),)
    CXXFLAGS += $(OPENMP_FLAGS)
endif

# Optional MAVSDK flight-controller bridge: probe whether we can compile+link a
# tiny TU that pulls in <mavsdk/mavsdk.h> against -lmavsdk. Same shape as the
# OPENMP probe above. `make` without MAVSDK just skips `pilot_main` (and the
# default `all` target stays the embedded mapper only).
MAVSDK_PROBE := $(shell printf '%s\n' '\#include <mavsdk/mavsdk.h>' > /tmp/_mavsdk_probe.cpp && printf '%s\n' 'int main(){mavsdk::Mavsdk m; (void)m; return 0;}' >> /tmp/_mavsdk_probe.cpp && $(CXX) -std=c++17 -x c++ /tmp/_mavsdk_probe.cpp -lmavsdk -o /dev/null 2>/dev/null && echo '-lmavsdk' || echo '')
ifneq ($(MAVSDK_PROBE),)
    MAVSDK_FLAGS := $(MAVSDK_PROBE)
    HAVE_MAVSDK := 1
endif

# Optional onnxruntime (ORT) CfC inference: probe whether we can compile+link a
# tiny TU pulling in <onnxruntime_cxx_api.h> against -lonnxruntime. Same shape
# as the MAVSDK / OPENMP probes. When found, set DRONECTL_HAVE_ORT so the CfC
# pilot compiles in the real ONNX inference path; otherwise the no-op CfC
# (returns HOLD) keeps `make pilot_main` working on dev machines.
ORT_PROBE := $(shell printf '%s\n' '\#include <onnxruntime_cxx_api.h>' > /tmp/_ort_probe.cpp && printf '%s\n' 'int main(){ Ort::Env e(ORT_LOGGING_LEVEL_WARNING,"x"); (void)e; return 0; }' >> /tmp/_ort_probe.cpp && $(CXX) -std=c++17 -x c++ /tmp/_ort_probe.cpp -lonnxruntime -o /dev/null 2>/dev/null && echo '-lonnxruntime' || echo '')
ifneq ($(ORT_PROBE),)
    ORT_FLAGS := $(ORT_PROBE)
    HAVE_ORT := 1
endif

# Directories
SRC_DIR     = .
CORE_DIR    = core
BUILD_DIR   = build
INSTALL_DIR = /usr/local

# Include path: core/ holds the header-only voxelcore interface library, so bare
#include "math.hpp" / "pose.hpp" / etc. resolve from any translation unit.
INCLUDES    = -I$(SRC_DIR) -I$(CORE_DIR)
CXXFLAGS    += $(INCLUDES)

# Source files (Phase-0 headers live in core/ now; the voxelcore INTERFACE lib)
COMMON_HEADERS = $(CORE_DIR)/math.hpp $(CORE_DIR)/grid.hpp $(CORE_DIR)/pose.hpp \
                 $(CORE_DIR)/motion.hpp $(CORE_DIR)/ray_marching.hpp \
                 $(CORE_DIR)/clustering.hpp $(CORE_DIR)/navigation.hpp
LIVE_MAPPER_SRC  = $(CORE_DIR)/live_voxel_mapper.cpp
LEGACY_SRC       = $(CORE_DIR)/ray_voxel.cpp
PERCEPTION_SRC   = perception/drone_pose_provider.cpp
PERCEPTION_HDRS  = perception/drone_pose_provider.hpp perception/depth_occupancy.hpp

# Targets
EMBEDDED_TARGET = $(BUILD_DIR)/embedded_voxel_mapper
EMBEDDED_FULL_TARGET = $(BUILD_DIR)/embedded_voxel_mapper_full
LEGACY_TARGET    = $(BUILD_DIR)/ray_voxel_legacy

# Default target — keep the Phase 0 single-TU mapper as the default so the
# existing CI/smoke flow is unchanged.
all: $(BUILD_DIR) $(EMBEDDED_TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Embedded target (live camera operation) — Phase 0 single-TU default, unchanged.
$(EMBEDDED_TARGET): $(LIVE_MAPPER_SRC) $(COMMON_HEADERS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -o $@ $(LIVE_MAPPER_SRC)

# Full mapper: links the perception TU alongside the mapper so the new
# translation unit is exercised by the two-TU ODR gate implicitly. Does not
# replace the single-TU default; build explicitly with `make full`.
full: $(BUILD_DIR) $(EMBEDDED_FULL_TARGET)
$(EMBEDDED_FULL_TARGET): $(LIVE_MAPPER_SRC) $(PERCEPTION_SRC) $(COMMON_HEADERS) $(PERCEPTION_HDRS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -c $(PERCEPTION_SRC) -o $(BUILD_DIR)/drone_pose_provider.o
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -c $(LIVE_MAPPER_SRC) -o $(BUILD_DIR)/live_voxel_mapper_full.o
	$(CXX) $(OPENMP_FLAGS) -o $@ $(BUILD_DIR)/live_voxel_mapper_full.o $(BUILD_DIR)/drone_pose_provider.o

# Legacy target (optional, requires third_party/ deps)
legacy: $(BUILD_DIR) $(LEGACY_TARGET)
$(LEGACY_TARGET): $(LEGACY_SRC) $(COMMON_HEADERS) | $(BUILD_DIR)
	@echo "Attempting legacy build (requires third_party/nlohmann/json.hpp and third_party/stb_image.h)..."
	@$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -Ithird_party -o $@ $(LEGACY_SRC) 2>/dev/null || \
		echo "  skipped: third_party deps missing. See third_party/README.md"

# Pilot main: Phase 3 onboard loop. Composes DronePoseProvider + FcBridge
# (optional MAVSDK) + CfCPilot (optional onnxruntime) + Failsafe + DualVoxelGrid
# and runs a 20 Hz control loop. Header-only CfC pieces (observation.hpp +
# cfc_pilot.hpp) are pulled in via -Icontrol/cfc_pilot. Without MAVSDK the
# FcBridge TU is omitted; without ORT the CfC no-op (HOLD) is used. Either
# way `make pilot_main` produces a working binary.
PILOT_TARGET    = $(BUILD_DIR)/pilot_main
PILOT_STUB_SRC  = control/pilot_main.cpp
PILOT_HDRS      = control/cfc_pilot/observation.hpp control/cfc_pilot/cfc_pilot.hpp
FC_BRIDGE_SRC   = control/fc_bridge/fc_bridge.cpp

# Per-target defines for the optional autonomy backends.
PILOT_DEFS :=
ifdef HAVE_MAVSDK
    PILOT_DEFS += -DDRONECTL_HAVE_MAVSDK
endif
ifdef HAVE_ORT
    PILOT_DEFS += -DDRONECTL_HAVE_ORT
endif

pilot_main: $(BUILD_DIR) $(PILOT_TARGET)
$(PILOT_TARGET): $(PILOT_STUB_SRC) $(PERCEPTION_SRC) $(COMMON_HEADERS) $(PERCEPTION_HDRS) $(PILOT_HDRS) control/fc_bridge/fc_bridge.hpp control/failsafe.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -c $(PERCEPTION_SRC) -o $(BUILD_DIR)/drone_pose_provider.o
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. $(PILOT_DEFS) -c $(PILOT_STUB_SRC) -o $(BUILD_DIR)/pilot_main.o
ifeq ($(MAVSDK_FLAGS),)
	@echo "Building pilot_main (no MAVSDK; FC bridge TU omitted)"
	$(CXX) $(OPENMP_FLAGS) -o $@ $(BUILD_DIR)/pilot_main.o $(BUILD_DIR)/drone_pose_provider.o $(ORT_FLAGS)
else
	@echo "Building pilot_main (MAVSDK available; linking FC bridge)"
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -Icontrol/fc_bridge -c $(FC_BRIDGE_SRC) -o $(BUILD_DIR)/fc_bridge.o
	$(CXX) $(OPENMP_FLAGS) -o $@ $(BUILD_DIR)/pilot_main.o $(BUILD_DIR)/drone_pose_provider.o $(BUILD_DIR)/fc_bridge.o $(MAVSDK_FLAGS) $(ORT_FLAGS)
endif

# Debug builds
debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: all

# Install
install: all
	install -d $(INSTALL_DIR)/bin
	install $(EMBEDDED_TARGET) $(INSTALL_DIR)/bin/
	install -d $(INSTALL_DIR)/include/voxel_mapper
	install $(COMMON_HEADERS) $(INSTALL_DIR)/include/voxel_mapper/

# Uninstall
uninstall:
	rm -f $(INSTALL_DIR)/bin/embedded_voxel_mapper
	rm -rf $(INSTALL_DIR)/include/voxel_mapper

# Clean
clean:
	rm -rf $(BUILD_DIR)
distclean: clean
	rm -f *.bin *.log *.out

# Quick smoke test - launch mapper for 2 seconds and check it produced frames
test: $(EMBEDDED_TARGET) $(BUILD_DIR)/smoke_test $(BUILD_DIR)/odr_two_tu $(BUILD_DIR)/perception_compile_test $(BUILD_DIR)/failsafe_test $(BUILD_DIR)/observation_test $(BUILD_DIR)/cfc_safety_test
	@echo "Running smoke test (mapper 2s)..."
	@./$(EMBEDDED_TARGET) --duration 2 --grid 48 --out $(BUILD_DIR)/make_ci_grid.bin > $(BUILD_DIR)/mapper.log 2>&1
	@grep -q "Total frames: [1-9]" $(BUILD_DIR)/mapper.log || { echo "FAIL: no frames processed"; cat $(BUILD_DIR)/mapper.log; exit 1; }
	@echo "Running unit smoke tests..."
	@./$(BUILD_DIR)/smoke_test
	@echo "Running two-TU ODR regression test..."
	@./$(BUILD_DIR)/odr_two_tu
	@echo "Running perception compile test..."
	@./$(BUILD_DIR)/perception_compile_test
	@echo "Running failsafe state-machine tests..."
	@./$(BUILD_DIR)/failsafe_test
	@echo "Running CfC observation builder tests..."
	@./$(BUILD_DIR)/observation_test
	@echo "Running CfC safety-filter tests..."
	@./$(BUILD_DIR)/cfc_safety_test
	@echo "All smoke tests passed."

# Unit smoke test binary (cross-header behavior checks)
$(BUILD_DIR)/smoke_test: tests/smoke_test.cpp $(COMMON_HEADERS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -o $@ tests/smoke_test.cpp

# Failsafe state-machine unit tests (Part 7). Header-only, so the test TU is a
# single self-contained compilation unit just like smoke_test.
$(BUILD_DIR)/failsafe_test: tests/failsafe_test.cpp control/failsafe.hpp $(COMMON_HEADERS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -o $@ tests/failsafe_test.cpp

# Phase 3 CfC observation builder unit tests (Part 4.1). Header-only
# observation.hpp included in core/, so the test TU is a single self-contained
# compilation unit. Compiles with or without onnxruntime.
$(BUILD_DIR)/observation_test: tests/observation_test.cpp control/cfc_pilot/observation.hpp $(COMMON_HEADERS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -o $@ tests/observation_test.cpp

# Phase 3 CfC safety-filter unit tests (Part 4.4). Exercises
# CfCPilot::safety_filter (a public static helper) against a tiny DualVoxelGrid.
# Does not require onnxruntime — the __has_include gate in cfc_pilot.hpp keeps
# it single-TU-compilable on machines without ORT.
$(BUILD_DIR)/cfc_safety_test: tests/cfc_safety_test.cpp control/cfc_pilot/cfc_pilot.hpp control/cfc_pilot/observation.hpp $(COMMON_HEADERS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -o $@ tests/cfc_safety_test.cpp

# Two-TU ODR regression: two TUs including all headers must link cleanly
$(BUILD_DIR)/odr_two_tu: tests/odr_two_tu_main.cpp tests/odr_two_tu_other.cpp $(COMMON_HEADERS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -c tests/odr_two_tu_other.cpp -o $(BUILD_DIR)/odr_other.o
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -c tests/odr_two_tu_main.cpp  -o $(BUILD_DIR)/odr_main.o
	$(CXX) $(OPENMP_FLAGS) -o $@ $(BUILD_DIR)/odr_main.o $(BUILD_DIR)/odr_other.o

# Phase 1 perception compile/link gate: links tests/perception_compile_test.cpp
# against the perception TU (drone_pose_provider.cpp) so the new headers+TU are
# exercised. Implicitly also a small two-TU ODR check for the perception code.
$(BUILD_DIR)/perception_compile_test: tests/perception_compile_test.cpp $(PERCEPTION_SRC) $(COMMON_HEADERS) $(PERCEPTION_HDRS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -c $(PERCEPTION_SRC) -o $(BUILD_DIR)/drone_pose_provider.o
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -I. -c tests/perception_compile_test.cpp -o $(BUILD_DIR)/perception_compile_test.o
	$(CXX) $(OPENMP_FLAGS) -o $@ $(BUILD_DIR)/perception_compile_test.o $(BUILD_DIR)/drone_pose_provider.o

# Documentation
docs:
	@if command -v doxygen >/dev/null 2>&1; then \
		doxygen Doxyfile 2>/dev/null || echo "Doxyfile not found, skipping"; \
		echo "Documentation generated in docs/html/"; \
	else \
		echo "Doxygen not found. Install doxygen to generate documentation."; \
	fi

# Help
help:
	@echo "Live Voxel Mapping System - Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  all          - Build embedded mapper (default, single-TU)"
	@echo "  full         - Build mapper + perception TU (two-TU ODR gate)"
	@echo "  pilot_main   - Build Phase 3 onboard pilot loop (+ MAVSDK FcBridge / +ORT CfC if found)"
	@echo "  legacy       - Build legacy ray_voxel demo (optional third_party deps)"
	@echo "  debug        - Build with debug symbols"
	@echo "  test         - Run smoke + ODR + perception + failsafe tests"
	@echo "  tether-test  - Build the Phase 4 tether-agent scaffold (optional)"
	@echo "  coordinator  - Build the Phase 5 ground coordinator scaffold (optional)"
	@echo "  install      - Install to $(INSTALL_DIR)"
	@echo "  uninstall    - Remove from $(INSTALL_DIR)"
	@echo "  clean        - Clean build artifacts"
	@echo "  docs         - Generate Doxygen docs"
	@echo "  help         - Show this help"
	@echo ""
	@echo "Variables:"
	@echo "  CXX          - C++ compiler (default: g++)"
	@echo "  CXXFLAGS     - Compiler flags"
	@echo "  BUILD_DIR    - Build directory (default: build)"
	@echo "  INSTALL_DIR  - Install directory (default: /usr/local)"

# Optional tether-agent scaffold (Phase 4 link plumbing; NOT in the
# main build/test targets per manual Part 9.6 step 5 — link comes after
# autonomy). Builds link/tether_agent/tether_agent.cpp into a self-test
# binary if a POSIX-socket compile succeeds — which it always does on
# Linux/macOS where this repo targets. The build rule itself is the probe:
# if the compile fails, `make tether-test` fails normally; if it succeeds
# the self-test binary lands in $(BUILD_DIR).
TETHER_SRC   = link/tether_agent/tether_agent.cpp
TETHER_HDR   = link/tether_agent/tether_agent.hpp
TETHER_TGT   = $(BUILD_DIR)/tether_agent

tether-test: $(TETHER_TGT)
$(TETHER_TGT): $(TETHER_SRC) $(TETHER_HDR) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I. -DTETHER_AGENT_TEST_MAIN -o $@ $(TETHER_SRC)

# Optional per-drone ground coordinator (Phase 5 link/ground plumbing;
# manual Part 6.1). NOT in the main build/test targets — same shape as
# tether-test: build only on demand. One coordinator process per drone; a
# multi-drone ground box runs N of these. The self-test binary dials one
# tether-agent over the fiber IP and serves a read-only /fleet scrape.
COORD_SRC    = ground/coordinator/coordinator.cpp
COORD_HDR    = ground/coordinator/coordinator.hpp
COORD_TGT    = $(BUILD_DIR)/coordinator

coordinator: $(COORD_TGT)
$(COORD_TGT): $(COORD_SRC) $(COORD_HDR) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I. -DCOORDINATOR_TEST_MAIN -o $@ $(COORD_SRC)

.PHONY: all full pilot_main debug legacy install uninstall clean distclean test docs help tether-test coordinator