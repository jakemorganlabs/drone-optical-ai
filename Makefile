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

# Directories
SRC_DIR     = .
BUILD_DIR   = build
INSTALL_DIR = /usr/local

# Source files
COMMON_HEADERS = math.hpp grid.hpp pose.hpp motion.hpp ray_marching.hpp clustering.hpp navigation.hpp
LIVE_MAPPER_SRC  = live_voxel_mapper.cpp
LEGACY_SRC       = ray_voxel.cpp

# Targets
EMBEDDED_TARGET = $(BUILD_DIR)/embedded_voxel_mapper
LEGACY_TARGET    = $(BUILD_DIR)/ray_voxel_legacy

# Default target
all: $(BUILD_DIR) $(EMBEDDED_TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Embedded target (live camera operation)
$(EMBEDDED_TARGET): $(LIVE_MAPPER_SRC) $(COMMON_HEADERS)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -o $@ $(LIVE_MAPPER_SRC)

# Legacy target (optional, requires third_party/ deps)
legacy: $(BUILD_DIR) $(LEGACY_TARGET)
$(LEGACY_TARGET): $(LEGACY_SRC) $(COMMON_HEADERS)
	@echo "Attempting legacy build (requires third_party/nlohmann/json.hpp and third_party/stb_image.h)..."
	@$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) -Ithird_party -o $@ $(LEGACY_SRC) 2>/dev/null || \
		echo "  skipped: third_party deps missing. See third_party/README.md"

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

# Quick smoke test - launch mapper and terminate after 3 seconds
test: $(EMBEDDED_TARGET)
	@echo "Running smoke test (3s)..."
	@timeout 3 $(EMBEDDED_TARGET) || (sleep 3 && pkill -f embedded_voxel_mapper || true)
	@echo "Smoke test completed"

# Python bindings
python-setup:
	pip install -e .

python-embedded:
	BUILD_EMBEDDED=1 pip install -e .

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
	@echo "  all          - Build embedded mapper (default)"
	@echo "  legacy       - Build legacy ray_voxel demo (optional third_party deps)"
	@echo "  debug        - Build with debug symbols"
	@echo "  test         - Run a 3s smoke test"
	@echo "  install      - Install to $(INSTALL_DIR)"
	@echo "  uninstall    - Remove from $(INSTALL_DIR)"
	@echo "  clean        - Clean build artifacts"
	@echo "  python-setup - Build Python bindings (desktop)"
	@echo "  python-embedded - Build Python bindings (embedded)"
	@echo "  docs         - Generate Doxygen docs"
	@echo "  help         - Show this help"
	@echo ""
	@echo "Variables:"
	@echo "  CXX          - C++ compiler (default: g++)"
	@echo "  CXXFLAGS     - Compiler flags"
	@echo "  BUILD_DIR    - Build directory (default: build)"
	@echo "  INSTALL_DIR  - Install directory (default: /usr/local)"

.PHONY: all debug legacy install uninstall clean distclean test python-setup python-embedded docs help