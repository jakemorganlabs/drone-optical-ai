# Makefile for Phase 5 ground-side + server-side scaffolds.
#
# This is NOT the root Makefile. Phase 5's deliverables are Python + a small
# standalone C++ binary sitting OUTSIDE the root `make`/`make test` flight-
# critical path (manual Phase 5 task #7 + constraints). The root Makefile is
# deliberately untouched.
#
# Targets:
#   coordinator  - build ground/coordinator/coordinator (C++17, -Wall -Wextra,
#                  stdlib + pthreads only; see ground/coordinator/README.md)
#   lint         - python3 -m py_compile over the Python files (smoke gate)
#   test         - lint + LLM hallucination guard smoke (forbidden fields
#                  reject) + the server/test_validator.py unittest suite
#                  (geofence/altitude/separation/per-goal override/hallucination)
#   clean        - remove build artifacts
#
# The Makefile is the smoke gate for CI: `cd ground && make test` must pass on
# a machine with only stdlib Python + a C++17 compiler.

CXX       ?= g++
CXXFLAGS  ?= -std=c++17 -Wall -Wextra -O2 -pthread

ROOT_DIR  := ..
COORD_DIR := coordinator
# server/ is a sibling of ground/ at the repo root (not under ground/).
SERVER_DIR := $(ROOT_DIR)/server
COORD_HDR := $(COORD_DIR)/coordinator.hpp
COORD_SRC := $(COORD_DIR)/coordinator.cpp
COORD_BIN := $(COORD_DIR)/coordinator

# mission_brain.py is the LLM brain + its hallucination guard; validator.py is
# the deterministic geometric + hallucination gate; test_validator.py is its
# stdlib unittest suite (geofence/altitude/separation/override/hallucination).
PY_FILES := $(SERVER_DIR)/mission_brain.py $(SERVER_DIR)/validator.py $(SERVER_DIR)/test_validator.py $(SERVER_DIR)/main.py
PY        ?= python3

.PHONY: all coordinator lint test clean help

all: coordinator

# ---------------------------------------------------------------------------
# C++ coordinator standalone binary. stdlib + pthreads only.
# ---------------------------------------------------------------------------
$(COORD_BIN): $(COORD_SRC) $(COORD_HDR)
	$(CXX) $(CXXFLAGS) -I$(COORD_DIR) -DCOORDINATOR_TEST_MAIN -o $@ $(COORD_SRC)

coordinator: $(COORD_BIN)

# ---------------------------------------------------------------------------
# Python smoke gate. `python3 -m py_compile` is the bar on a stdlib-only box
# because every heavy import (requests, fastapi, ollama-client, shapely, yaml)
# is guarded with try/except.
# ---------------------------------------------------------------------------
lint:
	@echo "py_compile: $(PY_FILES)"
	@$(PY) -m py_compile $(PY_FILES)
	@echo "py_compile OK"

# ---------------------------------------------------------------------------
# test: lint + the server/test_validator.py unittest suite. The suite covers
# geofence, altitude band, separation, per-goal override, AND the LLM
# hallucination guard (velocity/motor/throttle/pwm/landing_velocity reject).
# Run directly so there is no pytest dep.
# ---------------------------------------------------------------------------
test: lint
	@echo "validator unittest suite..."
	@$(PY) $(SERVER_DIR)/test_validator.py
	@echo "All Phase 5 ground/server tests passed."

clean:
	rm -f $(COORD_BIN)
	rm -rf $(SERVER_DIR)/__pycache__

help:
	@echo "Phase 5 ground/server scaffold Makefile"
	@echo "  coordinator  - build ground/coordinator/coordinator (C++17, stdlib+pthreads)"
	@echo "  lint         - python3 -m py_compile over the Python files"
	@echo "  test         - lint + validator.py CLI smoke cases"
	@echo "  clean        - remove build artifacts"