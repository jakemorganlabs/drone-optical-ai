#!/usr/bin/env bash
# SITL quickstart -- run the full onboard stack against a simulated PX4 drone
# with zero hardware. Implements Option A manual Part 3.4 + Part 8.6.
#
# What it does:
#   1. Clones PX4-Autopilot (recursive) if PX4_DIR doesn't already point at one.
#   2. Builds + launches PX4 SITL Gazebo (gz_x500) in a subshell. That spins up a
#      simulated quad exposing a MAVLink UDP endpoint at 127.0.0.1:14540.
#   3. Waits for that UDP endpoint to accept a connection.
#   4. Launches the onboard `pilot_main` pointed at udp://:14540.
#
# Idempotent: re-running reuses the existing PX4 clone and just relaunches SITL
# + pilot. The PX4 clone location is overridable via the PX4_DIR env var. This
# script does NOT clone during script authoring; you run it.
#
# Usage:   ./sim/quickstart.sh
# Env:     PX4_DIR     (default: $PWD/sim/PX4-Autopilot)
#          PILOT_BIN   (default: $PWD/build/bin/pilot_main)
set -euo pipefail

# Absolute path of the repo root, no matter where this script is invoked from.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PX4_DIR="${PX4_DIR:-${REPO_ROOT}/sim/PX4-Autopilot}"
PILOT_BIN="${PILOT_BIN:-${REPO_ROOT}/build/bin/pilot_main}"
SITL_TARGET="px4_sitl gz_x500"
SITL_PORT="${SITL_PORT:-14540}"   # UDP port the sim exposes; matches FcBridge default.

log() { echo "[quickstart] $*"; }

# ---------------------------------------------------------------------------
# 1. Ensure PX4-Autopilot is present.
# ---------------------------------------------------------------------------
if [[ ! -d "${PX4_DIR}/.git" ]]; then
  log "PX4 not found at ${PX4_DIR}; cloning (recursive, this takes a while)..."
  mkdir -p "$(dirname "${PX4_DIR}")"
  git clone --recursive https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
else
  log "Reusing existing PX4 clone at ${PX4_DIR}"
fi

# ---------------------------------------------------------------------------
# 2. Launch PX4 SITL in its own subshell. Killing this script kills SITL too.
# ---------------------------------------------------------------------------
SITL_LOG="${REPO_ROOT}/build/sitl.log"
mkdir -p "${REPO_ROOT}/build"
log "Starting PX4 SITL (${SITL_TARGET}) -> logs to ${SITL_LOG}"
(
  cd "${PX4_DIR}"
  make "${SITL_TARGET}"
) >"${SITL_LOG}" 2>&1 &
SITL_PID=$!

cleanup() {
  log "Tearing down SITL (pid ${SITL_PID})..."
  kill "${SITL_PID}" 2>/dev/null || true
  wait "${SITL_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# 3. Wait for the SITL UDP endpoint at 127.0.0.1:${SITL_PORT}.
#    nc -z is the cheapest portable probe; fall back to a Python one-liner if
#    netcat is missing.
# ---------------------------------------------------------------------------
probe_udp() {
  if command -v nc >/dev/null 2>&1; then
    nc -z -w1 127.0.0.1 "${SITL_PORT}" 2>/dev/null
  else
    python3 - <<PY 2>/dev/null || return 1
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(1.0)
try:
    s.sendto(b"\x00", ("127.0.0.1", ${SITL_PORT}))
    s.recvfrom(1)
    sys.exit(0)
except Exception:
    sys.exit(1)
PY
  fi
}

log "Waiting for SITL UDP endpoint 127.0.0.1:${SITL_PORT} ..."
for _ in $(seq 1 120); do   # up to ~2 minutes
  if probe_udp; then
    log "SITL endpoint is up."
    break
  fi
  if ! kill -0 "${SITL_PID}" 2>/dev/null; then
    log "SITL process exited early; see ${SITL_LOG}"
    exit 1
  fi
  sleep 1
else
  log "Timed out waiting for SITL endpoint; see ${SITL_LOG}"
  exit 1
fi

# ---------------------------------------------------------------------------
# 4. Launch the onboard pilot pointed at the sim.
# ---------------------------------------------------------------------------
if [[ ! -x "${PILOT_BIN}" ]]; then
  log "pilot_main not found at ${PILOT_BIN}."
  log "Build it first (MAVSDK required):  make pilot_main   or   cmake --build build --target pilot_main"
  exit 1
fi

log "Launching ${PILOT_BIN} -> udp://:${SITL_PORT}"
exec "${PILOT_BIN}" --fc-url "udp://:${SITL_PORT}"