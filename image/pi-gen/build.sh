#!/usr/bin/env bash
# Skeleton pi-gen wrapper for the stage-dronectl stage (manual Part 8.3).
#
# Symlinks this repo's stage into a pi-gen fork's stage-dronectl/ directory
# and invokes the upstream pi-gen ./build.sh, then xz -9s the resulting .img.
#
# USAGE:
#   ./build.sh --repo-root <drone-optical-ai checkout> \
#              [--pi-gen <pi-gen checkout>] \
#              [--artifacts <dir with build/pilot_main etc>] \
#              [--release]
#
# Defaults:
#   --pi-gen   ./pi-gen   (a checkout of https://github.com/RPi-Distro/pi-gen)
#   --artifacts  <repo-root>/build
#
# This skeleton is intentionally not exhaustive — it writes the stage scripts
# from heredocs so a fleet operator can edit them in place. It assumes the
# native aarch64 artifacts already exist (built by you, not by this script).
set -euo pipefail

REPO_ROOT=""
PIGEN="./pi-gen"
ARTIFACTS=""
RELEASE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root)  REPO_ROOT="$2"; shift 2;;
    --pi-gen)     PIGEN="$2"; shift 2;;
    --artifacts)  ARTIFACTS="$2"; shift 2;;
    --release)    RELEASE=1; shift;;
    -h|--help)
      sed -n '2,18p' "$0"; exit 0;;
    *) echo "unknown arg: $1" >&2; exit 1;;
  esac
done

[[ -n "$REPO_ROOT" ]] || { echo "ERROR: --repo-root is required" >&2; exit 1; }
[[ -d "$REPO_ROOT/image" ]] || { echo "ERROR: $REPO_ROOT/image not found" >&2; exit 1; }
ARTIFACTS="${ARTIFACTS:-$REPO_ROOT/build}"

if [[ ! -d "$PIGEN" ]]; then
  echo "ERROR: pi-gen checkout not found at $PIGEN" >&2
  echo "Clone it: git clone https://github.com/RPi-Distro/pi-gen.git $PIGEN" >&2
  exit 1
fi

STAGE="$PIGEN/stage-dronectl"
mkdir -p "$STAGE"

# 00-packages — apt installs runtime deps.
cat > "$STAGE/00-packages" <<'EOF'
libcamera libcamera-apps
onnxruntime
libmavsdk-dev
mavlink-router
hostapd dnsmasq
python3
iproute2
EOF

# 01-install-dronectl.files — drop build artifacts + wizard + dronectl CLI.
mkdir -p "$STAGE/01-install-dronectl.files/opt/dronectl/bin"
mkdir -p "$STAGE/01-install-dronectl.files/opt/dronectl"
mkdir -p "$STAGE/01-install-dronectl.files/usr/local/bin"

cp "$REPO_ROOT/image/wizard/wizard.py"   "$STAGE/01-install-dronectl.files/opt/dronectl/bin/wizard"
chmod +x "$STAGE/01-install-dronectl.files/opt/dronectl/bin/wizard"

cp "$REPO_ROOT/image/dronectl"           "$STAGE/01-install-dronectl.files/usr/local/bin/dronectl"
chmod +x "$STAGE/01-install-dronectl.files/usr/local/bin/dronectl"

for binname in embedded_voxel_mapper pilot_main tether_agent; do
  if [[ -f "$ARTIFACTS/$binname" ]]; then
    cp "$ARTIFACTS/$binname" "$STAGE/01-install-dronectl.files/opt/dronectl/bin/"
  else
    echo "WARN: $ARTIFACTS/$binname missing — unit will restart-loop until a later image flash" >&2
  fi
done
# kebab-case symlink for the tether agent (see README).
ln -sf tether_agent "$STAGE/01-install-dronectl.files/opt/dronectl/bin/tether-agent" || true

if [[ -f "$ARTIFACTS/cfc_policy.onnx" ]]; then
  cp "$ARTIFACTS/cfc_policy.onnx" "$STAGE/01-install-dronectl.files/opt/dronectl/"
else
  echo "WARN: $ARTIFACTS/cfc_policy.onnx missing — CfC pilot will fault on boot" >&2
fi

# 02-systemd.units — drop + enable units.
mkdir -p "$STAGE/02-systemd.units/etc/systemd/system"
cp "$REPO_ROOT"/image/systemd/*.service \
   "$STAGE/02-systemd.units/etc/systemd/system/"
cat > "$STAGE/02-systemd.units/00-run" <<'EOF'
#!/bin/bash -e
systemctl enable dronectl-pilot.service
systemctl enable mavlink-router.service
systemctl enable tether-agent.service
systemctl enable dronectl-wizard.service
EOF
chmod +x "$STAGE/02-systemd.units/00-run"

# 03-drop-config.files — default config placeholder for the wizard.
mkdir -p "$STAGE/03-drop-config.files/etc/dronectl"
cp "$REPO_ROOT/image/config/config.yaml" \
   "$STAGE/03-drop-config.files/etc/dronectl/config.yaml"

# Also ship the hostapd/dnsmasq stubs with the wizard.
mkdir -p "$STAGE/03-drop-config.files/opt/dronectl/wizard"
cp "$REPO_ROOT"/image/wizard/hostapd.conf "$STAGE/03-drop-config.files/opt/dronectl/wizard/"
cp "$REPO_ROOT"/image/wizard/dnsmasq.conf "$STAGE/03-drop-config.files/opt/dronectl/wizard/"

echo "stage-dronectl staged at: $STAGE"

cd "$PIGEN"
echo "invoking pi-gen ./build.sh …"
./build.sh

if [[ "$RELEASE" -eq 1 ]]; then
  echo "compressing deploy/*.img -> .img.xz"
  xz -9 -k deploy/*.img
  echo "release artifact: $(ls deploy/*.img.xz)"
fi