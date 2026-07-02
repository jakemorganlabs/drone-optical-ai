# pi-gen stage — `stage-dronectl`

This stage bakes the flashable `dronectl-pi.img.xz` (manual Part 8.3) on top of
the upstream [pi-gen](https://github.com/RPi-Distro/pi-gen) image builder.

## What the stage does

`stage-dronectl/` is a custom pi-gen stage dropped into a pi-gen fork (or a
checkout of `RPi-Distro/pi-gen`). It runs in `pi-gen`'s normal stage order
(each stage is a directory of numbered `00-pkg`, `01-files`, … scripts run
chrooted inside the image being built). Concretely it:

1. **apt installs runtime deps** (no build toolchain left in the final image):
   - `libcamera` + `libcamera-apps` (rpicam-vid for the mono path)
   - `onnxruntime` (CfC inference on the Pi)
   - `libmavsdk-dev` runtime (`mavsdk` library; the `-dev` is dropped after
     build if you cross-compile inside the stage)
   - `mavlink-router` (FC MAVLink multiplexer, Part 3.2)
   - `hostapd`, `dnsmasq` (first-boot wizard AP, Part 8.4)
   - `python3` (stdlib-only wizard + `dronectl` CLI)
2. **Copies build artifacts** into the image:
   - `build/embedded_voxel_mapper` → `/opt/dronectl/bin/`
   - `build/pilot_main`              → `/opt/dronectl/bin/pilot_main`
   - `cfc_policy.onnx`               → `/opt/dronectl/cfc_policy.onnx`
   - Phase-4 tether agent: see *Install-name convention* below.
3. **Installs the `dronectl` CLI** at `/usr/local/bin/dronectl`
   (from `image/dronectl` in this repo).
4. **Installs the first-boot wizard** at `/opt/dronectl/bin/wizard`
   (from `image/wizard/wizard.py`).
5. **Drops systemd units** from `image/systemd/` to
   `/etc/systemd/system/` and enables them with `systemctl enable`:
   - `dronectl-pilot.service`
   - `mavlink-router.service`
   - `tether-agent.service`
   - `dronectl-wizard.service` (first-boot; self-disables after the wizard
     writes `/etc/dronectl/config.yaml`)
6. **Drops the default config** `image/config/config.yaml` →
   `/etc/dronectl/config.yaml` **only as a placeholder** for the wizard to
   overwrite. (If you want a non-wizarded fleet deploy, ship your own
   `config.yaml` and the wizard unit's `ConditionPathExists=!` keeps it
   from running at all.)
7. **Uses pi-gen's `export-image` / `prerun.sh`** to deliver the final
   `deploy/*.img`; `build.sh` then `xz -9`s it for GitHub Releases.

### Install-name convention: `tether-agent` (kebab-case)

The Phase 4 Makefile builds the tether agent as `build/tether_agent`
(snake_case, matching the source `tether_agent.cpp`). The systemd unit
(`image/systemd/tether-agent.service`) and the `dronectl status` health probe
both reference the kebab-case install path `/opt/dronectl/bin/tether-agent`.
The stage reconciles this by **symlinking** (preferred) or renaming:

```sh
install -m 0755 build/tether_agent       /opt/dronectl/bin/tether_agent
ln -sf tether_agent                       /opt/dronectl/bin/tether-agent
```

Pick one and keep it consistent — the symlink approach is recommended because
it leaves the upstream build target name unchanged. `dronectl status` and the
systemd unit both only ever reference `/opt/dronectl/bin/tether-agent`.

### Cross-compile vs on-Pi build

The C++ build (`pilot_main`, `embedded_voxel_mapper`) is aarch64. Two viable
paths (pick per your CI):

- **Cross-compile** in a container (`aarch64-linux-gnu-g++`, the `MAVSDK` +
  `onnxruntime` aarch64 tarballs) and `install` the resulting binaries in the
  stage. Fastest; pin the exact `MAVSDK`/`onnxruntime` versions.
- **Build on a Pi** in CI (QEMU or self-hosted Pi 5 runner) and `scp` the
  artifacts into the pi-gen work tree before staging.

Either way the stage itself never recompiles — it only lays files down.

## Files in this repo

```
image/
├── systemd/
│   ├── dronectl-pilot.service
│   ├── mavlink-router.service
│   ├── tether-agent.service
│   └── dronectl-wizard.service
├── config/config.yaml              # /etc/dronectl/config.yaml default
├── wizard/
│   ├── wizard.py                   # first-boot AP wizard (stdlib only)
│   ├── hostapd.conf                # stub, filled at runtime by wizard
│   └── dnsmasq.conf                # stub, filled at runtime by wizard
├── dronectl                        # /usr/local/bin/dronectl CLI (stdlib only)
└── pi-gen/
    ├── README.md                   # this file
    └── build.sh                    # skeleton: stages this repo into pi-gen
```

## Skeleton stage layout (what `build.sh` writes into a pi-gen fork)

```
pi-gen/
├── build.sh                                    # upstream pi-gen entry
└── stage-dronectl/
    ├── 00-packages                             # apt line: libcamera onnxruntime …
    ├── 01-install-dronectl.files               # copies build artifacts + wizard + dronectl
    ├── 02-systemd.units                        # enables dronectl-*.service
    └── 03-drop-config.files                   # /etc/dronectl/config.yaml
```

(Real stage script bodies are written by `build.sh` from a small template
skeleton here; it is intentionally not committed — every fleet has its own
artifact paths. See `build.sh` for the exact heredocs.)

## Running it

```sh
# 1. Clone a pi-gen fork (this repo only supplies the stage):
git clone https://github.com/RPi-Distro/pi-gen.git
cd pi-gen

# 2. Build the project's native artifacts somewhere first (aarch64 or on a Pi),
#    so that build/pilot_main, build/embedded_voxel_mapper,
#    build/tether_agent, and cfc_policy.onnx all exist.

# 3. Stage this repo's stage-dronectl/ in and build:
/path/to/drone-optical-ai/image/pi-gen/build.sh \
    --repo-root /path/to/drone-optical-ai \
    --artifacts  /path/to/aarch64-build-dir

# 4. Compress for release:
xz -9 deploy/*.img     # -> deploy/dronectl-pi.img.xz
```

`build.sh` writes `stage-dronectl/` into the pi-gen checkout, then invokes
the upstream `./build.sh`. See [image/pi-gen/build.sh](build.sh) for the
skeleton.

## Reference

- Upstream pi-gen: https://github.com/RPi-Distro/pi-gen
- Stage authoring docs: `pi-gen/README.md` (the `stage*` section) in that repo.

## GitHub Release flow

After `build.sh`, `xz -9 deploy/*.img` yields `dronectl-pi.img.xz`. Attach it
to a GitHub Release tagged e.g. `v0.X`; the root `README.md` "Packaging"
section points users here. A user flashes with Raspberry Pi Imager, boots,
joins the AP, runs the wizard, and flies.