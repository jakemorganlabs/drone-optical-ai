# Hardware — Bill of Materials (Option A)

Per the Option A build manual (Part 9.3). The design rule is simple and
load-bearing: **everything expensive lives on the ground and is preserved
across airframe losses; everything on the airframe is cheap and
expendable.** Lose a drone and you lose a Pi and an airframe, never the
Starlink terminal, the ground box, or the LLM server.

The tether itself is the other key piece — it is **RF-free** (see
[REGULATORY.md](REGULATORY.md)), which is the whole reason this BOM
exists in this shape.

## Per drone (expendable)

These items go down with the airframe. Budget for losing all of them.

| Item | Notes |
| --- | --- |
| Airframe + motors / ESCs | Sized with **tether-weight margin** — you are dragging a fiber spool + media converter + Pi + camera in addition to the battery. |
| Pixhawk-class flight controller | Runs **PX4** or **ArduPilot**. The Pi is the *companion computer*, never the autopilot — the FC owns the motors. |
| Raspberry Pi 5 (or CM5 + carrier w/ SFP cage) | The onboard pilot. CM5 carrier with an SFP cage removes the media-converter box and its ~1–2 W. |
| Depth camera | **Recommended.** A stereo or ToF depth sensor makes the avoider real. See "Recommended depth cameras" below. |
| Fiber payout spool + drone-side SFP / media converter | Single-mode fiber on a spool sized to your ceiling (1–5 km micro-fiber). Drone-side converter turns the optical signal into Ethernet for the Pi NIC. |
| Regulated 5 V supply | Powers the Pi + camera + converter from the flight battery. Cleanly regulated; brown-outs kill the Pi mid-flight. |

## Shared ground (preserved)

These items stay on the ground across many drone losses. Buy them once.

| Item | Notes |
| --- | --- |
| Ground-side converters (one per drone) + switch | Each drone's fiber lands in its own media converter; the switch fans them all into the ground box. |
| Ground box (NUC / mini-PC / Pi 5) | Runs the **coordinator** (one process per drone), hosts the operator UI, and bridges fibers up to the WAN. See "Ground box minimum specs" below. |
| One Starlink terminal | The single shared uplink. This is the "many drones, one terminal" property — telem + map deltas scale to many drones; simultaneous HD video does not. See [FLEET.md](FLEET.md). |
| Self-hosted AI server (with a public / rendezvous endpoint) | Hosts the LLM mission brain + map fusion. The ground box dials out to it (Starlink is often CGNAT, so the server holds the public endpoint; or use Tailscale / Headscale). WireGuard over Starlink is the transport. |

## Recommended depth cameras

Per manual Part 0 (reality check #1): a single mono camera cannot recover
metric depth without motion parallax, which is fragile on a moving drone.
For reliable **real-time obstacle avoidance** you should populate
occupancy from a depth source. The mapper's ray-casting stays useful, but
the avoider consumes the occupancy grid — depth writes that grid directly.

| Model | Interface | Notes |
| --- | --- | --- |
| OAK-D (Luxonis) | USB / DepthAI SDK | Onboard depth + IMU; well-supported on aarch64. Top recommendation for Option A. |
| Intel RealSense D4xx (e.g. D435i, D455) | USB / `librealsense` | Per-pixel depth + (optional) IMU. Mature SDK, broadly deployed on Pi. |
| Pi Camera Module 3 (mono) | CSI / libcamera | Fallback path only — works for structure-from-motion mapping but is fragile for real-time avoidance. |

The training / observation side (see `control/cfc_pilot/observation.hpp`)
consumes the occupancy grid, not raw pixels, so the CfC pipeline is
identical regardless of which depth camera you pick. The depth path is
called out where it matters in `perception/depth_occupancy.hpp`.

## Fiber payout recommendations

- **Single-mode** duplex (or single-strand BiDi SFP) fiber. multimode is
  not worth saving the few dollars — single-mode reaches further and is
  the only thing the cheap SFP modules expect.
- Spool sized to your **ceiling**, not your hopes. **1–5 km micro-fiber**
  on a payout reel is plenty for any legal-VLOS or tethered-UAS ceiling.
- Treat the link as **plain Ethernet/IP**. Static IPs, no DHCP, no
  spanning-tree surprises. See [TETHER.md](TETHER.md) for the `10.8.0.x`
  convention.
- Drone-side SFP → Ethernet media converter draws **~1–2 W**. A CM5
  carrier with an SFP cage removes that box and its cable; a Pi 5 with a
  USB/external converter is fine for the v0 image.

## Ground box minimum specs

The ground box is the coordinator host. It runs **one process per drone**
(telemetry merge + LLM-client fanout) and the operator UI. It is **not**
the inference host for the pilot — the Pi does that onboard. So the ground
box can be modest.

| Tier | CPU | RAM | Storage | Notes |
| --- | --- | --- | --- | --- |
| Minimum | Intel NUC (or equivalent 4-core mini-PC) / Raspberry Pi 5 | 8 GB | 64 GB SSD / SD | Comfortably runs the coordinator for 1–4 drones + the operator UI. |
| Recommended | Mini-PC w/ 4–8 cores | 16 GB | 256 GB NVMe | Headroom for the local map-fusion cache, the Starlink WireGuard tunnel, and a second drone. |
| Server (off ground, off-site) | Discrete server / cloud box | 32 GB+ | NVMe | Holds the LLM mission brain + the long-term map store. **This** is where the GPU lives; nothing flight-critical depends on it. |

The ground box and the AI server are usually two different machines. The
ground box sits next to the operator and the Starlink terminal; the AI
server can be anywhere with a public IP and dials in over WireGuard. See
[FLEET.md](FLEET.md) and [TETHER.md](TETHER.md) for the topology.