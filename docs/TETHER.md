# Tether — fiber-optic datalink

The jam-immune leash between the onboard Pi pilot and the ground box.
Carries telemetry up, goals + teleop overrides down, and optional
on-demand HD video. It is **not** in the flight-critical loop: if it
drops, the onboard CfC pilot (manual Part 4) keeps flying toward the last
goal, then runs the configured link-loss action. The drone does not fall.

This document is the operator-facing summary; the wire-format contract and
the POSIX-socket scaffold live at [`link/tether_agent/README.md`](../link/tether_agent/README.md).
See [HARDWARE.md](HARDWARE.md) for the BOM and [REGULATORY.md](REGULATORY.md)
for why a fiber link is regulatory-clean (it is RF-free).

## Hardware

The whole link is plain Ethernet over glass. Nothing on either end knows
it is "a drone".

| Side | Pieces | Notes |
| --- | --- | --- |
| Drone | fiber payout spool → **SFP → Ethernet media converter** (~1–2 W) → Pi NIC | Or a CM5 carrier with an SFP cage (no converter box). Spool sizes in [HARDWARE.md](HARDWARE.md). |
| Fiber | **single-mode** duplex (or single-strand BiDi SFP) on a payout reel | 1–5 km micro-fiber is more than any legal/tethered ceiling. |
| Ground | matching media converter → ground-box switch | One converter per drone; the switch fans them all into the ground box. |

Treat the link as plain Ethernet/IP. No DHCP, no spanning-tree, no bridge
loops. Static IPs only.

## Static IP convention

The fleet uses the `10.8.0.0/24` private range for the tether segment.
Each drone is assigned `.2`, `.3`, ... and the ground box is always `.1`
on its switch-facing interface.

| Host | Address | Notes |
| --- | --- | --- |
| Ground box (switch-facing) | `10.8.0.1` | Default gateway for the tether segment; coordinator listens here. |
| Drone `drone-01` | `10.8.0.2` | The default `self_ip` in `/etc/dronectl/config.yaml`. |
| Drone `drone-02` | `10.8.0.3` | Assigned by the wizard when a second airframe is added. |
| ... | `10.8.0.x` | One address per drone. Telemetry + map deltas scale; video does not. |

The Phase-4 scaffold (`link/tether_agent/tether_agent.cpp`) currently
binds on `127.0.0.1` and forwards over the fiber; a future phase exposes
the bind address as a config option. The IP scheme is pinned now so the
upgrade is mechanical.

## Link-loss contract

Per manual Part 5.3. The agent itself never imports
`control/failsafe.hpp`; the hooks are plain `std::function<void()>` so
Phase 6 lands the `Failsafe` class without recompiling the link.

- **Heartbeat cadence:** a telemetry frame every **200 ms** (5 Hz
  heartbeat, faster on real telemetry up to ~20 Hz).
- **Loss detector:** **1 s of silence** from the ground (no inbound line)
  → the agent fires `on_tether_loss()` **once** and enters `LINK_LOSS`.
- **It does not disconnect or stop.** It keeps the listener bound, keeps
  emitting heartbeat telemetry, and keeps the session socket open so the
  ground can re-attach on the same TCP connection if the silence was a
  transient stall. If the peer resets, it closes the dead socket and
  loops back to `accept()` for a fresh reconnect.
- **On reconnect:** the first good inbound frame after a loss fires
  `on_tether_recover()` **once**, re-syncs the map, and resumes goals
  cleanly (idempotent goal IDs — see the JSON-line / proto contract at
  [`link/tether_agent/README.md`](../link/tether_agent/README.md)).
- **Flight behavior on loss:** the failsafe state machine transitions
  `AUTONOMOUS → LINK_LOSS` and the onboard CfC keeps flying toward the
  last goal, then runs the configured link-loss action (`hover` /
  `return-along-track` / `land`). **The drone does not fall.** See
  `control/failsafe.hpp` (`on_tether_loss`, `on_tether_recover`).

## Backhaul independence

The tether and the backhaul (Starlink + WireGuard to the AI server, see
[FLEET.md](FLEET.md)) are independent links.

- **Tether up, backhaul down:** each drone continues its last goal on
  onboard CfC; the ground operator still has full fiber teleop. No
  freeze, no fall.
- **Tether down, backhaul up:** the drone runs its link-loss action; the
  ground box still has all the other drones.

The tether is in the operator's hand. The backhaul is to the LLM brain.
Loss of the second one is a *mission* event, not a *flight* event.