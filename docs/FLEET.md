# FLEET.md — many drones, one Starlink terminal (manual Part 6.4)

## The "many drones, one terminal" property

Option A's ground topology is:

```
   drone-01 ──fiber──►┐
   drone-02 ──fiber──►├──[switch]── ground box ── wg0 ─── Starlink ─── self-hosted AI server
   drone-NN ──fiber──►┘   (one port    [coordinator    (Tier-3 VPN,    [LLM mission brain]
                            per drone)  + tether          non-flight)    server/mission_brain.py
                                       clients :7400]
```

The thing each drone needs its own copy of is: **airframe, Pi, depth camera,
fiber payout spool + drone-side SFP converter, one switch port, one `/30` on
the fiber link**. Everything fiber/switch scales linearly with the fleet —
it's cheap and dedicated.

The **shared, scarce resource** is the single Starlink terminal. That is the
point of the "many drones, one terminal" framing: you add drones by adding
fiber, not by adding expensive WAN radios.

## What scales to many drones, and what does not

| Stream                          | Scales to N drones? | Why                                                                                       | Design rule (Phase 5)                            |
|---------------------------------|:------------------:|-------------------------------------------------------------------------------------------|--------------------------------------------------|
| **Telemetry** (pose, battery, failsafe state) | YES (~20 Hz × N × small JSON line) | A pose+battery+state telemetry frame is ~100 bytes. 20 Hz × 100 drones = ~2 MB/s — trivial over fiber + negligible on Starlink. | Coordinator fans this in by default (`WorldModel::fuse`). |
| **Map deltas** (occupancy-grid deltas up)    | YES (sparse, on change)                | Map deltas are small, sparse, and only sent on change. Fuse into the shared world model; re-task the LLM over the union. Low-rate enough that it rides `wg0` just fine. | `DroneState.map_delta_b64` (opaque) → `WorldModel`. Phase 7 will swap opaque-hold for a real voxel merge. |
| **Goal plans** (down)            | YES (one per re-task, rare)            | Re-tasking is a low-rate event (human-thought cadence over Starlink). One tiny JSON plan per re-task. | `POST /plan` from the LLM brain to the coordinator, fan-out per drone. |
| **HD video**                     | **NO**                                 | Simultaneous HD video from every drone does not scale — it saturates the one Starlink uplink. | **Video is on-demand.** Pull a single drone's stream only when an operator wants it; never broadcast the fleet video. Telemetry + map deltas are the persistent uplink. |

So the fleet uplink budget on Starlink is **telemetry + map deltas + an
occasional HD stream from one drone** — never HD from all of them. This is
manual Part 6.4 ("Telemetry + map deltas scale to many drones; simultaneous HD
video does not — make video on-demand") made concrete.

## Graceful-degradation matrix (manual Part 6.4)

The whole design turns a brittle uplink into a benign one. The drones never
depend on Starlink for flight; Starlink only buys you mission-level
re-tasking.

| Backhaul state   | Fleet reasoning / re-tasking              | Per-drone flight behavior                              | Operator teleop                         | Net effect on the mission                              |
|------------------|-------------------------------------------|--------------------------------------------------------|-----------------------------------------|--------------------------------------------------------|
| **Starlink up**  | Full fleet reasoning; dynamic re-tasking on telemetry / map deltas. LLM brain + C++ coordinator + validator all live. | Goals flow down to drones via the coordinator + tether-agent. CfC flies each drone to its goal. | Full fiber teleop available (operator override at any time). | Nominal — the Option A promise.                        |
| **Starlink down** (`wg0` down) | LLM brain unreachable; **no new goals** are emitted. The world model stops refreshing from the brain side. | Each drone **continues its last goal** on the onboard CfC. Onboard CfC + deterministic planner handle obstacle avoidance locally. No freeze, no fall. | **Operator still has full fiber teleop** end-to-end (fiber is independent of Starlink). The operator can fly, override, or kill any drone directly over the tether. | Fleet degrades to "frozen plan + operator teleop" — safe, controllable, recoverable. |
| **wg0 down + fiber down on one drone** | That drone is in `LINK_LOSS` per the failsafe machine (Part 7). It flies its onboard link-loss action (hover / return-along-track / land). The other drones keep flying. | Battery-only autonomy for that drone, then RTL/land per the configured link-loss action. | Operator teleop for the other drones is intact; only the one drone is lost-link. | One drone is "lost-link" — never "fallen out of the sky". Recoverable on fiber re-splice / reconnect. |

### Why this is the right shape

- **No flight-critical dependency on Starlink.** The closed loop is
  onboard-CfC → safety filter → FC bridge. Starlink only feeds the
  mission-level brain, which is a re-tasking optimization, not a flight
  necessity.
- **No flight-critical dependency on the LLM.** The C++ coordinator's
  `Validator` rejects anything unsafe the LLM says; if the LLM says nothing
  (Starlink down), the drones keep their last validated goal. The worst the
  LLM can do is stop improving the mission.
- **Fiber and Starlink fail independently.** Starlink is one WAN hop; fiber
  is the box↔airframe leash. Losing Starlink does not touch fiber, so the
  operator keeps direct teleop on every drone — including the kill switch,
  which is highest-priority and never gated (Part 7).

### Test it (the manual's bar)

- **Starlink-up path:** with `wg0` up, POST a mission to
  `server/mission_brain.py` `/mission`; verify the validated plan reaches the
  coordinator's `/plan` and is fanned out to the tether-agents; verify the
  world model refreshes on the brain side via `/world`.
- **Starlink-down test (manual Part 6.4):** mid-mission, `sudo wg-quick down
  wg0` on the ground box. Expect: drones keep flying toward their last goal
  on the onboard CfC; operator fiber teleop is unaffected; no freeze, no
  fall. Bring `wg0` back up; the drones resume re-tasking — no re-arm, no
  re-sync dance (idempotent goal IDs over the tether dedup on reconnect).

## TODOs for Phase 7 packaging

- A real `/world` streaming path (SSE / WebSocket) so the LLM context window
  is fed continuously rather than polled per-mission.
- Fleet-level observability (per-drone last-seen, failsafe-state histogram)
  on a small status page. Not flight-critical; lives on the Tier-3 VPN.
- Operator-to-many-aircraft control ratios (FAA Part 107.39 / BVLOS waiver
  territory, manual Part 9.4) — this is a regulatory constraint, not a
  technical one. The architecture degrades safely to one-operator-per-drone
  if the waiver authorizes no more.
- Real voxel-map merge across N drones for true shared-map re-tasking; the
  Phase 5 `WorldModel` holds opaque per-drone deltas until Phase 7 swaps in
  the merge.