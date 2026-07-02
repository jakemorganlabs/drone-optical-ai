# Safety — the test ladder and pre-flight card

Never skip a rung. Manual Part 9.5, verbatim. Every rung builds on the
trust earned by the rung below it. Skipping a rung to "save time" is the
most expensive thing you can do in this project.

The failsafe state machine (`control/failsafe.hpp`) is consulted
everywhere; it owns link-loss / low-battery / geofence / kill behavior
and vetoes offboard setpoints when pose is stale, CfC output is invalid
(or the safety-filter veto is sustained), or the operator has issued a
kill. Read it before you test.

## The test ladder (Part 9.5, never skip a rung)

1. **Unit.** Observation builder, safety filter, failsafe transitions.
   - Automation: full. CI runs these on every push. See
     [tests/README.md](../tests/README.md) for the binary → rung mapping.
2. **SITL.** Full stack vs PX4 SITL / Gazebo; CfC on **unseen** courses;
   kill-link tests.
   - Automation: `sim/quickstart.sh` spins up PX4 SITL + the full onboard
     stack pointed at the sim. Killing the tether session mid-flight must
     not change flight behavior (autonomy continues; the drone does not
     fall). Manual to run; automated asserts in the simulator-side test
     harness are recommended.
3. **HITL.** Real Pi + real FC, **props removed**, sim world. The Pi is
     the real hardware; the FC is the real hardware; the world is still
     Gazebo/SITL. You are catching Pi-specific timing, USB jitter, FC
     protocol quirks, and the real MAVLink round-trip you cannot get on
     the bench.
4. **Tethered low hover.** Open area, **< 2 m** altitude, tether attached,
   **kill-switch in hand** (operator finger on the hardware switch).
   First time real props spin. No autonomy. You are validating the
   mechanical stack, the FC tuning, and the bring-up order.
5. **Open-area autonomy.** Progressively larger goals; log everything.
   CfC drives; the safety filter vetoes rarely; the deterministic
   fallback planner is the cut-over when the CfC goes NaN or the
   safety-filter keeps firing.
6. **Backhaul / fleet.** Add Starlink + a second drone only after
   single-drone is boring. The gated step. See [FLEET.md](FLEET.md).

The order is a chain — each rung earns the trust that lets you run the
next one. **Do not run rung 5 before rung 4. Do not run rung 6 before
single-drone rung 5 is boring.**

## Written pre-flight card template

Per manual Part 9.4 ("insurance + a written flight-test card"). Print
this, fill it in for the airframe, sign it before the props spin. No
card, no flight.

```
DRONECTL PRE-FLIGHT CARD — airframe: ____________  date: ________
PILOT-IN-COMMAND: ______________________  OBSERVER: ____________________
LOCATION: ______________________  CEILING (m): __________  WX: __________

1. AIRFRAME
   [ ] Battery > config.low_batt_pct (____ %)
   [ ] Motors / props / arms free, no play, no damage
   [ ] Tether payout clear, no snags, spool rotates freely
   [ ] Center of gravity within manufacturer spec
   [ ] Tether weight on this airframe: __________ (confirm < margin)

2. COMPUTING (Pi + camera + converter)
   [ ] Pi boots, dronectl-pilot.service ACTIVE (systemctl status)
   [ ] `dronectl status` reports:
         camera: OK    fc-link: OK (pose < 100 ms)
         cfc:    loaded                tether: UP (10.8.0.1)
         backhaul: UP or DOWN (intended)      failsafe: IDLE
   [ ] `dronectl preflight` passes (pose fresh, grid populating, CfC
        loaded, geofence set, battery OK)

3. FLIGHT CONTROLLER
   [ ] Pixhawk boot OK, GPS fix: ____ sats, HDOP: ____
   [ ] FC firmware matches the tested build (PX4 ____ / ArduPilot ____)
   [ ] RTL set, return altitude set above any planned obstacle
   [ ] Offboard mode available (verify via QGC over tether)

4. TETHER (see TETHER.md)
   [ ] Fiber end clean, converter link LED solid on both ends
   [ ] Heartbeat visible at the ground box (200 ms cadence)
   [ ] IP up: drone __10.8.0.__ reachable from ground box
   [ ] Kill command round-trips: operator -> drone -> ground logs it

5. BACKHAUL / FLEET (rung 6 only)
   [ ] Starlink up; WireGuard (`wg0`) handshakes
   [ ] Coordinator per-drone process up for drone __
   [ ] Validator (geofence + separation) rejects synthetic out-of-fence
        goal in a dry-run before the live goal

6. FAILSAFE (read control/failsafe.hpp before signing)
   [ ] link_loss_action configured: __________
   [ ] Kill switch in hand (RUNC 6), tested by clicking it on the bench
   [ ] Geofence + altitude band entered into dronectl config
   [ ] Determined the cut-over: CfC NaN -> planner -> HOLD happens at
        what sustained-veto count: ____

7. RANGE / NOTAMS / AUTH
   [ ] LAANC authorization for this grid obtained
   [ ] Remote ID broadcast active (check on a second receiver)
   [ ] Tethered-UAS provision confirmed applicable or waiver in hand
   [ ] BVLOS waiver in hand (if any goal is not VLOS)

PILOT-IN-COMMAND SIGNATURE: ______________________
TIME: ____________________
```

Anything unchecked on this card means you do not have permission to spin
the props. The failsafe machine (`control/failsafe.hpp`) refuses
offboard setpoints when its preconditions are not met. The card makes
the *operator's* preconditions just as enforceable.

## Reference

- Manual Part 9.5 — the test ladder verbatim.
- `control/failsafe.hpp` — the state machine; consult everywhere.
- [REGULATORY.md](REGULATORY.md) — the waivers / certs that gate the
  later rungs.