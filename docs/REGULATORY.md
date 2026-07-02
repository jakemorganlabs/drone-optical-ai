# Regulatory — US-centric framework

> **Verify for your region.** This page is a US-centric framework, not
> legal advice and not a checklist someone else can apply blindly. The
> rules differ by country, by airspace class, by aircraft weight, by
> tethered-or-not, and by year. The constraints below are **gating, not
> optional**: you do not fly at a rung until the relevant box is
> satisfied for your jurisdiction. Treat this page the way you treat
> the failsafe machine — it lets you fly when its preconditions are
> met, and it refuses the rest of the time.

This is the regulatory framework referenced from [SAFETY.md](SAFETY.md)
and the manual Part 9.4. The tether itself is RF-free — that is the
point of the whole BOM in [HARDWARE.md](HARDWARE.md) — and it changes
which rules apply (and which you no longer need).

## FAA Part 107 — remote pilot certification

Commercial unmanned aircraft operations in the US are governed by
**14 CFR Part 107**. The pilot in command needs a **Remote Pilot
Certificate** (Part 107 cert). Recreational use has its own carve-out
(49 USC 44809) but the Option A build is commercial-grade and you
should plan to Part 107.

- Initial: pass the initial aeronautical knowledge test
  ("sUAS Knowledge Test") at an FAA-authorized testing center, or hold
  Part 61 cert + the online sUAS course.
- Recurrent: keep current (online training course every 24 months).
- Operating rules that this design leans on: VLOS ceiling, max altitude
  400 ft AGL (with structure relief under Part 107.51), daylight or
  anti-collision lighting for civil twilight, NOTAM awareness, careless
  / reckless prohibition (Part 107.23).

## Remote ID

All sUAS in the US National Airspace need **Remote ID** broadcast
(14 CFR Part 89, in force). Three compliance paths:

1. **Standard Remote ID** — the aircraft broadcasts ID, location,
   altitude, velocity, control station, and timestamp in flight. The
   most common path for a kit like this; pick a Remote ID module or an
   FC firmware mode that emits it.
2. **Broadcast Module** — an add-on module broadcasts the required
   fields; aircraft ID is "the module's" rather than the airframe's.
   Useful if your FC firmware does not emit Remote ID natively.
3. **FAA-Recognized Identification Area (FRIA)** — only for aircraft
   flown within a recognized area and only useful in narrow cases. Not
   a general Option-A solution.

Remote ID broadcast must be active before you fly. The pre-flight card
in [SAFETY.md](SAFETY.md) has you verify it on a second receiver.

## Tethered UAS provisions

A tethered UAS has **specific relief and specific rules** in the
US (and in many other jurisdictions) that change how much you can
sometimes fly without a Part 107 + Remote ID stack — but only within
the tethered framework.

- Tethered flight is generally treated as an aircraft bound to a fixed
  ground point by a physical connection. The relief varies; in the US,
  pay close attention to the **tethered balloon / kite / tethered UAS
  definitions** and any state / local interpretation in addition to
  the federal one.
- The tether itself, per this project, is **fiber** — it carries no
  RF and is not a navigable airspace hazard in the RF sense. Confirm
  the physical-tether rules in your jurisdiction apply (including the
  tether's **mechanical** characteristics: tensile strength, max
  length, marker / visibility requirements if any).
- Not every operation you might do under "tethered relief" stays inside
  the tethered framework. Part 107 + Remote ID still apply the moment
  you leave that framework.

**Confirm applicability for your region and your flight before relying
on tethered-UAS relief.** This is one of the biggest "your mileage will
vary" items in this document.

## BVLOS waiver

Beyond Visual Line of Sight flight (BVLOS) is the hard part, and the
manual calls this out explicitly: **it is regulatory, not technical.**
The technical stack in this repo lets you fly a drone out past VLOS
with no problem; the FAA does not let you, by default.

- BVLOS requires a **Part 107 waiver** (49 USC 44807 / 14 CFR Part 107.205
  waiver process) or an equivalent authorization. Be specific in the
  application: the operational area, the observers (or the detect-and-avoid
  solution), the contingency handling, and the lost-link behavior.
- The fiber tether makes a chunk of the DAA ("Detect And Avoid") story
  easier to argue — the drone is on a physical leash, the operator still
  has fiber telemetry / teleop, and the onboard CfC keeps flying when
  the link drops. But it does **not** make BVLOS legal by itself.
- **One-pilot-to-many-aircraft ratios** are tightly constrained. The
  fleet design in [FLEET.md](FLEET.md) is not legal under a default
  Part 107. You need an explicit authorization that covers the ratio
  you actually fly (1:n, with n usually small at first), and any
  fleet test must justify the n observer + DAA architecture for that
  waiver.

This is the gating exception — it lives on top of Part 107, not
instead of it.

## LAANC

**Low Altitude Authorization and Notification Capability** — automated
airspace authorization in controlled airspace around many US airports.

- Apply for a LAANC authorization for the grid you will fly in. The
  ceiling you get is the ceiling you may fly; the_option A pre-flight
  card in [SAFETY.md](SAFETY.md) has you write it down before props spin.
- Some grid is zero-altitude-authorization (apply well ahead); some is
  near-zero or instant-auto-approve. Plan around it; do not collect the
  LAANC ticket on the field.
- LAANC applies to Part 107 ops. It is **not** the tethered-rule; you
  cannot substitute one for the other.

## RF / spectrum compliance

There are three RF sources in Option A, and the tether is explicitly
**not** one of them.

| Source | What it is | Notes |
| --- | --- | --- |
| **Starlink terminal** | Ku/Ka-band user terminal | FCC Part 25 (and the relevant market authorizations). Starlink terminals are licensed terminals for the consumer-internet service; you operate the terminal, not the satellite. Stay on the Starlink / T-Mobile coordination pattern for your region. |
| **Backup RF link(s)** | Whatever you add past the tether (e.g. a 900 MHz / 2.4 GHz telemetry radio, a cellular modem) | FCC Part 15 (unlicensed) or Part 97 (amateur) or licensed. **The tether intentionally removes the need for these.** If you add one, the rule below applies: spectrum + power + EIRP + antenna all matter. |
| **The tether** | Fiber | **RF-free. That's the point.** No Part 15 emission, no frequency coordination, no jammer risk, no interference with anything. |

This is the headline regulatory property of the Option A BOM: the
flight-critical operator link is the tether, and the tether is optical.
The RF rules apply to the Starlink uplink (the terminal vendor holds
that), the backup link if you add one, and the Remote ID broadcast.

## Insurance + the written flight-test card

- **Insurance** is a real cost, not a checkbox. Commercial drone
  liability policies (and TAM/NAM / hull coverage) are usually cheap
  for a single airframe and steep for a fleet — and the fleet policy
  is what the [FLEET.md](FLEET.md) backhaul rung implies. Quote both.
  A written record of the flight-test card is frequently required to
  keep the policy active after an incident.
- The **written flight-test card** is in [SAFETY.md](SAFETY.md). Print
  it, fill it in, sign it before props spin. An unsigned (or
  unchecked) card means you do not have permission to fly that rung.

## "Verify for your region" — gating, not optional

This page is US-centric because most of the implementers are in the
US. For any other region:

- Re-derive every bullet above against your civil aviation authority's
  rules (CAA UK, EASA in the EU/CASA in Australia / Transport Canada /
  JCAB in Japan / etc.). The shape is broadly similar (pilot cert +
  aircraft registration/Remote ID + airspace authorization + BVLOS
  exception + spectrum compliance) but the **specific rules,
  ceilings, and waiver processes differ**.
- Re-derive the tethered-UAS treatment specifically — it is one of the
  most variable items.
- Re-derive the spectrum treatment for Starlink and any backup RF link.
- Re-derive the fleet / one-pilot-to-many-aircraft rules — this is
  the single hardest thing to get an authorization for and it varies
  a lot.

The pre-flight card in [SAFETY.md](SAFETY.md) reads the same in any
region; the gate items on it differ. The constraint is the same: **you
do not fly a rung until the relevant regulatory box is satisfied for
your jurisdiction.**