# Wiring — flight controller ↔ companion computer

The Raspberry Pi is the **companion computer**; the Pixhawk-class FC runs PX4 or
ArduPilot and owns the motors. The Pi talks MAVLink to the FC and never drives
the ESCs directly.

## Physical link

| Connection | Notes |
| --- | --- |
| Pixhawk **TELEM2** (or a spare UART) → Pi **GPIO 14/15** (UART0) | 3.3 V logic, common ground. Cheap + low-jitter. |
| Pixhawk USB → Pi USB | Easier bring-up if you have no free UART; higher latency floor. |

Use TELEM2 for the flight-critical offboard link; reserve USB for bench bring-up.

## Pixhawk / PX4 params

```
MAV_1_CONFIG   = TELEM2
MAV_1_MODE     = Onboard        # companion computer
MAV_1_RATE     = 921600         # or SER_BAUD_921600 depending on PX4 version
```

## ArduPilot params

```
SERIAL2_PROTOCOL = 2            # MAVLink 2
SERIAL2_BAUD     = 921         # 921600 baud
```

## Baud

**921600** is recommended for offboard setpoints; lower bauds starve the
20 Hz velocity stream and the bridge will fall back to HOLD on stale pose
(see `control/fc_bridge/fc_bridge.cpp`, 100 ms staleness gate).

## Onboard multiplex

`mavlink-router` (config in `control/fc_bridge/mavlink-router.conf`, installed to
`/etc/mavlink-router/main.conf`) exposes the single FC link to multiple consumers:

- `127.0.0.1:14540` UDP — the onboard MAVSDK bridge (this project) connects here.
- `5760` TCP — QGroundControl / tethered GCS.
- Optional MAVLink logger.

The bridge is the **only** offboard command sink; other endpoints are read-only
or pass operator kill/hover through the failsafe machine (Part 7).