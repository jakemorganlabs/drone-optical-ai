# server/ — Phase 5: self-hosted AI mission brain + WireGuard-over-Starlink

Per Option A build manual **Part 6**. The expensive LLM brain, the fleet
HTTP API, and the deterministic validator live on a self-hosted server the
ground box reaches over WireGuard-on-Starlink. **Nothing flight-critical
rides `wg0`** — the fiber tether (Phase 4, `10.8.0.0/30`) carries telemetry
+ teleop + goals in near-real-time, and the WAN behind `wg0` only carries
operator->server mission requests and server->coordinator goal pushes.

## Layout

```
server/
├── main.py              # zero-dep http.server app (FastAPI/Flask optional)
├── mission_brain.py     # pluggable LLM (Ollama default, Stub for tests)
├── validator.py         # deterministic geofence/altitude/separation gate
├── config.yaml          # geofence polygon + altitude band + separation
├── test_validator.py    # stdlib unittest; runnable as a script
├── wireguard.example.conf  # PLACEHOLDER wg0 config (real keys by ops)
└── README.md            # this file
```

## Hard contract (manual Part 6.3)

> The LLM emits goals / waypoints / constraints, **never velocities or
> motor commands**. The onboard CfC (Part 4) turns goals into flight.

This is enforced three ways:

1. **System prompt** (`mission_brain.SYSTEM_PROMPT`) pins the contract in
   every chat call.
2. **Parser** (`mission_brain.parse_llm_plan`) asserts the raw LLM output
   contains none of `velocity / motor / pwm / thrust / vx / vy / vz`;
   on a violation it raises `AssertionError` before the validator runs.
3. **Deterministic validator** (`validator.validate_goals`) is the final
   gate before any coordinator push — it rejects goals outside the
   geofence polygon, below/above the altitude band, or violating
   inter-drone separation, *regardless of what the LLM said*.

Defends-in-depth: the LLM can lie, hallucinate, or be jailbroken; neither
the server nor any drone can move outside the validated envelope as a
result.

## WireGuard-over-Starlink for Tier-3 traffic

```ini
# ground box: /etc/wireguard/wg0.conf   (from wireguard.example.conf)
[Interface]
PrivateKey = <ground_priv>
Address    = 10.9.0.2/24
[Peer]
PublicKey  = <server_pub>
Endpoint   = your.server.example:51820
AllowedIPs = 10.9.0.0/24
PersistentKeepalive = 25
```

- **10.8.0.0/30** — fiber tether LAN (Phase 5.1). Telemetry, teleop, goals.
- **10.9.0.0/24** — WireGuard overlay for Tier-3 (this server). Operator
  HTTP, LLM mission requests, server->coordinator goal pushes.
- **Nothing flight-critical rides `wg0`.** Starlink has variable latency
  and can drop; the tether does not (fiber), and onboard CfC does not
  need either to keep flying the last goal.

### Provisioning WireGuard keys

```bash
# Ground box:
wg genkey | tee ground_box.priv | wg pubkey > ground_box.pub
# Server:
wg genkey | tee server.priv    | wg pubkey > server.pub
# Exchange the *.pub files out of band (they are not secret).
# Paste into wireguard.example.conf / the server's mirror block, then:
sudo cp wireguard.example.conf /etc/wireguard/wg0.conf
sudo chmod 600 /etc/wireguard/wg0.conf
sudo wg-quick up wg0
ping 10.9.0.1     # sanity
```

## Running

```bash
# Zero-dep default (stdlib http.server):
python3 server/main.py                # listens on 0.0.0.0:8080

# Or with FastAPI/uvicorn if installed:
pip install fastapi uvicorn
python3 server/main.py

# Endpoints:
curl localhost:8080/healthz
curl -X POST localhost:8080/mission \
     -H 'content-type: application/json' \
     -d '{"prompt":"search the north field, avoid the tree line","n_drones":2}'
curl localhost:8080/fleet
```

`DRONE_LLM=stub python3 server/main.py` forces the no-network `StubClient`
(good for dev / CI). `DRONE_LLM=ollama` forces the local Ollama client.

`DRONE_COORDINATORS=drone-01=127.0.0.1:8090,drone-02=127.0.0.1:8091`
tells the server where each per-drone coordinator lives.

## Graceful degradation (design + test)

Manual **Part 6.4**:

- **Starlink up:**   full fleet reasoning + dynamic re-tasking.
- **Starlink down:** each drone continues its **last goal** on onboard
  CfC; the ground operator still has **full fiber teleop**. No freeze,
  no fall.

Implementation: `server/main.py:POST /mission` always returns the LLM
proposal + validator verdict to the operator, even if the
server->coordinator push fails (Starlink drop). The `pushed` /
`degraded` fields in the response tell the operator which path the
fleet is on.

### Degraded-mode test (kill wg0 mid-mission)

```bash
# 1. Bring up the full stack (single-drone bench):
python3 server/main.py &                # WAN-side mission brain
make coordinator && ./build/coordinator --tether-ip 10.8.0.2 &  # ground box
# (drone-side: pilot_main + tether-agent per Phase 4, on the Pi or SITL)

# 2. Submit a real mission over wg0:
curl -X POST 10.9.0.1:8080/mission -d '{"prompt":"search north field"}'
#    -> drones accept goals + start flying on CfC.

# 3. KILL WG0 mid-mission (simulate Starlink drop):
sudo wg-quick down wg0

# 4. Submit another /mission: the server still returns the LLM proposal
#    with "degraded": true — drones DO NOT stop; they keep flying the
#    last validated goal on onboard CfC. Operator teleop over the fiber
#    (10.8.0.x) is unaffected.

# 5.wg-quick up wg0 -> re-tasking resumes; reconnect resyncs goals
#    via the tether-agent's idempotent goal-id dedup (Phase 4 5.3).
sudo wg-quick up wg0
```

## Tests

```bash
python3 server/test_validator.py
# Validates: geofence out-of-bounds reject, altitude-band reject,
# separation reject, and a valid-goal accept. Uses stdlib unittest.
```