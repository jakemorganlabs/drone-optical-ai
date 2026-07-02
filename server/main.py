"""Phase 5 (manual Part 6) — mission-brain HTTP server.

Zero external dependency by default: it uses stdlib ``http.server``. If
``fastapi`` is importable we serve via FastAPI/uvicorn; else if ``flask``
is importable we serve via Flask; else plain ``wsgiref``-style
``http.server``. The framework is selected lazily inside guarded
``importlib.util.find_spec`` blocks so the module always imports cleanly
with stdlib only.

Endpoints (identical regardless of backend):

  POST /mission   body: {"prompt": "...", "n_drones": int}
       -> LLM decompose -> validator -> push to coordinators
       -> {"proposal": [...], "accepted": [...], "rejected": [...],
           "pushed": bool, "degraded": bool}
       Even if the WAN/Starlink link to the coordinators is down, the
       operator still gets the validated LLM proposal back (manual Part
       6.4 graceful degradation: drones keep flying on onboard CfC).

  GET  /fleet     -> proxies a fleet view from the coordinators. Each
       coordinator runs its own /fleet scrape on 127.0.0.1:8090 by
       default; in a single-host bench we read the shared state file
       (server/.fleet.json) that each coordinator appends to, falling
       back to a per-drone HTTP fan-out if env DRONE_FLEET_URLS is set.

  GET  /healthz   -> {"status":"ok"}
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
import urllib.request
import urllib.error
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Mapping, Tuple

# Make `python3 server/main.py` and `python3 -m server.main` both work.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mission_brain  # noqa: E402
import validator  # noqa: E402


# ---------------------------------------------------------------------------
# Coordinator push (graceful-degrade path)
# ---------------------------------------------------------------------------
# Coordinators are wired to the server over the WG tunnel (manual Part 6.2).
# In dev the server and coordinators run on the same host; the server pushes
# each validated goal via the coordinator's HTTP API (sent as JSON-line frames
# on the tether). We keep this tiny: the server POSTs to
# http://<coord_host>:<coord_port>/command with a single GroundCommand frame.
#
# If the push fails (Starlink down, coordinator unreachable), the /mission
# response still includes the LLM proposal. The drones keep flying their
# last goal on onboard CfC; the operator still has fiber teleop (Part 6.4).

COORDINATORS_ENV = "DRONE_COORDINATORS"   # "drone-01=127.0.0.1:8090,drone-02=..."
FLEET_URLS_ENV   = "DRONE_FLEET_URLS"
FLEET_STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".fleet.json")


def _coordinator_table() -> Dict[str, Tuple[str, int]]:
    """Return {drone_id: (host, port)} from DRONE_COORDINATORS env."""
    out: Dict[str, Tuple[str, int]] = {}
    spec = os.environ.get(COORDINATORS_ENV, "")
    if not spec:
        out["drone-01"] = ("127.0.0.1", 8090)
        return out
    for part in spec.split(","):
        part = part.strip()
        if not part or "=" not in part:
            continue
        did, hp = part.split("=", 1)
        if ":" not in hp:
            continue
        h, p = hp.rsplit(":", 1)
        try:
            out[did.strip()] = (h.strip(), int(p))
        except ValueError:
            continue
    if not out:
        out["drone-01"] = ("127.0.0.1", 8090)
    return out


def push_goal_to_coordinator(drone_id: str, goal: Mapping[str, Any],
                             timeout_s: float = 2.0) -> Tuple[bool, str]:
    """Push one validated goal to a coordinator's HTTP command endpoint.

    The coordinator doesn't expose a write HTTP endpoint yet in this
    scaffold (it has its own /fleet scrape + the tether-worker downlink).
    For the Phase 5 deliverable we write the goal into the shared fleet
    state file keyed by drone_id; the coordinator's downlink path picks it
    up // TODO(phase7): replace with a real POST /command once the
    coordinator grows an HTTP write API.
    """
    # Provisional implementation: append to the shared state file under a
    # "pending_commands" key. The coordinator reads this file each tick and
    # forwards the goal down the tether.
    lock_path = FLEET_STATE_FILE + ".lock"
    state = _read_fleet_state()
    cmds = state.setdefault("pending_commands", {})
    cmds[drone_id] = {
        "kind": "goal",
        "goal": goal,
        "ts": time.time(),
    }
    ok = _write_fleet_state(state)
    return (ok, "queued" if ok else "write-failed")


def _read_fleet_state() -> Dict[str, Any]:
    try:
        with open(FLEET_STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}


def _write_fleet_state(state: Mapping[str, Any]) -> bool:
    tmp = FLEET_STATE_FILE + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(state, f)
        os.replace(tmp, FLEET_STATE_FILE)
        return True
    except OSError:
        return False


def _fleet_urls() -> List[str]:
    env = os.environ.get(FLEET_URLS_ENV, "")
    if env:
        return [u.strip() for u in env.split(",") if u.strip()]
    # default: one coordinator on loopback
    return ["http://127.0.0.1:8090/fleet"]


def fetch_fleet() -> Dict[str, Any]:
    """Aggregate the fleet view: each coordinator's /fleet scrape.

    Fallback policy:
      1. If DRONE_FLEET_URLS is set, fan out a GET to each URL and merge.
      2. Else, read the shared FLEET_STATE_FILE (the coordinators and this
         server share it on the ground box; in dev the .fleet.json file is
         the rendezvous).
    """
    drones: List[Dict[str, Any]] = {}
    urls = _fleet_urls()
    merged = []
    for u in urls:
        try:
            with urllib.request.urlopen(u + "/fleet" if not u.endswith("/fleet") else u,
                                       timeout=1.5) as r:
                body = r.read().decode("utf-8")
            obj = json.loads(body)
            merged.append(obj)
        except Exception:
            continue
    if not merged:
        # fall back to the shared state file
        st = _read_fleet_state()
        return {"drones": st.get("drones", []), "source": "statefile"}
    return {"drones": merged, "source": "http"}


# ---------------------------------------------------------------------------
# Mission handler
# ---------------------------------------------------------------------------
def handle_mission(body: Mapping[str, Any]) -> Tuple[int, Dict[str, Any]]:
    prompt = body.get("prompt", "").strip()
    if not prompt:
        return 400, {"error": "missing 'prompt'"}
    try:
        n_drones = int(body.get("n_drones", 2))
    except (TypeError, ValueError):
        n_drones = 2
    n_drones = max(1, min(n_drones, 32))

    client = mission_brain.make_default_client()
    try:
        proposal = mission_brain.decompose(client, prompt, n_drones=n_drones)
    except RuntimeError as e:
        # LLM did not return parseable JSON — degrade gracefully: return
        # the error + an empty proposal so the operator can retry.
        return 502, {"error": "llm", "message": str(e),
                    "proposal": [], "accepted": [], "rejected": [],
                    "pushed": False, "degraded": True}

    # Deterministic validator (manual Part 6.3).
    cfg = validator.load_config()
    result = validator.validate_goals(proposal, cfg=cfg)

    # Push accepted goals to coordinators (graceful-degrade path).
    pushed_ok = True
    push_errors: List[str] = []
    for g in result["accepted"]:
        did = g["drone_id"]
        ok, msg = push_goal_to_coordinator(did, g["goal"])
        if not ok:
            pushed_ok = False
            push_errors.append("%s: %s" % (did, msg))

    degraded = (not pushed_ok) or bool(result["rejected"])
    return 200, {
        "proposal": proposal,
        "accepted": result["accepted"],
        "rejected": result["rejected"],
        "reasons":  result["reasons"],
        "pushed":   pushed_ok,
        "push_errors": push_errors,
        "degraded": degraded,
        # hard contract echo so the operator can confirm the server itself
        # believes the LLM proposed waypoints only:
        "contract": "waypoints-only; cfC on drone turns goals into flight",
    }


# ---------------------------------------------------------------------------
# stdlib http.server backend (default; zero deps)
# ---------------------------------------------------------------------------
class _Handler(BaseHTTPRequestHandler):
    server_version = "dronectl-mission-brain/0.1"

    def _send(self, code: int, obj: Mapping[str, Any]):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            self.wfile.write(body)
        except BrokenPipeError:
            pass

    def _read_body(self) -> Mapping[str, Any]:
        ln = int(self.headers.get("Content-Length", "0") or "0")
        if ln <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(ln).decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {"__bad__": True}

    def do_GET(self):
        if self.path == "/healthz":
            self._send(200, {"status": "ok"})
        elif self.path == "/fleet":
            self._send(200, fetch_fleet())
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/mission":
            self._send(404, {"error": "not found"})
            return
        body = self._read_body()
        if body.get("__bad__"):
            self._send(400, {"error": "invalid json body"})
            return
        code, resp = handle_mission(body)
        self._send(code, resp)

    def log_message(self, fmt, *args):
        # quiet, but keep errors visible
        sys.stderr.write("[mission-brain] %s - %s\n" % (self.address_string(), fmt % args))


def _run_stdlib(host: str = "0.0.0.0", port: int = 8080):
    srv = ThreadingHTTPServer((host, port), _Handler)
    print("mission-brain (stdlib http.server) on http://%s:%d" % (host, port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


# ---------------------------------------------------------------------------
# FastAPI / Flask backends (optional, lazy)
# ---------------------------------------------------------------------------
def _run_fastapi(host: str = "0.0.0.0", port: int = 8080):
    from fastapi import FastAPI, Request  # type: ignore
    from fastapi.responses import JSONResponse  # type: ignore
    import uvicorn  # type: ignore

    app = FastAPI(title="dronectl mission brain")

    @app.get("/healthz")
    async def healthz():
        return {"status": "ok"}

    @app.get("/fleet")
    async def fleet():
        return JSONResponse(fetch_fleet())

    @app.post("/mission")
    async def mission(req: Request):
        try:
            body = await req.json()
        except Exception:
            return JSONResponse({"error": "invalid json body"}, status_code=400)
        code, resp = handle_mission(body)
        return JSONResponse(resp, status_code=code)

    uvicorn.run(app, host=host, port=port, log_level="info")


def _run_flask(host: str = "0.0.0.0", port: int = 8080):
    from flask import Flask, request, jsonify  # type: ignore

    app = Flask(__name__)

    @app.get("/healthz")
    def healthz():
        return jsonify({"status": "ok"})

    @app.get("/fleet")
    def fleet():
        return jsonify(fetch_fleet())

    @app.post("/mission")
    def mission():
        body = request.get_json(silent=True) or {}
        code, resp = handle_mission(body)
        return jsonify(resp), code

    app.run(host=host, port=port)


def run(host: str = "0.0.0.0", port: int = 8080):
    """Pick a backend. FastAPI > Flask > stdlib http.server."""
    if importlib.util.find_spec("fastapi") and importlib.util.find_spec("uvicorn"):
        _run_fastapi(host, port)
    elif importlib.util.find_spec("flask"):
        _run_flask(host, port)
    else:
        _run_stdlib(host, port)


if __name__ == "__main__":
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "8080"))
    run(host, port)