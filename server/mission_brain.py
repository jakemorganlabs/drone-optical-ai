"""Phase 5 (manual Part 6.3) — self-hosted LLM mission brain.

Pluggable LLM backend with a tiny ``LLMClient`` interface: a single
``chat(messages) -> response`` method. Ships two implementations:

  * ``OllamaClient`` (default): POSTs to http://localhost:11434/api/chat
    (the standard local-LLM endpoint) using only the stdlib ``urllib``.
  * ``StubClient``   : no network. Emits a canned goal list from a fixed
    prompt template so the server and tests run with zero extras.

HARD CONTRACT  (manual Part 6.3):
    The LLM NEVER emits velocities or motor commands. It emits waypoints
    and constraints only:
        [{"drone_id": str, "goal": {"x","y","z"}, "geofence": {...}}, ...]
    The onboard CfC (Part 4) turns goals into flight. The deterministic
    validator (validator.py) is called between this output and any
    coordinator push, so even a misbehaving LLM cannot fly a drone into a
    fence / into another drone / into the ground.

    Defends-in-depth: this module *also* asserts the contract on its own
    parsed output and strips any forbidden fields (velocity, motor, thrust)
    before handing the list to the validator.
"""

from __future__ import annotations

import importlib.util
import json
import urllib.request
import urllib.error
from typing import Any, List, Mapping, Protocol, Sequence


# ---------------------------------------------------------------------------
# Hard contract
# ---------------------------------------------------------------------------
# hard contract: the LLM survives waypoints/constraints only. Never velocities,
# never motor commands, never thrust / pwm / attitude-rate setpoints.
FORBIDDEN_GOAL_KEYS = {
    "velocity", "vel", "vx", "vy", "vz",
    "motor", "motors", "pwm", "thrust",
    "rate", "rates", "attitude", "angular",
}

# System prompt that pins the contract. Sent as the first message in every
# chat call.
SYSTEM_PROMPT = (
    "You are the mission brain for a fleet of tethered drones. "
    "Given a free-text operator mission, decompose it into a JSON array of "
    "goals. Each goal is a waypoint: "
    '{"drone_id": string, "goal": {"x": float, "y": float, "z": float}, '
    '"geofence": optional {"polygon": [[x,y],...], "floor_m": float, '
    '"ceil_m": float}}. '
    "Coordinates are meters in the local ENU frame (x=north, y=east, z=up, "
    "z=0 is the takeoff origin). "
    "HARD CONTRACT: emit waypoints and geofence constraints ONLY. NEVER emit "
    "velocities, motor commands, thrust, PWM, attitude rates, or any control "
    "surface command. The onboard CfC pilot turns waypoints into flight; the "
    "deterministic validator rejects anything outside the geofence / altitude "
    "band / separation rules. Reply with a single JSON array and nothing else."
)


class LLMClient(Protocol):
    """Minimal pluggable LLM interface."""

    def chat(self, messages: Sequence[Mapping[str, str]]) -> str:  # pragma: no cover - iface
        ...


# ---------------------------------------------------------------------------
# OllamaClient (default) — local self-hosted LLM via /api/chat
# ---------------------------------------------------------------------------
class OllamaClient:
    def __init__(self, base_url: str = "http://localhost:11434",
                 model: str = "llama3:8b", timeout_s: float = 30.0):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout_s = timeout_s

    def chat(self, messages: Sequence[Mapping[str, str]]) -> str:
        payload = json.dumps({
            "model": self.model,
            "messages": list(messages),
            "stream": False,
            "format": "json",  # nudge Ollama to return parseable JSON
        }).encode("utf-8")
        url = self.base_url + "/api/chat"
        req = urllib.request.Request(
            url, data=payload, method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
                body = r.read().decode("utf-8")
        except urllib.error.URLError as e:
            # Surface a structured error the server can degrade on.
            raise RuntimeError("ollama unreachable: %s" % e) from e
        try:
            parsed = json.loads(body)
            # Ollama /api/chat returns {"message": {"content": "..."}}
            msg = parsed.get("message", {})
            content = msg.get("content", "")
        except json.JSONDecodeError as e:
            raise RuntimeError("ollama returned non-JSON: %s" % e) from e
        return content


# ---------------------------------------------------------------------------
# StubClient — no network, canned goals from a fixed template
# ---------------------------------------------------------------------------
class StubClient:
    """Deterministic canned LLM for tests + offline dev.

    Recognizes a handful of keyword cues in the prompt ("north", "south",
    "east", "west", "search", "inspect") and emits a matching two-drone
    search pattern inside a 400x300 box. Falls back to a small default
    ladder otherwise.
    """

    def __init__(self):
        self.last_messages: List[Mapping[str, str]] = []

    def chat(self, messages: Sequence[Mapping[str, str]]) -> str:
        self.last_messages = list(messages)
        user_text = ""
        for m in reversed(messages):
            if m.get("role") == "user":
                user_text = m.get("content", "")
                break
        return StubClient.canned_plan(user_text)

    @staticmethod
    def canned_plan(prompt: str) -> str:
        p = prompt.lower()
        # naively pick a corner / pattern from keywords
        if "south" in p:
            waypoints = [(20, 20, 25), (60, 20, 25), (100, 20, 25)]
        elif "east" in p:
            waypoints = [(350, 50, 25), (350, 150, 25), (350, 250, 25)]
        elif "west" in p:
            waypoints = [(10, 50, 25), (10, 150, 25), (10, 250, 25)]
        elif "tree" in p or "avoid" in p:
            # explicit avoid: stay in open north field, low altitude
            waypoints = [(50, 250, 15), (150, 250, 15), (250, 250, 15)]
        elif "inspect" in p:
            waypoints = [(100, 100, 10), (100, 100, 30), (100, 100, 50)]
        else:
            # default: north field ladder search
            waypoints = [(50, 250, 30), (200, 250, 30), (200, 50, 30), (50, 50, 30)]

        # two drones, alternating through the ladder
        drones = ["drone-01", "drone-02"]
        plan: List[dict] = []
        for i, (x, y, z) in enumerate(waypoints):
            plan.append({
                "drone_id": drones[i % len(drones)],
                "goal": {"x": float(x), "y": float(y), "z": float(z)},
            })
        return json.dumps(plan)


# ---------------------------------------------------------------------------
# Output parsing + hard-contract enforcement
# ---------------------------------------------------------------------------
def _strip_json_fence(s: str) -> str:
    s = s.strip()
    if s.startswith("```"):
        # strip a leading ```json or ``` fence
        first_nl = s.find("\n")
        if first_nl != -1:
            s = s[first_nl + 1:]
        if s.endswith("```"):
            s = s[:-3]
    return s.strip()


def parse_llm_plan(raw: str) -> List[dict]:
    """Parse + sanitize the LLM's raw string into a goal list.

    Enforces the hard contract in depth (see module docstring):
      * result must be a JSON array of objects,
      * each must have drone_id + goal{x,y,z},
      * any forbidden keys (velocity/motor/thrust/...) are stripped and
        an AssertionError is raised so the server never forwards them.
    """
    # hard contract: assert velocity/motor never appear in the raw output
    _raw_lower = raw.lower()
    assert not any(k in _raw_lower for k in ('"velocity"', '"motor"', '"pwm"',
                                            '"thrust"', '"vy"', '"vx"', '"vz"')), \
        "hard contract violation: LLM output referenced forbidden control keys"

    body = _strip_json_fence(raw)
    try:
        parsed = json.loads(body)
    except json.JSONDecodeError as e:
        raise RuntimeError("LLM did not return valid JSON: %s" % e) from e

    if not isinstance(parsed, list):
        # tolerate a single wrapped object: {"goals": [...]}
        if isinstance(parsed, dict) and isinstance(parsed.get("goals"), list):
            parsed = parsed["goals"]
        else:
            raise RuntimeError("LLM output root is not a JSON array")

    clean: List[dict] = []
    for i, item in enumerate(parsed):
        if not isinstance(item, dict):
            raise RuntimeError("goal #%d is not an object" % i)
        drone_id = item.get("drone_id")
        if not isinstance(drone_id, str) or not drone_id:
            raise RuntimeError("goal #%d missing/invalid drone_id" % i)
        goal = item.get("goal")
        if not isinstance(goal, dict):
            raise RuntimeError("goal #%d missing 'goal' object" % i)
        try:
            x = float(goal.get("x")); y = float(goal.get("y")); z = float(goal.get("z"))
        except (TypeError, ValueError) as e:
            raise RuntimeError("goal #%d goal.x/y/z must be numeric" % i) from e
        out = {"drone_id": drone_id, "goal": {"x": x, "y": y, "z": z}}
        # optional per-goal geofence (keep polygon + band only)
        gf = item.get("geofence")
        if isinstance(gf, dict) and gf:
            clean_gf = {}
            if "polygon" in gf and isinstance(gf["polygon"], list):
                clean_gf["polygon"] = gf["polygon"]
            if "floor_m" in gf: clean_gf["floor_m"] = gf["floor_m"]
            if "ceil_m" in gf:  clean_gf["ceil_m"]  = gf["ceil_m"]
            if clean_gf:
                out["geofence"] = clean_gf
        # assert + strip any forbidden keys at this layer too
        leaked = [k for k in item.keys() if k.lower() in FORBIDDEN_GOAL_KEYS]
        assert not leaked, "hard contract violation: goal #%d has keys %r" % (i, leaked)
        clean.append(out)
    return clean


def decompose(client: LLMClient, mission_text: str, n_drones: int = 2) -> List[dict]:
    """End-to-end: take free-text mission, return sanitized goal list.

    The caller (server/main.py) runs the validator on the result before
    pushing anything to a coordinator.
    """
    sys_msg = {"role": "system", "content": SYSTEM_PROMPT}
    user_msg = {
        "role": "user",
        "content": (
            "Mission: %s\n"
            "Available drones: %d (drone-01 .. drone-%02d).\n"
            "Reply with a single JSON array of goals, nothing else."
        ) % (mission_text, n_drones, n_drones),
    }
    raw = client.chat([sys_msg, user_msg])
    return parse_llm_plan(raw)


def make_default_client() -> LLMClient:
    """Pick a backend at startup. Ollama if reachable, else StubClient.

    The server imports cleanly with stdlib only; Ollama is probed lazily
    via a short urlopen. If the probe fails we fall back to the stub
    (good for laptops/CI where no local LLM runs).
    """
    # allow force-selection via env
    import os
    sel = os.environ.get("DRONE_LLM", "").lower()
    if sel == "stub":
        return StubClient()
    if sel == "ollama":
        return OllamaClient()
    # auto: probe ollama with a short timeout (fail fast if not running)
    try:
        OllamaClient(timeout_s=1.5).chat([{"role": "system", "content": "ping"}])
        return OllamaClient()
    except Exception:
        return StubClient()


if __name__ == "__main__":
    # demo: stub decompose + print
    c = StubClient()
    plan = decompose(c, "search the north field, avoid the tree line", n_drones=2)
    print(json.dumps(plan, indent=2))