"""Phase 5 (manual Part 6.3) — deterministic mission validator.

Sits between the LLM mission brain (server/mission_brain.py) and the
ground coordinators (ground/coordinator). It is a *deterministic* gate:
regardless of what the LLM proposed, a goal that falls outside the
geofence polygon / altitude band / inter-drone separation rules is
rejected with a structured error so the server can re-prompt the LLM
with the specific violation.

Config is loaded from server/config.yaml (a tiny hand-written subset of
YAML is parsed here so the server has zero non-stdlib deps; if PyYAML is
importable we use it for full fidelity). See config.yaml for the schema.

Contract:
    goal = {"drone_id": str, "goal": {"x": float, "y": float, "z": float},
            "geofence": {...}  # optional per-goal override, usually omitted}

    ValidationResult = {
        "ok": bool,
        "accepted": [goal, ...],          # only if ok
        "rejected": [{"goal":..., "reasons":[str,...]}, ...],  # only if not ok
        "reasons": [str,...],             # flat list (for re-prompt)
    }
"""

from __future__ import annotations

import json
import math
import os
from typing import Iterable, List, Mapping, Sequence, Tuple

CONFIG_PATH_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")


# ---------------------------------------------------------------------------
# Tiny YAML parser (stdlib only)
# ---------------------------------------------------------------------------
def _parse_tiny_yaml(text: str) -> dict:
    """Parse the narrow subset of YAML used by config.yaml.

    Supports:
      - top-level ``key:`` mappings with nested indented mappings
      - ``key: value`` scalars (numbers parsed heuristically, else strings)
      - block lists of two-element arrays under ``- [a, b]``
    No anchors, no flow mappings except a single inline ``[a, b]`` line.
    PyYAML is used if available for full fidelity.
    """
    try:
        import yaml  # type: ignore
        loaded = yaml.safe_load(text)
        if isinstance(loaded, dict):
            return loaded
    except Exception:
        pass

    root: dict = {}
    stack: List[Tuple[int, dict]] = [(-1, root)]

    def coerce(s: str):
        s = s.strip()
        if s == "":
            return ""
        if (s.startswith('"') and s.endswith('"')) or (s.startswith("'") and s.endswith("'")):
            return s[1:-1]
        # try int then float
        try:
            return int(s)
        except ValueError:
            pass
        try:
            return float(s)
        except ValueError:
            pass
        return s

    def parse_inline_list(s: str):
        s = s.strip()
        if s.startswith("[") and s.endswith("]"):
            inner = s[1:-1]
            parts = [p.strip() for p in inner.split(",")] if inner.strip() else []
            return [coerce(p) for p in parts]
        return coerce(s)

    for raw in text.splitlines():
        # strip comments
        if "#" in raw:
            # naive: only strip if # is not inside quotes (config has none)
            raw = raw.split("#", 1)[0]
        if not raw.strip():
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        stripped = raw.strip()

        # pop stack to current indent
        while stack and stack[-1][0] >= indent:
            stack.pop()
        parent = stack[-1][1] if stack else root

        if stripped.startswith("- "):
            # list item: the value after '- ' is either an inline [x,y] or a scalar
            item = parse_inline_list(stripped[2:])
            # find or create a list in parent under the last-seen key
            # In our config, lists live under a known key two levels up.
            # Simpler: track the "active list key" via stack tail.
            lst = parent.get("__list__")
            if lst is None:
                lst = []
                parent["__list__"] = lst
            lst.append(item)
            continue

        if ":" in stripped:
            key, _, val = stripped.partition(":")
            key = key.strip()
            val = val.strip()
            if val == "":
                # new nested mapping
                child: dict = {}
                parent[key] = child
                stack.append((indent, child))
            else:
                if val.startswith("[") and val.endswith("]"):
                    parent[key] = parse_inline_list(val)
                else:
                    parent[key] = coerce(val)

    # flatten: turn {"geofence": {"polygon": {"__list__": [...]}}} into
    # {"geofence": {"polygon": [...]}}
    def flatten(node):
        if isinstance(node, dict):
            out = {}
            for k, v in node.items():
                if k == "__list__" and isinstance(v, list):
                    # the parent should have built a list directly; but our
                    # naive stack uses parent["__list__"] only when the list
                    # key itself was elided. Map it back into 'polygon'.
                    out = v  # the list IS the value
                    break
                out[k] = flatten(v)
            return out
        if isinstance(node, list):
            return [flatten(x) for x in node]
        return node

    # The tiny parser above places a list under parent["__list__"] when it
    # sees the block sequence. We want that list to be the value of the
    # *containing* key (e.g. geofence.polygon). Re-parent.
    def reparent(node):
        if isinstance(node, dict):
            for k, v in list(node.items()):
                if isinstance(v, dict) and "__list__" in v and len(v) == 1:
                    node[k] = v["__list__"]
                else:
                    reparent(v)
        return node

    reparent(root)
    return root


# ---------------------------------------------------------------------------
# Geofence geometry
# ---------------------------------------------------------------------------
def _point_in_polygon(px: float, py: float, poly: Sequence[Sequence[float]]) -> bool:
    """Ray-casting point-in-polygon. ``poly`` is a list of [x,y] vertices."""
    n = len(poly)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = float(poly[i][0]), float(poly[i][1])
        xj, yj = float(poly[j][0]), float(poly[j][1])
        # is the edge crossing the horizontal ray at py?
        cond = ((yi > py) != (yj > py))
        if cond:
            x_int = (xj - xi) * (py - yi) / (yj - yi) + xi
            if px < x_int:
                inside = not inside
        j = i
    return inside


def _dist3(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


# ---------------------------------------------------------------------------
# Hallucination guard (manual Part 6.3 hard contract + Phase 5 task #4).
# The LLM emits goals/waypoints/constraints ONLY — never velocities, motor
# commands, throttle, PWM, attitude rates, or landing_velocity. Any of these
# keys appearing anywhere in a goal (top-level OR nested) is rejected
# wholesale before the geometric checks run. This mirrors the guard already
# enforced in mission_brain.py; keeping it here too means a downstream
# caller that bypasses the brain still cannot push a motor command through
# the validator. Belt and suspenders.
# ---------------------------------------------------------------------------
FORBIDDEN_KEYS = (
    "velocity", "vel", "vx", "vy", "vz",
    "motor", "motors", "pwm",
    "throttle", "thrust",
    "landing_velocity",
    "attitude_rate", "rate_cmd",
)


def _find_forbidden_keys(obj, prefix: str = "") -> List[str]:
    """Recursively walk `obj` (parsed JSON-ish) and return the list of
    forbidden keys found (with their dotted path for the reason string).
    """
    found: List[str] = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            if isinstance(k, str):
                kl = k.lower()
                if kl in FORBIDDEN_KEYS:
                    found.append(f"{prefix}{k}")
                # also catch compound keys like "target_velocity"
                for f in FORBIDDEN_KEYS:
                    if kl != f and f in kl and f not in ("vel",):  # 'vel' too noisy as substring
                        found.append(f"{prefix}{k}")
            path = f"{prefix}{k}." if isinstance(k, str) else f"{prefix}."
            found.extend(_find_forbidden_keys(v, path))
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            found.extend(_find_forbidden_keys(v, f"{prefix}[{i}]."))
    return found


# ---------------------------------------------------------------------------
# Config load + validation
# ---------------------------------------------------------------------------
def load_config(path: str = CONFIG_PATH_DEFAULT) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    cfg = _parse_tiny_yaml(text)
    # normalize
    poly = cfg.get("geofence", {}).get("polygon", [])
    if not poly:
        raise ValueError(f"config {path}: geofence.polygon missing or empty")
    return cfg


def _altitude_band(cfg: Mapping) -> Tuple[float, float]:
    ab = cfg.get("altitude_band", {})
    return float(ab.get("floor_m", 0.0)), float(ab.get("ceil_m", 120.0))


def _separation(cfg: Mapping) -> float:
    return float(cfg.get("separation", {}).get("min_distance_m", 5.0))


def _polygon(cfg: Mapping) -> List[List[float]]:
    return [[float(p[0]), float(p[1])] for p in cfg["geofence"]["polygon"]]


def validate_goals(goals: Iterable[Mapping],
                   cfg: Mapping | None = None,
                   other_targets: Mapping[str, Sequence[float]] | None = None) -> dict:
    """Validate one batch of LLM-proposed goals.

    ``goals``     : iterable of {drone_id, goal:{x,y,z}, geofence?:...}
    ``cfg``       : parsed config dict; if None, load_config() default path
    ``other_targets``: {drone_id: [x,y,z]} of already-assigned targets to
                    check separation against (typically the drones' current
                    positions, or previously-accepted goals in this batch).

    Returns a structured result dict (see module docstring).
    """
    if cfg is None:
        cfg = load_config()
    poly = _polygon(cfg)
    floor, ceil = _altitude_band(cfg)
    min_sep = _separation(cfg)

    accepted: List[Mapping] = []
    rejected: List[dict] = []
    reasons_flat: List[str] = []

    # Track accepted targets as we go so separation is checked within the batch too.
    assigned: dict = dict(other_targets or {})

    for g in goals:
        drone_id = g.get("drone_id", "?")
        goal = g.get("goal", {})

        # Hallucination guard (manual Part 6.3 hard contract + Phase 5 task #4):
        # any forbidden control key anywhere in this goal is rejected before
        # geometry. The LLM may emit goals/waypoints/constraints only.
        forbidden = _find_forbidden_keys(g)
        if forbidden:
            msg = "hallucination: forbidden control keys %r (hard contract: goals only, never velocity/motor/throttle/pwm/landing_velocity)" % forbidden
            rejected.append({"goal": g, "reasons": [msg]})
            reasons_flat.append(f"{drone_id}: {msg}")
            continue

        try:
            x = float(goal.get("x", 0.0))
            y = float(goal.get("y", 0.0))
            z = float(goal.get("z", 0.0))
        except (TypeError, ValueError):
            rejected.append({"goal": g, "reasons": ["goal.x/y/z must be numeric"]})
            reasons_flat.append(f"{drone_id}: goal must have numeric x/y/z")
            continue

        reasons: List[str] = []

        # Geofence (allow a per-goal geofence override if present)
        gf = g.get("geofence")
        if isinstance(gf, Mapping) and "polygon" in gf and gf["polygon"]:
            use_poly = [[float(p[0]), float(p[1])] for p in gf["polygon"]]
        else:
            use_poly = poly
        if not _point_in_polygon(x, y, use_poly):
            reasons.append("geofence: goal (%.1f, %.1f) outside polygon" % (x, y))

        # Altitude band
        if z < floor:
            reasons.append("altitude: z=%.1f below floor %.1f" % (z, floor))
        if z > ceil:
            reasons.append("altitude: z=%.1f above ceiling %.1f" % (z, ceil))

        # Inter-drone separation
        for oid, tpos in assigned.items():
            if oid == drone_id:
                continue
            d = _dist3([x, y, z], list(tpos))
            if d < min_sep:
                reasons.append(
                    "separation: drone %s target (%.1f,%.1f,%.1f) is %.2fm away "
                    "(min %.1fm)" % (oid, tpos[0], tpos[1], tpos[2], d, min_sep)
                )

        if reasons:
            rejected.append({"goal": g, "reasons": reasons})
            for r in reasons:
                reasons_flat.append("%s: %s" % (drone_id, r))
        else:
            accepted.append(g)
            assigned[drone_id] = [x, y, z]

    return {
        "ok": not rejected,
        "accepted": accepted,
        "rejected": rejected,
        "reasons": reasons_flat,
    }


# Convenience: validate a single goal (used by tests + re-prompt path).
def validate_one(goal: Mapping, cfg: Mapping | None = None,
                 other_targets: Mapping[str, Sequence[float]] | None = None) -> dict:
    return validate_goals([goal], cfg=cfg, other_targets=other_targets)


if __name__ == "__main__":
    # smoke: load + validate a tiny good + bad batch
    c = load_config()
    sample = [
        {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 20}},
        {"drone_id": "drone-02", "goal": {"x": 1000, "y": 1000, "z": 200}},  # out
    ]
    print(json.dumps(validate_goals(sample, c), indent=2))