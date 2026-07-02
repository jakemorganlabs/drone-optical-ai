"""Phase 5 (manual Part 6.3) — validator unit tests (stdlib unittest).

Covers:
  1. geofence out-of-bounds reject
  2. altitude-band reject (below floor and above ceiling)
  3. inter-drone separation reject
  4. valid-goal accept
  5. LLM hallucination guard reject (velocity/motor/throttle/pwm/landing_velocity)

Runnable two ways:
  python3 server/test_validator.py             # auto-discovers main()
  python3 -m unittest server.test_validator    # if you have pytest/unittest CLI

Uses ONLY the stdlib (unittest + the server package). No pytest dep.
"""

from __future__ import annotations

import os
import sys
import unittest

# Make `import validator` work when run as a bare script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import validator  # noqa: E402

# Use the shipped example config so tests exercise the real schema. The
# shipped polygon is the 0..400 x 0..300 rectangle, altitude band 0..120 m,
# separation 5 m.
CFG = validator.load_config()


class GeofenceTests(unittest.TestCase):
    def test_inside_polygon_accepted(self):
        g = {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30}}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertTrue(r["ok"], "inside-polygon goal should be accepted")
        self.assertEqual(len(r["accepted"]), 1)
        self.assertEqual(len(r["rejected"]), 0)

    def test_outside_polygon_rejected(self):
        g = {"drone_id": "drone-01", "goal": {"x": 1000, "y": 1000, "z": 30}}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertFalse(r["ok"], "out-of-bounds goal should be rejected")
        self.assertEqual(len(r["rejected"]), 1)
        reasons = " ".join(r["rejected"][0]["reasons"]).lower()
        self.assertIn("geofence", reasons)
        self.assertIn("polygon", reasons)


class AltitudeBandTests(unittest.TestCase):
    def test_below_floor_rejected(self):
        g = {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": -10}}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertFalse(r["ok"])
        self.assertIn("altitude", " ".join(r["rejected"][0]["reasons"]).lower())
        self.assertIn("floor", " ".join(r["rejected"][0]["reasons"]).lower())

    def test_above_ceiling_rejected(self):
        g = {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 200}}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertFalse(r["ok"])
        self.assertIn("ceiling", " ".join(r["rejected"][0]["reasons"]).lower())

    def test_at_floor_and_ceiling_boundary_accepted(self):
        # The band is inclusive ([floor, ceil]); these should pass.
        for z in (0.0, 120.0):
            g = {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": z}}
            r = validator.validate_goals([g], cfg=CFG)
            self.assertTrue(r["ok"], "boundary altitude z=%s should be accepted" % z)


class SeparationTests(unittest.TestCase):
    def test_too_close_rejected(self):
        # drone-01 at (50,50,30); drone-02 at (52,50,30) — only 2m apart.
        goals = [
            {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30}},
            {"drone_id": "drone-02", "goal": {"x": 52, "y": 50, "z": 30}},
        ]
        r = validator.validate_goals(goals, cfg=CFG)
        self.assertFalse(r["ok"])
        self.assertGreater(len(r["rejected"]), 0)
        self.assertIn("separation", " ".join(r["reasons"]).lower())

    def test_far_enough_accepted(self):
        # 10m apart > 5m min, both inside polygon, both in altitude band.
        goals = [
            {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30}},
            {"drone_id": "drone-02", "goal": {"x": 60, "y": 50, "z": 30}},
        ]
        r = validator.validate_goals(goals, cfg=CFG)
        self.assertTrue(r["ok"], "10m-separated goals should be accepted")
        self.assertEqual(len(r["accepted"]), 2)

    def test_separation_skips_self_drone(self):
        # A drone re-tasked to its own previous target should not collide
        # with itself.
        goals = [
            {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30}},
            {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30}},
        ]
        # The second goal replaces the first in the batch via the assigned
        # table; same drone_id is allowed to update its own target.
        r = validator.validate_goals(goals, cfg=CFG)
        self.assertTrue(r["ok"], "self re-task should not trigger separation")
        self.assertEqual(len(r["accepted"]), 2)


class PermGoalOverrideTests(unittest.TestCase):
    def test_per_goal_geofence_override_used(self):
        # Default geofence is [0..400]x[0..300]; goal at (1000,1000) is
        # outside it, but a per-goal geofence override allows it.
        g = {
            "drone_id": "drone-01",
            "goal": {"x": 1000, "y": 1000, "z": 30},
            "geofence": {"polygon": [[900, 900], [1100, 900], [1100, 1100], [900, 1100]]},
        }
        r = validator.validate_goals([g], cfg=CFG)
        self.assertTrue(r["ok"], "per-goal geofence override should admit goal")


class HallucinationGuardTests(unittest.TestCase):
    """LLM hallucination guard (manual Part 6.3 hard contract + Phase 5 task #4).

    Any forbidden control key (velocity/motor/throttle/pwm/landing_velocity/...)
    appearing anywhere in a goal is rejected wholesale before the geometric
    checks run. The LLM may emit goals/waypoints/constraints only.
    """

    def test_velocity_field_rejected(self):
        g = {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30},
             "velocity": 2.0}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertFalse(r["ok"], "velocity field must be rejected")
        self.assertEqual(len(r["rejected"]), 1)
        self.assertIn("hallucination", " ".join(r["rejected"][0]["reasons"]).lower())

    def test_motor_field_rejected(self):
        g = {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30},
             "motor": 4}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertFalse(r["ok"])

    def test_nested_throttle_rejected(self):
        # Forbidden key nested under a constraints block must also be caught.
        g = {"drone_id": "drone-01",
             "goal": {"x": 50, "y": 50, "z": 30},
             "constraints": {"throttle": 0.9}}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertFalse(r["ok"], "nested throttle must be rejected")

    def test_landing_velocity_rejected(self):
        g = {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30},
             "landing_velocity": -1.0}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertFalse(r["ok"])

    def test_clean_goal_still_accepted(self):
        # The guard must not fire on a clean goal; sanity against over-flagging.
        g = {"drone_id": "drone-01", "goal": {"x": 50, "y": 50, "z": 30}}
        r = validator.validate_goals([g], cfg=CFG)
        self.assertTrue(r["ok"], "clean goal should still be accepted")


if __name__ == "__main__":
    unittest.main(verbosity=2)