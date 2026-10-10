"""直线候选必须精确保留航点段，并把未能平滑接入的转向显式计时。"""

import math
import unittest

from navigation.curve import fit_safe_curve, splice_straight_leg
from navigation.models import KinematicLimits, Waypoint
from navigation.time_parameterization import time_parameterize


class StraightLegCandidateTests(unittest.TestCase):
    def setUp(self):
        self.points = [
            (0.0, 0.0, -5.0),
            (10.0, 0.0, -5.0),
            (10.0, 10.0, -5.0),
            (20.0, 10.0, -5.0),
        ]
        self.anchors = [1, 2, 3]
        self.waypoints = [
            Waypoint(10.0, 0.0, -5.0),
            Waypoint(10.0, 10.0, -5.0),
            Waypoint(20.0, 10.0, -5.0),
        ]

    def test_smooth_candidate_keeps_clear_waypoint_leg_straight(self):
        curve = fit_safe_curve(
            self.points, self.anchors, (), preserve_straight_leg=1
        )

        self.assertIsNotNone(curve)
        start_distance = curve.waypoint_distances[0]
        end_distance = curve.waypoint_distances[1]
        segment = [
            point for distance, point in zip(curve.distances, curve.positions)
            if start_distance - 1e-7 <= distance <= end_distance + 1e-7
        ]
        self.assertGreater(len(segment), 2)
        self.assertTrue(all(abs(point[0] - 10.0) < 1e-7 for point in segment))
        self.assertAlmostEqual(curve.positions[0][2], -5.0)
        self.assertAlmostEqual(curve.waypoint_distances[1] - start_distance, 10.0, places=3)

    def test_stop_and_go_straight_candidate_sets_zero_speed_at_both_corners(self):
        base = fit_safe_curve(self.points, self.anchors, ())
        curve = splice_straight_leg(
            self.points, self.anchors, base, leg_index=1
        )
        trajectory = time_parameterize(
            curve, self.waypoints, KinematicLimits(max_speed=2.0), "test"
        )

        self.assertEqual(len(curve.stop_distances), 2)
        for stop_distance in curve.stop_distances:
            sample = min(trajectory.samples, key=lambda item: abs(item.distance - stop_distance))
            self.assertAlmostEqual(math.hypot(sample.vx, sample.vy), 0.0, places=6)
        self.assertAlmostEqual(
            curve.waypoint_distances[1] - curve.waypoint_distances[0], 10.0, places=3
        )


if __name__ == "__main__":
    unittest.main()
