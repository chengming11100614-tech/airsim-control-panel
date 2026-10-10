"""离线验证固定航点顺序、障碍净空、时间规划和预测跟踪。"""

import math
import unittest

from navigation.calibration import (
    SPEED_STEPS,
    CalibrationSample,
    analyze_calibration,
)
from navigation.controller import PredictivePathController
from navigation.curve import fit_safe_curve
from navigation.geometry import inflate_obstacles, plan_leg_candidates
from navigation.models import (
    CircleObstacle,
    KinematicLimits,
    PlannedTrajectory,
    TrajectorySample,
    Waypoint,
)
from navigation.planner import (
    PlanningError,
    PlannerSettings,
    _lift_leg,
    plan_ordered_route,
)
from navigation.time_parameterization import time_parameterize


def _limits(speed=2.0):
    return KinematicLimits(
        max_speed=speed,
        max_vertical_speed=2.0,
        max_acceleration=2.0,
        max_braking=2.0,
        max_lateral_acceleration=2.0,
        response_time=0.25,
        command_delay=0.10,
        safety_clearance=0.8,
        tracking_reserve=0.25,
    )


def _assert_waypoint_order(test, trajectory, waypoints):
    previous = -1.0
    for waypoint, distance in zip(waypoints, trajectory.waypoint_distances):
        test.assertGreaterEqual(distance, previous)
        sample = min(trajectory.samples, key=lambda row: abs(row.distance - distance))
        test.assertLess(math.dist(sample.position, waypoint.position), 1e-6)
        previous = distance


class RoutePlanningTests(unittest.TestCase):
    def test_saved_routes_keep_legacy_waypoints_but_ignore_per_point_speed(self):
        waypoint = Waypoint.from_dict({
            "x": 1, "y": 2, "z": -5, "speed": 0.5, "hold": 1.25,
        })
        obstacle = CircleObstacle.from_dict({"x": 3, "y": 4, "r": 2, "name": "old"})
        self.assertEqual(waypoint.position, (1, 2, -5))
        self.assertEqual(waypoint.hold, 1.25)
        self.assertNotIn("speed", waypoint.to_dict())
        self.assertEqual(obstacle.radius, 2)
        self.assertEqual(obstacle.r, 2)

    def test_nonfinite_coordinates_are_rejected_before_geometry_search(self):
        with self.assertRaises(ValueError):
            Waypoint(float("nan"), 0, -5)
        with self.assertRaises(PlanningError):
            plan_ordered_route((float("inf"), 0, -5), [Waypoint(1, 0, -5)], [], _limits())

    def test_single_and_overlapping_obstacles_keep_aircraft_center_clear(self):
        cases = (
            ([CircleObstacle(5, 0, 1.0)], [Waypoint(10, 0, -5)]),
            ([CircleObstacle(5, 0, 1.5), CircleObstacle(7, 0, 1.5)],
             [Waypoint(12, 0, -5)]),
        )
        for obstacles, waypoints in cases:
            with self.subTest(obstacles=obstacles):
                trajectory = plan_ordered_route((0, 0, -5), waypoints, obstacles, _limits())
                self.assertGreaterEqual(
                    trajectory.minimum_clearance,
                    _limits().safety_clearance + _limits().tracking_reserve - 1e-3,
                )
                _assert_waypoint_order(self, trajectory, waypoints)

    def test_enclosed_route_and_waypoint_inside_obstacle_are_rejected(self):
        ring = [CircleObstacle(
            3.0 * math.cos(2.0 * math.pi * index / 8),
            3.0 * math.sin(2.0 * math.pi * index / 8),
            0.6,
        ) for index in range(8)]
        with self.assertRaises(PlanningError):
            plan_ordered_route((0, 0, -5), [Waypoint(8, 0, -5)], ring, _limits())
        with self.assertRaises(PlanningError):
            plan_ordered_route(
                (0, 0, -5), [Waypoint(5, 0, -5)], [CircleObstacle(5, 0, 1)], _limits()
            )

    def test_height_and_hold_are_included_and_speed_changes_estimated_time(self):
        waypoints = [Waypoint(5, 0, -10, hold=0.7), Waypoint(10, 5, -5)]
        trajectory = plan_ordered_route((0, 0, -5), waypoints, [], _limits())
        _assert_waypoint_order(self, trajectory, waypoints)
        self.assertGreaterEqual(trajectory.estimated_time, 0.7)
        slow = plan_ordered_route((0, 0, -5), [Waypoint(20, 0, -5)], [], _limits(2.0))
        fast = plan_ordered_route((0, 0, -5), [Waypoint(20, 0, -5)], [], _limits(6.0))
        self.assertLess(fast.estimated_time, slow.estimated_time)

    def test_route_selection_prefers_lower_time_over_shorter_distance(self):
        obstacles = [
            CircleObstacle(4.8982827148, 0.6298900106, 1.1997694240),
            CircleObstacle(3.5704014117, -1.9565582276, 0.9372790429),
            CircleObstacle(4.0961925579, 1.2413922089, 1.2215333877),
        ]
        waypoint = Waypoint(10, 0, -5)
        limits = KinematicLimits(
            max_speed=8, max_acceleration=4, max_braking=4,
            max_lateral_acceleration=1, safety_clearance=0.8,
            tracking_reserve=0.25,
        )
        settings = PlannerSettings(max_candidates_per_leg=12)
        search = inflate_obstacles(obstacles, 0.8, 0.25, settings.curve_margin)
        protected = inflate_obstacles(obstacles, 0.8, 0.25, 0.0)
        candidates = plan_leg_candidates((0, 0), (10, 0), search, 12)
        estimates = []
        for candidate in candidates:
            points = _lift_leg(candidate, -5, -5)
            curve = fit_safe_curve(points, [len(points) - 1], protected,
                                  settings.sample_spacing)
            if curve is not None:
                estimate = time_parameterize(curve, [waypoint], limits, "candidate")
                estimates.append((candidate.length, estimate.estimated_time))
        self.assertGreaterEqual(len(estimates), 2)
        shortest = min(estimates, key=lambda row: row[0])
        fastest = min(estimates, key=lambda row: row[1])
        self.assertGreater(fastest[0], shortest[0] + 0.5)
        self.assertLess(fastest[1], shortest[1])

        selected = plan_ordered_route(
            (0, 0, -5), [waypoint], obstacles, limits, settings
        )
        self.assertAlmostEqual(selected.estimated_time, fastest[1], places=5)
        self.assertGreater(selected.length, shortest[0] + 0.5)


class ControllerTests(unittest.TestCase):
    @staticmethod
    def straight_trajectory(waypoints=(10.0,)):
        samples = tuple(TrajectorySample(
            time=index * 0.1,
            distance=index * 0.1,
            x=index * 0.1,
            y=0.0,
            z=-5.0,
            vx=1.0 if index not in (0, 100) else 0.0,
            vy=0.0,
            vz=0.0,
            ax=0.0,
            ay=0.0,
            az=0.0,
            curvature=0.0,
        ) for index in range(101))
        return PlannedTrajectory(samples, waypoints, 10.0, 10.0, float("inf"), "test")

    def test_predictive_controller_tracks_straight_line_without_lateral_wobble(self):
        limits = _limits(speed=5.0)
        controller = PredictivePathController(self.straight_trajectory(), limits)
        position, velocity = [0.0, 0.0, -5.0], [0.0, 0.0]
        maximum_deviation = 0.0
        finished = False
        for index in range(240):
            result = controller.step(position, velocity, index * 0.05)
            self.assertFalse(result.fault)
            self.assertLessEqual(math.hypot(result.vx, result.vy), limits.max_speed + 1e-6)
            maximum_deviation = max(maximum_deviation, result.cross_track)
            alpha = math.exp(-0.05 / 0.35)
            velocity = [alpha * current + (1.0 - alpha) * command
                        for current, command in zip(velocity, (result.vx, result.vy))]
            position[0] += velocity[0] * 0.05
            position[1] += velocity[1] * 0.05
            position[2] = result.z
            if result.finished:
                finished = True
                break
        self.assertTrue(finished)
        self.assertLess(maximum_deviation, 0.25)

    def test_controller_hovers_on_obstacle_clearance_or_waypoint_miss(self):
        limits = _limits(speed=5.0)
        route = self.straight_trajectory()
        blocked = PredictivePathController(
            route, limits, [CircleObstacle(0, 0, 1.0)]
        ).step((0, 0, -5), (0, 0, 0), 0.0)
        self.assertIn("障碍物", blocked.fault)

        skipped = PredictivePathController(
            self.straight_trajectory((5.0, 10.0)), limits
        ).step((10, 0, -5), (0, 0, 0), 0.0)
        self.assertIn("航点", skipped.fault)

    def test_predictive_controller_follows_a_certified_obstacle_curve(self):
        limits = _limits(speed=3.0)
        obstacle = CircleObstacle(5, 0, 1.0)
        trajectory = plan_ordered_route(
            (0, 0, -5), [Waypoint(10, 0, -5)], [obstacle], limits
        )
        controller = PredictivePathController(trajectory, limits, [obstacle])
        position, velocity = [0.0, 0.0, -5.0], [0.0, 0.0]
        maximum_deviation = 0.0
        minimum_clearance = float("inf")
        finished = False
        for index in range(1000):
            result = controller.step(position, velocity, index * 0.05)
            self.assertFalse(result.fault)
            maximum_deviation = max(maximum_deviation, result.cross_track)
            minimum_clearance = min(
                minimum_clearance,
                math.hypot(position[0] - obstacle.x, position[1] - obstacle.y)
                - obstacle.radius,
            )
            alpha = math.exp(-0.05 / 0.35)
            velocity = [alpha * current + (1.0 - alpha) * command
                        for current, command in zip(velocity, (result.vx, result.vy))]
            position[0] += velocity[0] * 0.05
            position[1] += velocity[1] * 0.05
            position[2] = result.z
            if result.finished:
                finished = True
                break
        self.assertTrue(finished)
        self.assertLess(maximum_deviation, 0.25)
        self.assertGreaterEqual(
            minimum_clearance,
            limits.safety_clearance + limits.tracking_reserve - 1e-3,
        )


class CalibrationTests(unittest.TestCase):
    def test_calibration_uses_highest_speed_that_meets_accuracy(self):
        samples = []
        timestamp = 0.0
        for speed in SPEED_STEPS[:3]:
            error = 0.1 if speed <= 2 else (0.2 if speed <= 4 else 0.4)
            for _ in range(10):
                samples.append(CalibrationSample(
                    time=timestamp,
                    requested_speed=speed,
                    along_speed=speed * 0.95,
                    cross_track=error,
                ))
                timestamp += 0.05
        result = analyze_calibration(samples)
        self.assertTrue(result.profile.calibrated)
        self.assertEqual(result.profile.safe_max_speed, 4.0)


if __name__ == "__main__":
    unittest.main()
