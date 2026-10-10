"""离线验证固定航点顺序、障碍净空、时间规划和预测跟踪。"""

import math
import unittest

from main import GCSApp
from navigation.calibration import (
    SPEED_STEPS,
    CalibrationSample,
    analyze_calibration,
)
from navigation.controller import CarrotPathController
from navigation.curve import fit_safe_curve
from navigation.demo import demo_obstacles, demo_waypoints
from navigation.execution import (
    EMPIRICAL_EXECUTION_TIME_SCALE,
    _scale_prediction_clock,
    predict_execution,
    sample_execution_at_time,
)
from navigation.geometry import inflate_obstacles, plan_leg_candidates
from navigation.models import (
    CircleObstacle,
    CalibrationProfile,
    ExecutionPrediction,
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
from navigation.telemetry import deviation_to_path
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
    def test_execution_prediction_interpolates_position_at_telemetry_time(self):
        samples = (
            TrajectorySample(
                time=0.0, distance=0.0, x=0.0, y=1.0, z=-5.0,
                vx=0.0, vy=0.0, vz=0.0, ax=0.0, ay=0.0, az=0.0,
                curvature=0.0,
            ),
            TrajectorySample(
                time=2.0, distance=10.0, x=10.0, y=5.0, z=-3.0,
                vx=10.0, vy=4.0, vz=2.0, ax=0.0, ay=0.0, az=0.0,
                curvature=0.2,
            ),
        )
        prediction = ExecutionPrediction(
            samples=samples, estimated_time=2.0, minimum_clearance=1.0,
            waypoint_errors=(), complete=True, safe=True,
        )

        middle = sample_execution_at_time(prediction, 1.0)

        self.assertEqual(middle.position, (5.0, 3.0, -4.0))
        self.assertEqual(middle.distance, 5.0)
        self.assertEqual((middle.vx, middle.vy, middle.vz), (5.0, 2.0, 1.0))

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

    def test_closed_route_keeps_last_waypoint_at_the_end_not_at_start(self):
        waypoints = demo_waypoints()
        trajectory = plan_ordered_route(
            (0, 0, -5), waypoints, demo_obstacles(), KinematicLimits()
        )

        prediction = trajectory.execution_prediction
        self.assertAlmostEqual(
            trajectory.waypoint_distances[-1], trajectory.length, places=3
        )
        self.assertTrue(prediction.safe)
        self.assertLessEqual(prediction.waypoint_errors[-1], 0.6)
        self.assertLess(math.dist(prediction.samples[-1].position,
                                  waypoints[-1].position), 0.6)

    def test_race_route_prediction_matches_measured_outer_turn(self):
        # 这是 route_20261010_214901.csv 的同一任务：实飞在 (21.4, -4.0)
        # 障碍附近自然外绕，预测也应保留这一条可飞路线，而不是为压低拐点误差
        # 改选另一侧、与实际飞行相差数米的候选。
        start = (2.9046869278, -23.7249641418, -5.0)
        waypoints = [
            Waypoint(21.8, -19.1, -5),
            Waypoint(24.7, -9.1, -5),
            Waypoint(15.3, -3.7, -5),
            Waypoint(3.0, -11.6, -5),
            Waypoint(-3.8, 3.8, -5),
        ]
        obstacles = [
            CircleObstacle(-0.7, -2.6, 3.0),
            CircleObstacle(30.3, -17.7, 3.0),
            CircleObstacle(21.4, -4.0, 3.0),
        ]

        trajectory = plan_ordered_route(start, waypoints, obstacles, KinematicLimits())
        prediction = trajectory.execution_prediction
        predicted_path = [sample.position for sample in prediction.samples]
        predicted_distances = [sample.distance for sample in prediction.samples]
        measured_turn_position = (26.06, -2.74)
        measured_turn_error = deviation_to_path(
            measured_turn_position[0], measured_turn_position[1],
            predicted_path, predicted_distances,
        )["dist"]

        self.assertTrue(prediction.safe)
        self.assertGreater(max(prediction.waypoint_errors), 0.6)
        self.assertLessEqual(prediction.waypoint_errors[0], 1.25)
        self.assertLessEqual(prediction.waypoint_errors[3], 1.25)
        self.assertLessEqual(prediction.waypoint_errors[2], 0.6)
        self.assertLessEqual(prediction.waypoint_errors[4], 0.6)
        # 跟踪预留从 0.25 m 提高到 0.35 m 后，外绕弧线相应变长约 0.37 m。
        self.assertAlmostEqual(trajectory.length, 85.07, delta=0.2)
        self.assertLess(measured_turn_error, 0.2)
        self.assertGreaterEqual(
            trajectory.minimum_clearance,
            KinematicLimits().safety_clearance + KinematicLimits().tracking_reserve,
        )

    def test_height_and_hold_are_included_and_speed_changes_estimated_time(self):
        waypoints = [Waypoint(5, 0, -10, hold=0.7), Waypoint(10, 5, -5)]
        trajectory = plan_ordered_route((0, 0, -5), waypoints, [], _limits())
        _assert_waypoint_order(self, trajectory, waypoints)
        self.assertGreaterEqual(trajectory.estimated_time, 0.7)
        slow = plan_ordered_route((0, 0, -5), [Waypoint(20, 0, -5)], [], _limits(2.0))
        fast = plan_ordered_route((0, 0, -5), [Waypoint(20, 0, -5)], [], _limits(6.0))
        self.assertLess(fast.estimated_time, slow.estimated_time)

    def test_speed_profile_uses_available_acceleration_and_braking_distance(self):
        limits = KinematicLimits(
            max_speed=15.0, max_acceleration=1.0, max_braking=1.0,
            max_lateral_acceleration=1.0, response_time=0.25,
            command_delay=0.10,
        )
        short = plan_ordered_route(
            (0, 0, -5), [Waypoint(20, 0, -5)], [], limits
        )
        long = plan_ordered_route(
            (0, 0, -5), [Waypoint(200, 0, -5)], [], limits
        )

        short_speeds = [math.sqrt(sample.vx ** 2 + sample.vy ** 2 + sample.vz ** 2)
                        for sample in short.samples]
        long_speeds = [math.sqrt(sample.vx ** 2 + sample.vy ** 2 + sample.vz ** 2)
                       for sample in long.samples]
        self.assertLess(max(short_speeds), 5.0)
        self.assertGreater(max(long_speeds), 13.0)

        # 速度平方沿距离的变化必须满足 v² 差 <= 2*a*距离，
        # 分别验证加速和制动方向，确保短路线不会虚构瞬时提速或刹停。
        for trajectory, speeds in ((short, short_speeds), (long, long_speeds)):
            for first, second, first_speed, second_speed in zip(
                    trajectory.samples, trajectory.samples[1:], speeds, speeds[1:]):
                distance = second.distance - first.distance
                self.assertGreaterEqual(distance, -1e-9)
                self.assertLessEqual(
                    second_speed ** 2 - first_speed ** 2,
                    2.0 * limits.max_acceleration * distance + 1e-5,
                )
                self.assertLessEqual(
                    first_speed ** 2 - second_speed ** 2,
                    2.0 * limits.max_braking * distance + 1e-5,
                )

    def test_route_selection_uses_closed_loop_time_not_ideal_curve_time(self):
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
                nominal = time_parameterize(curve, [waypoint], limits, "candidate")
                prediction = predict_execution(nominal, limits, obstacles)
                if prediction.safe:
                    estimates.append((candidate.length, nominal.estimated_time,
                                      prediction.estimated_time))
        self.assertGreaterEqual(len(estimates), 2)
        shortest = min(estimates, key=lambda row: row[0])
        fastest_predicted = min(estimates, key=lambda row: row[2])
        self.assertGreater(fastest_predicted[0], shortest[0] + 0.5)
        self.assertLess(fastest_predicted[2], shortest[2])

        selected = plan_ordered_route(
            (0, 0, -5), [waypoint], obstacles, limits, settings
        )
        self.assertAlmostEqual(selected.estimated_time, fastest_predicted[2], places=5)
        self.assertAlmostEqual(selected.nominal_time, fastest_predicted[1], places=5)
        self.assertGreaterEqual(selected.minimum_clearance,
                                limits.safety_clearance + limits.tracking_reserve)
        self.assertTrue(selected.execution_prediction.safe)
        self.assertGreater(selected.length, shortest[0] + 0.5)
        prediction = selected.execution_prediction
        self.assertAlmostEqual(
            prediction.samples[1].time - prediction.samples[0].time,
            limits.control_period * EMPIRICAL_EXECUTION_TIME_SCALE,
        )
        midpoint = sample_execution_at_time(prediction, prediction.estimated_time * 0.5)
        self.assertAlmostEqual(midpoint.time, prediction.estimated_time * 0.5)
        self.assertAlmostEqual(prediction.samples[-1].time, prediction.estimated_time)

    def test_prediction_clock_fit_preserves_waypoint_hold_duration(self):
        samples = tuple(TrajectorySample(
            time=when, distance=when, x=when, y=0.0, z=-5.0,
            vx=1.0, vy=0.0, vz=0.0, ax=2.0, ay=0.0, az=0.0,
            curvature=0.0,
        ) for when in (0.0, 1.0, 4.0, 5.0))

        adjusted, total = _scale_prediction_clock(samples, 5.0, [(1.0, 4.0)])

        self.assertAlmostEqual(adjusted[1].time - adjusted[0].time,
                               EMPIRICAL_EXECUTION_TIME_SCALE)
        self.assertAlmostEqual(adjusted[2].time - adjusted[1].time, 3.0)
        self.assertAlmostEqual(adjusted[3].time - adjusted[2].time,
                               EMPIRICAL_EXECUTION_TIME_SCALE)
        self.assertAlmostEqual(adjusted[0].vx, 1.0 / EMPIRICAL_EXECUTION_TIME_SCALE)
        self.assertAlmostEqual(adjusted[1].vx, 1.0)
        self.assertAlmostEqual(adjusted[0].ax,
                               2.0 / EMPIRICAL_EXECUTION_TIME_SCALE ** 2)
        self.assertAlmostEqual(total, adjusted[-1].time)

    def test_uncalibrated_panel_keeps_user_speed_cap_for_time_planning(self):
        limits = GCSApp._limits_for_planning(
            15.0, 0.8, CalibrationProfile(calibrated=False, safe_max_speed=2.0)
        )
        self.assertEqual(limits.max_speed, 15.0)
        self.assertEqual(limits.max_acceleration, 2.4)
        self.assertEqual(limits.max_braking, 0.75)
        self.assertEqual(limits.max_lateral_acceleration, 1.15)
        self.assertEqual(limits.tracking_reserve, 0.35)
        self.assertEqual(limits.command_delay, 0.05)
        self.assertEqual(limits.velocity_time_constant, 1.3)
        self.assertEqual(limits.velocity_acceleration, 1.3)
        self.assertEqual(limits.control_period, 0.0625)

        calibrated = GCSApp._limits_for_planning(
            15.0, 0.8,
            CalibrationProfile(calibrated=True, safe_max_speed=6.0),
        )
        self.assertEqual(calibrated.max_speed, 6.0)


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

    def test_carrot_controller_tracks_straight_line_without_lateral_wobble(self):
        limits = _limits(speed=5.0)
        controller = CarrotPathController(self.straight_trajectory(), limits)
        position, velocity = [0.0, 0.0, -5.0], [0.0, 0.0]
        maximum_deviation = 0.0
        finished = False
        for index in range(260):
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

    def test_carrot_controller_finishes_when_tracking_enters_planning_buffer(self):
        limits = _limits(speed=5.0)
        obstacle = CircleObstacle(5.0, 1.79, 1.0)
        controller = CarrotPathController(
            self.straight_trajectory(), limits, [obstacle]
        )
        position, velocity = [0.0, 0.0, -5.0], [0.0, 0.0]
        minimum_clearance = float("inf")
        finished = False
        for index in range(260):
            result = controller.step(position, velocity, index * 0.05)
            self.assertFalse(result.fault)
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
        self.assertLess(minimum_clearance, limits.safety_clearance)
        self.assertGreaterEqual(minimum_clearance, 0.10)

    def test_controller_hard_stops_at_physical_edge(self):
        limits = _limits(speed=5.0)
        route = self.straight_trajectory()
        blocked = CarrotPathController(
            route, limits, [CircleObstacle(0, 0, 1.0)]
        ).step((0, 0, -5), (0, 0, 0), 0.0)
        self.assertIn("障碍物", blocked.fault)

        # 0.8 m 是规划时的绕障余量；进入这段余量仍可继续沿路线飞行。
        buffered = CarrotPathController(
            route, limits, [CircleObstacle(5, 1.79, 1.0)]
        )
        buffered._progress_distance = 5.0
        buffered._previous_position = (5.0, 0.0, -5.0)
        buffered = buffered.check_state((5, 0, -5))
        self.assertEqual(buffered[2], "")
        self.assertAlmostEqual(buffered[1], 0.0)
        edge_clearance = math.hypot(5 - 5, 0 - 1.79) - 1.0
        self.assertAlmostEqual(edge_clearance, 0.79)

        # 接近手绘障碍物实体边缘时，仍保留小的硬保护间距。
        near_edge = CarrotPathController(
            route, limits, [CircleObstacle(5, 1.05, 1.0)]
        )
        near_edge._progress_distance = 5.0
        near_edge._previous_position = (5.0, 0.0, -5.0)
        near_edge = near_edge.check_state((5, 0, -5))
        self.assertIn("障碍物", near_edge[2])

    def test_off_route_tracking_keeps_forward_progress_and_does_not_follow_wall_clock(self):
        limits = _limits(speed=2.0)
        controller = CarrotPathController(self.straight_trajectory(), limits)

        # 虽然任务时钟已到末段，实际位置仍在前半段；参考必须跟实测进度走。
        result = controller.step((2.0, 0.8, -5.0), (0.0, 0.0), elapsed=8.0)

        self.assertEqual(result.fault, "")
        self.assertGreater(result.vx, 0.0)
        self.assertLess(result.vy, 0.0)
        self.assertIn("横向偏差", result.warning)
        self.assertLess(result.progress, 3.0)

    def test_missed_waypoint_advances_order_without_reverse_command(self):
        limits = _limits(speed=2.0)
        controller = CarrotPathController(
            self.straight_trajectory((5.0, 10.0)), limits
        )
        controller._progress_distance = 4.8
        controller._previous_position = (4.8, 0.4, -5.0)

        result = controller.step((5.2, 0.4, -5.0), (0.1, 0.0), elapsed=3.0)

        self.assertEqual(result.fault, "")
        self.assertIn("航点", result.warning)
        self.assertEqual(controller._next_waypoint, 1)
        self.assertGreater(result.vx, 0.0)
        self.assertEqual(len(result.waypoint_passages), 1)
        self.assertEqual(result.waypoint_passages[0][0], 0)
        self.assertAlmostEqual(result.waypoint_passages[0][1], 0.4)

    def test_final_waypoint_is_logged_when_stopping_inside_accuracy_radius(self):
        controller = CarrotPathController(
            self.straight_trajectory(), _limits(speed=2.0)
        )
        controller._progress_distance = 9.8
        controller._previous_position = (9.8, 0.0, -5.0)

        result = controller.step((9.9, 0.0, -5.0), (0.2, 0.0), elapsed=10.0)

        self.assertTrue(result.finished)
        self.assertEqual(len(result.waypoint_passages), 1)
        self.assertEqual(result.waypoint_passages[0][0], 0)
        self.assertAlmostEqual(result.waypoint_passages[0][1], 0.1)

    def test_missed_final_waypoint_brakes_without_reverse_recovery(self):
        controller = CarrotPathController(
            self.straight_trajectory(), _limits(speed=2.0)
        )
        controller._progress_distance = 9.8
        controller._previous_position = (9.8, 0.0, -5.0)

        result = controller.step((10.3, 0.5, -5.0), (1.0, 0.0), elapsed=10.0)

        self.assertFalse(result.finished)
        self.assertEqual(result.fault, "")
        self.assertIn("航点", result.warning)
        self.assertEqual(result.controller_mode, "terminal-braking")
        self.assertEqual((result.vx, result.vy), (0.0, 0.0))
        self.assertEqual(len(result.waypoint_passages), 1)
        self.assertEqual(result.waypoint_passages[0][0], 0)
        self.assertAlmostEqual(result.waypoint_passages[0][1], math.hypot(0.3, 0.5))

        stopped = controller.step((10.3, 0.5, -5.0), (0.2, 0.0), elapsed=10.1)
        self.assertTrue(stopped.finished)
        self.assertEqual(stopped.controller_mode, "terminal-braking")

    def test_small_tracking_error_does_not_reduce_planned_cruise_speed(self):
        route = self.straight_trajectory()
        normal = CarrotPathController(route, _limits(speed=2.0))
        offset = CarrotPathController(route, _limits(speed=2.0))

        normal_result = normal.step((2.0, 0.0, -5.0), (0.0, 0.0), elapsed=2.0)
        offset_result = offset.step((2.0, 0.4, -5.0), (0.0, 0.0), elapsed=2.0)

        self.assertAlmostEqual(
            math.hypot(normal_result.vx, normal_result.vy),
            math.hypot(offset_result.vx, offset_result.vy),
        )
        self.assertLess(offset_result.vy, 0.0)

    def test_acceleration_preview_uses_open_speed_ramp_and_respects_curvature_cap(self):
        speeds = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 0.0)

        def trajectory(curvature):
            samples = tuple(TrajectorySample(
                time=float(index), distance=float(index),
                x=float(index), y=0.0, z=-5.0,
                vx=speed, vy=0.0, vz=0.0,
                ax=0.0, ay=0.0, az=0.0, curvature=curvature,
            ) for index, speed in enumerate(speeds))
            return PlannedTrajectory(samples, (10.0,), 10.0, 10.0,
                                     float("inf"), "speed-preview")

        limits = _limits(speed=5.0)
        straight = CarrotPathController(trajectory(0.0), limits)
        current = straight._scheduled_speed(2.0)
        preview = straight._speed_with_acceleration_preview(2.0, actual_speed=1.0)
        self.assertGreater(preview, current)
        self.assertLessEqual(
            preview, current + limits.max_acceleration
            * (max(limits.response_time, limits.velocity_time_constant)
               + limits.command_delay) + 1e-9,
        )

        curved = CarrotPathController(trajectory(0.5), limits)
        current = curved._scheduled_speed(2.0)
        preview = curved._speed_with_acceleration_preview(2.0, actual_speed=1.0)
        self.assertLessEqual(
            preview, math.sqrt(limits.max_lateral_acceleration / 0.5) + 1e-9,
        )
        self.assertGreaterEqual(preview, current)

    def test_acceleration_preview_does_not_cross_a_planned_stop(self):
        speeds = (0.0, 1.0, 1.5, 2.0, 1.0, 0.0,
                  1.0, 1.5, 2.0, 1.0, 0.0)
        samples = tuple(TrajectorySample(
            time=float(index), distance=float(index),
            x=float(index), y=0.0, z=-5.0,
            vx=speed, vy=0.0, vz=0.0,
            ax=0.0, ay=0.0, az=0.0, curvature=0.0,
        ) for index, speed in enumerate(speeds))
        route = PlannedTrajectory(samples, (5.0, 10.0), 10.0, 10.0,
                                  float("inf"), "stop-preview")
        controller = CarrotPathController(route, _limits(speed=3.0))

        current = controller._scheduled_speed(4.9)
        preview = controller._speed_with_acceleration_preview(4.9, actual_speed=1.5)

        self.assertAlmostEqual(preview, current)

    def test_lookahead_naturally_cuts_toward_the_inside_of_a_turn(self):
        points = []
        for index in range(101):
            distance = index * 0.1
            x = min(5.0, distance)
            y = max(0.0, distance - 5.0)
            points.append(TrajectorySample(
                time=distance, distance=distance, x=x, y=y, z=-5.0,
                vx=0.0 if index in (0, 100) else (2.0 if distance <= 5.0 else 0.0),
                vy=0.0 if distance <= 5.0 or index in (0, 100) else 2.0,
                vz=0.0, ax=0.0, ay=0.0, az=0.0, curvature=0.0,
            ))
        trajectory = PlannedTrajectory(
            tuple(points), (10.0,), 10.0, 10.0, float("inf"), "right-angle"
        )
        controller = CarrotPathController(trajectory, _limits(speed=3.0))
        controller._progress_distance = 4.5
        controller._previous_position = (4.5, 0.0, -5.0)

        result = controller.step((4.5, 0.0, -5.0), (3.0, 0.0), elapsed=4.5)

        # 航线仍是原来的直角；前视点落在弯后，指令自然提前向弯内转。
        self.assertGreater(result.vy, 0.0)
        self.assertGreater(result.reference_distance, controller._progress_distance)

    def test_controller_prediction_period_adapts_to_telemetry_interval(self):
        controller = CarrotPathController(
            self.straight_trajectory(), _limits(speed=2.0)
        )
        for _ in range(8):
            controller.update_period(0.0625)

        self.assertGreater(controller.dt, 0.060)
        self.assertGreater(controller.dt, 0.05)

    def test_final_braking_starts_before_end_of_synthetic_constant_speed_path(self):
        controller = CarrotPathController(
            self.straight_trajectory(), _limits(speed=2.0)
        )
        controller._progress_distance = 9.5
        controller._previous_position = (9.5, 0.0, -5.0)

        result = controller.step((9.5, 0.0, -5.0), (1.0, 0.0), elapsed=9.5)

        self.assertLess(math.hypot(result.vx, result.vy), 1.0)

    def test_vertical_segment_uses_three_dimensional_progress(self):
        limits = _limits(speed=2.0)
        samples = tuple(TrajectorySample(
            time=float(index), distance=float(index), x=0.0, y=0.0,
            z=-5.0 - index, vx=0.0, vy=0.0, vz=1.0 if index else 0.0,
            ax=0.0, ay=0.0, az=0.0, curvature=0.0,
        ) for index in range(3))
        trajectory = PlannedTrajectory(samples, (2.0,), 2.0, 2.0, 0.0, "vertical")
        controller = CarrotPathController(trajectory, limits)

        progress, deviation, fault = controller.check_state((0.0, 0.0, -5.5))

        self.assertEqual(fault, "")
        self.assertAlmostEqual(progress, 0.5)
        self.assertAlmostEqual(deviation, 0.0)

    def test_carrot_controller_follows_a_certified_obstacle_curve(self):
        limits = _limits(speed=3.0)
        obstacle = CircleObstacle(5, 0, 1.0)
        trajectory = plan_ordered_route(
            (0, 0, -5), [Waypoint(10, 0, -5)], [obstacle], limits
        )
        controller = CarrotPathController(trajectory, limits, [obstacle])
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
        self.assertGreaterEqual(minimum_clearance, limits.safety_clearance - 1e-3)


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

    def test_calibration_fits_aircraft_velocity_response_from_speed_steps(self):
        samples = []
        actual_speed = 0.0
        timestamp = 0.0
        time_constant = 0.8
        acceleration = 1.2
        for requested_speed, phase, duration in (
                (2.0, "speed", 2.0), (4.0, "speed", 2.0),
                (6.0, "speed", 2.0), (0.0, "braking", 2.0)):
            for _ in range(int(duration / 0.05)):
                samples.append(CalibrationSample(
                    time=timestamp,
                    requested_speed=requested_speed,
                    along_speed=actual_speed,
                    cross_track=0.05,
                    phase=phase,
                ))
                change = (requested_speed - actual_speed) * (
                    1.0 - math.exp(-0.05 / time_constant)
                )
                change = max(-acceleration * 0.05,
                             min(acceleration * 0.05, change))
                actual_speed += change
                timestamp += 0.05

        profile = analyze_calibration(samples).profile

        self.assertAlmostEqual(profile.velocity_time_constant, time_constant, delta=0.15)
        self.assertAlmostEqual(profile.velocity_acceleration, acceleration, delta=0.15)
        self.assertAlmostEqual(profile.command_delay, 0.0, delta=1e-9)
        self.assertAlmostEqual(profile.control_period, 0.05, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
