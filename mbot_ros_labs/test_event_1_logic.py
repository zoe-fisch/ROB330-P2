import csv
import math
import tempfile
import unittest
from pathlib import Path

from event_1_logic import (
    CSV_FIELDS,
    ControlConfig,
    CsvLogger,
    Waypoint,
    WaypointController,
    make_square_waypoints,
    pose_jump,
    safety_error,
    wrap,
)


class Event1LogicTests(unittest.TestCase):
    def test_square_geometry_for_cardinal_start_headings_and_directions(self):
        side = 0.8
        for heading in (0.0, math.pi / 2, math.pi, -math.pi / 2):
            for direction in (-1, 1):
                with self.subTest(heading=heading, direction=direction):
                    x0, y0 = 1.25, -0.75
                    waypoints = make_square_waypoints(
                        x0, y0, heading, side, direction, laps=2
                    )
                    self.assertEqual(len(waypoints), 8)
                    locations = [(x0, y0)] + [
                        (point.x, point.y) for point in waypoints[:4]
                    ]
                    for first, second in zip(locations, locations[1:]):
                        self.assertAlmostEqual(
                            math.dist(first, second), side, places=9
                        )

                    area = sum(
                        locations[i][0] * locations[i + 1][1]
                        - locations[i + 1][0] * locations[i][1]
                        for i in range(4)
                    ) / 2
                    self.assertEqual(math.copysign(1, area), direction)

                    for first, second in zip(waypoints, waypoints[1:]):
                        self.assertAlmostEqual(
                            math.dist(
                                (first.x, first.y),
                                (second.x, second.y),
                            ),
                            side,
                            places=9,
                        )

    def test_heading_wraps_at_pi_boundary(self):
        self.assertAlmostEqual(wrap(-math.pi + 0.1 - (math.pi - 0.1)), 0.2)
        self.assertAlmostEqual(wrap(math.pi + 0.1), -math.pi + 0.1)

    def test_turn_hysteresis_and_live_bearing(self):
        controller = WaypointController(ControlConfig())
        target = Waypoint(1.0, 0.0, 0.0, 1, 1)

        first = controller.step((0.0, 0.0, 0.0), target)
        self.assertEqual(first.state, "DRIVE")

        entering_turn = controller.step((0.0, 0.0, -0.5), target)
        self.assertEqual(entering_turn.state, "TURN")
        self.assertAlmostEqual(entering_turn.target_heading, 0.0)
        self.assertEqual(entering_turn.linear_velocity, 0.0)

        hysteresis_band = controller.step((0.0, 0.0, -0.2), target)
        self.assertEqual(hysteresis_band.state, "TURN")

        exiting_turn = controller.step((0.0, 0.0, -0.05), target)
        self.assertEqual(exiting_turn.state, "DRIVE")
        self.assertGreater(exiting_turn.linear_velocity, 0.0)

    def test_live_bearing_tracks_lateral_displacement(self):
        controller = WaypointController(ControlConfig())
        target = Waypoint(1.0, 0.0, 0.0, 1, 1)
        decision = controller.step((0.0, 0.2, 0.0), target)
        self.assertAlmostEqual(
            decision.target_heading, math.atan2(-0.2, 1.0)
        )
        self.assertLess(decision.heading_error, 0.0)

    def test_slowdown_and_waypoint_completion(self):
        controller = WaypointController(ControlConfig())
        target = Waypoint(1.0, 0.0, 0.0, 1, 1)
        close = controller.step((0.9, 0.0, 0.0), target)
        self.assertEqual(close.state, "DRIVE")
        self.assertLess(close.linear_velocity, controller.config.speed)

        reached = controller.step((0.98, 0.0, 1.0), target)
        self.assertTrue(reached.waypoint_reached)
        self.assertEqual(reached.state, "COMPLETE")
        self.assertEqual(reached.linear_velocity, 0.0)
        self.assertEqual(reached.angular_velocity, 0.0)

    def test_safety_checks_localization_scan_timeout_and_obstacle(self):
        limits = (0.7, 0.7, 100.0, 0.2)
        self.assertEqual(
            safety_error(0.8, 0.1, 1.0, 1.0, *limits),
            "No fresh /estimated_pose",
        )
        self.assertEqual(
            safety_error(0.1, 0.8, 1.0, 1.0, *limits),
            "No fresh /scan",
        )
        self.assertEqual(
            safety_error(0.1, 0.1, 101.0, 1.0, *limits),
            "Run timeout exceeded",
        )
        self.assertEqual(
            safety_error(0.1, 0.1, 1.0, 0.1, *limits),
            "Lidar obstacle detected",
        )
        self.assertEqual(safety_error(0.1, 0.1, 1.0, 1.0, *limits), "")

    def test_pose_jump_guard(self):
        self.assertEqual(
            pose_jump((0.0, 0.0, 0.0), (0.5, 0.0, 0.0), 0.35, 1.0),
            "Estimated pose jumped 0.500 m",
        )
        self.assertIn(
            "heading jumped",
            pose_jump((0.0, 0.0, 0.0), (0.0, 0.0, 1.5), 0.35, 1.0),
        )
        self.assertEqual(
            pose_jump(None, (0.0, 0.0, 0.0), 0.35, 1.0),
            "",
        )

    def test_csv_logger_writes_requested_header_and_flushes_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = CsvLogger(tmp)
            log_path = logger.path
            logger.write(
                {
                    "elapsed_s": "1.25",
                    "estimated_x_m": 2.0,
                    "command_angular_rad_s": 0.5,
                }
            )
            with log_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))
            logger.close()

        self.assertEqual(tuple(rows[0]), CSV_FIELDS)
        self.assertEqual(rows[1][CSV_FIELDS.index("elapsed_s")], "1.25")
        self.assertEqual(rows[1][CSV_FIELDS.index("estimated_x_m")], "2.0")
        self.assertEqual(
            rows[1][CSV_FIELDS.index("command_angular_rad_s")], "0.5"
        )

    def test_partial_square_route_has_requested_count(self):
        points = make_square_waypoints(0.0, 0.0, 0.0, 1.0, 1, laps=1, sides=1)
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0].x, 1.0)
        self.assertAlmostEqual(points[0].y, 0.0)


if __name__ == "__main__":
    unittest.main()
