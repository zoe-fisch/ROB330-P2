#!/usr/bin/env python3
"""ROB 330 Event 1: logged, feedback-controlled four-lap square navigation."""

import argparse
import importlib.util
import math
import time
from pathlib import Path

if importlib.util.find_spec("rclpy") is None:
    rclpy = None

    class Node:
        pass
else:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan
    from std_srvs.srv import SetBool

from event_1_logic import (
    ControlConfig,
    CsvLogger,
    WaypointController,
    make_square_waypoints,
    pose_jump,
    safety_error,
    wrap,
    yaw_from_quaternion,
)


class Event1Controller(Node):
    def __init__(self, args, csv_log):
        super().__init__("event_1_looper")
        self.args = args
        self.csv_log = csv_log
        self.pose = None
        self.pose_frame = ""
        self.pose_time = None
        self.previous_pose = None
        self.pose_jump_message = ""
        self.scan_time = None
        self.closest = float("inf")
        self.odom = None
        self.odom_time = None
        self.imu_angular_velocity_z = None
        self.imu_time = None
        self.start_time = None
        self.start_pose = None
        self.lap = 0
        self.side = 0
        self.state = "WAITING"
        self.active_turn = None
        self.last_turn_metrics = {}
        self.side_state_transitions = 0
        self.last_command = (0.0, 0.0)
        self.last_command_time = time.monotonic()
        self.last_iteration = {}
        self.last_heading_error = None
        self.last_heading_warning = 0.0
        self.last_direction_warning = 0.0
        self.motion_window = None

        self.control_config = ControlConfig(
            speed=args.speed,
            min_speed=args.min_speed,
            max_turn=args.max_turn,
            turn_gain=args.turn_gain,
            drive_gain=args.drive_gain,
            position_tolerance=args.position_tolerance,
            turn_enter=math.radians(args.turn_enter_deg),
            turn_exit=math.radians(args.turn_exit_deg),
            slowdown_distance=args.slowdown_distance,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/estimated_pose",
            self.pose_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.freeze_client = self.create_client(SetBool, "/slam/freeze_mapping")

    def pose_callback(self, message):
        pose = message.pose.pose
        current = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )
        if message.header.frame_id != "map":
            self.pose_jump_message = (
                "Expected /estimated_pose in map frame, got "
                + repr(message.header.frame_id)
            )
        else:
            self.pose_jump_message = pose_jump(
                self.previous_pose,
                current,
                self.args.max_pose_jump,
                math.radians(self.args.max_yaw_jump_deg),
            )

        self.previous_pose = current
        self.pose = current
        self.pose_frame = message.header.frame_id
        self.pose_time = time.monotonic()

    def scan_callback(self, message):
        self.scan_time = time.monotonic()
        valid = (
            reading
            for reading in message.ranges
            if math.isfinite(reading)
            and message.range_min <= reading <= message.range_max
        )
        self.closest = min(valid, default=float("inf"))

    def odom_callback(self, message):
        self.odom = {
            "frame": message.header.frame_id,
            "child_frame": message.child_frame_id,
            "x": message.pose.pose.position.x,
            "y": message.pose.pose.position.y,
            "yaw": yaw_from_quaternion(message.pose.pose.orientation),
            "angular_velocity": message.twist.twist.angular.z,
        }
        self.odom_time = time.monotonic()

    def imu_callback(self, message):
        self.imu_angular_velocity_z = message.angular_velocity.z
        self.imu_time = time.monotonic()

    def _integrate_command(self, now):
        if self.active_turn is not None:
            elapsed = max(0.0, now - self.last_command_time)
            self.active_turn["commanded_abs_rad"] += (
                abs(self.last_command[1]) * elapsed
            )
        self.last_command_time = now

    def send(self, linear=0.0, angular=0.0):
        now = time.monotonic()
        old_angular = self.last_command[1]
        self._integrate_command(now)

        if (
            abs(old_angular) > 0.08
            and abs(angular) > 0.08
            and old_angular * angular < 0
            and now - self.last_direction_warning > 0.5
        ):
            self.get_logger().warning(
                "Angular command reversed direction; inspect turn and pose logs"
            )
            self.last_direction_warning = now

        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)
        self.last_command = (float(linear), float(angular))
        self.last_command_time = now

    def stop(self):
        self.send()

    def elapsed(self):
        if self.start_time is None:
            return 0.0
        return max(0.0, time.monotonic() - self.start_time)

    def _tick(self, lap, side):
        rclpy.spin_once(self, timeout_sec=0.05)
        now = time.monotonic()
        pose_age = None if self.pose_time is None else now - self.pose_time
        scan_age = None if self.scan_time is None else now - self.scan_time
        error = safety_error(
            pose_age,
            scan_age,
            self.elapsed(),
            self.closest,
            self.args.pose_timeout,
            self.args.scan_timeout,
            self.args.timeout,
            self.args.stop_distance,
        )
        if self.pose_jump_message:
            error = self.pose_jump_message
        if error:
            self.send()
            self._write_row(
                lap=lap,
                side=side,
                event="SAFETY_STOP",
                safety_stop=True,
                error_message=error,
                now=now,
            )
            raise RuntimeError(error)
        return now

    def wait_for_inputs(self):
        deadline = time.monotonic() + self.args.startup_timeout
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if self.pose is not None and self.scan_time is not None:
                pose_age = now - self.pose_time
                scan_age = now - self.scan_time
                if self.pose_frame != "map":
                    raise RuntimeError(
                        f"/estimated_pose must use map frame, got {self.pose_frame!r}"
                    )
                if (
                    pose_age <= self.args.pose_timeout
                    and scan_age <= self.args.scan_timeout
                ):
                    return
            if now > deadline:
                raise RuntimeError(
                    "Timed out waiting for fresh /estimated_pose and /scan"
                )

    def _begin_turn(self, pose, target_heading, initial_error, now):
        self.active_turn = {
            "start_yaw": pose[2],
            "target_heading": target_heading,
            "initial_error": initial_error,
            "estimated_change": 0.0,
            "last_yaw": pose[2],
            "estimated_end_yaw": "",
            "commanded_abs_rad": 0.0,
            "transitions": 1,
            "started_at": now,
        }
        self.last_heading_error = abs(initial_error)

    def _update_turn_pose(self, pose):
        if self.active_turn is None:
            return
        turn = self.active_turn
        turn["estimated_change"] += wrap(pose[2] - turn["last_yaw"])
        turn["last_yaw"] = pose[2]

    def _finish_turn(self, pose, now):
        if self.active_turn is None:
            return
        self._integrate_command(now)
        self.active_turn["transitions"] += 1
        self.active_turn["estimated_end_yaw"] = pose[2]
        self.last_turn_metrics = dict(self.active_turn)
        duration = now - self.active_turn["started_at"]
        self.get_logger().info(
            "Turn complete: estimated heading change "
            f"{math.degrees(self.active_turn['estimated_change']):.1f} deg; "
            "commanded angular displacement "
            f"{math.degrees(self.active_turn['commanded_abs_rad']):.1f} deg "
            f"over {duration:.2f} s; "
            f"{self.active_turn['transitions']} TURN/DRIVE transitions"
        )
        self.active_turn = None

    def _state_change(self, decision, pose, now, lap, side):
        if not decision.transition_reason:
            return ""

        old_state = decision.previous_state or "START"
        transition = f"{old_state} -> {decision.state}"
        self.side_state_transitions += 1

        if old_state == "TURN":
            self._finish_turn(pose, now)
        if decision.state == "TURN":
            self._integrate_command(now)
            self._begin_turn(
                pose,
                decision.target_heading,
                decision.heading_error,
                now,
            )
        self.state = decision.state
        self.get_logger().info(
            f"[{self.elapsed():.2f}s] LAP {lap} SIDE {side} | "
            f"{transition} | heading error: "
            f"{math.degrees(decision.heading_error):.1f} deg | "
            f"target distance: {decision.distance:.2f} m | "
            f"{decision.transition_reason}"
        )
        return transition

    def _check_stall(self, state, pose, now):
        if state not in ("TURN", "DRIVE"):
            self.motion_window = None
            return
        if (
            self.motion_window is None
            or self.motion_window["state"] != state
        ):
            self.motion_window = {
                "state": state,
                "started_at": now,
                "pose": pose,
            }
            return
        window = self.motion_window
        if now - window["started_at"] < self.args.stall_timeout:
            return

        if state == "DRIVE":
            moved = math.hypot(
                pose[0] - window["pose"][0],
                pose[1] - window["pose"][1],
            )
            stalled = moved < self.args.stall_position
            measure = f"{moved:.3f} m translation"
        else:
            moved = abs(wrap(pose[2] - window["pose"][2]))
            stalled = moved < math.radians(self.args.stall_heading_deg)
            measure = f"{math.degrees(moved):.1f} deg rotation"

        if stalled:
            message = (
                f"Motion stalled in {state}: only {measure} in "
                f"{now - window['started_at']:.1f} s"
            )
            self._write_row(
                lap=self.lap,
                side=self.side,
                event="STALL",
                error_message=message,
                now=now,
            )
            raise RuntimeError(message)

        self.motion_window = {
            "state": state,
            "started_at": now,
            "pose": pose,
        }

    def _warn_if_heading_worsens(self, decision, now):
        current = abs(decision.heading_error)
        previous = self.last_heading_error
        if (
            decision.state == "TURN"
            and previous is not None
            and current > previous + math.radians(12.0)
            and now - self.last_heading_warning > 1.0
        ):
            self.get_logger().warning(
                "Heading error grew unexpectedly to "
                f"{math.degrees(current):.1f} deg"
            )
            self.last_heading_warning = now
        self.last_heading_error = current

    def _write_row(
        self,
        lap=None,
        side=None,
        decision=None,
        event="",
        transition_reason="",
        safety_stop=False,
        error_message="",
        event_detail="",
        now=None,
    ):
        now = time.monotonic() if now is None else now
        pose_age = (
            "" if self.pose_time is None else max(0.0, now - self.pose_time)
        )
        scan_age = (
            "" if self.scan_time is None else max(0.0, now - self.scan_time)
        )
        odom_age = (
            "" if self.odom_time is None else max(0.0, now - self.odom_time)
        )
        imu_age = (
            "" if self.imu_time is None else max(0.0, now - self.imu_time)
        )
        pose = self.pose or ("", "", "")
        start_pose = self.start_pose or ("", "", "")
        linear, angular = self.last_command
        turn = self.active_turn or self.last_turn_metrics
        row = {
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "elapsed_s": f"{self.elapsed():.6f}",
            "ros_time_s": f"{self.get_clock().now().nanoseconds / 1e9:.9f}",
            "lap": self.lap if lap is None else lap,
            "side": self.side if side is None else side,
            "state": self.state,
            "transition_reason": transition_reason,
            "event": event,
            "event_detail": event_detail,
            "pose_frame": self.pose_frame,
            "estimated_x_m": pose[0],
            "estimated_y_m": pose[1],
            "estimated_yaw_rad": pose[2],
            "estimated_yaw_deg": (
                "" if pose[2] == "" else math.degrees(pose[2])
            ),
            "start_x_m": start_pose[0],
            "start_y_m": start_pose[1],
            "start_yaw_rad": start_pose[2],
            "pose_age_s": pose_age,
            "command_linear_mps": linear,
            "command_angular_rad_s": angular,
            "command_angular_deg_s": math.degrees(angular),
            "command_type": (
                "translating"
                if abs(linear) > 1e-6
                else "turning"
                if abs(angular) > 1e-6
                else "stopped"
            ),
            "min_lidar_range_m": self.closest,
            "scan_age_s": scan_age,
            "odom_age_s": odom_age,
            "imu_age_s": imu_age,
            "safety_stop": safety_stop,
            "error_message": error_message,
            "turn_start_yaw_rad": turn.get("start_yaw", ""),
            "turn_target_heading_rad": turn.get("target_heading", ""),
            "turn_initial_error_deg": (
                ""
                if "initial_error" not in turn
                else math.degrees(turn["initial_error"])
            ),
            "turn_estimated_change_deg": (
                ""
                if "estimated_change" not in turn
                else math.degrees(turn["estimated_change"])
            ),
            "turn_estimated_end_yaw_rad": turn.get("estimated_end_yaw", ""),
            "turn_commanded_abs_displacement_deg": (
                ""
                if "commanded_abs_rad" not in turn
                else math.degrees(turn["commanded_abs_rad"])
            ),
            "turn_state_transitions": turn.get("transitions", ""),
            "side_state_transitions": self.side_state_transitions,
            "dry_run": False,
        }
        if decision is not None:
            row.update(
                {
                    "waypoint_x_m": self.last_iteration.get("waypoint_x", ""),
                    "waypoint_y_m": self.last_iteration.get("waypoint_y", ""),
                    "expected_side_heading_rad": self.last_iteration.get(
                        "expected_heading", ""
                    ),
                    "target_heading_rad": decision.target_heading,
                    "distance_m": decision.distance,
                    "along_track_error_m": decision.along_track_error,
                    "cross_track_error_m": decision.cross_track_error,
                    "heading_error_deg": math.degrees(decision.heading_error),
                    "waypoint_reached": decision.waypoint_reached,
                }
            )
        if self.odom is not None:
            row.update(
                {
                    "odom_frame": self.odom["frame"],
                    "odom_child_frame": self.odom["child_frame"],
                    "odom_x_m": self.odom["x"],
                    "odom_y_m": self.odom["y"],
                    "odom_yaw_rad": self.odom["yaw"],
                    "odom_twist_angular_rad_s": self.odom["angular_velocity"],
                }
            )
        if self.imu_angular_velocity_z is not None:
            row["imu_angular_velocity_z_rad_s"] = self.imu_angular_velocity_z
        self.csv_log.write(row)

    def follow_waypoint(self, waypoint):
        self.lap = waypoint.lap
        self.side = waypoint.side
        self.side_state_transitions = 0
        self.last_heading_error = None
        self.motion_window = None
        self.last_iteration = {
            "waypoint_x": waypoint.x,
            "waypoint_y": waypoint.y,
            "expected_heading": waypoint.expected_heading,
        }
        self.get_logger().info(
            f"[{self.elapsed():.2f}s] LAP {waypoint.lap} SIDE {waypoint.side} "
            f"waypoint=({waypoint.x:.3f}, {waypoint.y:.3f}) m; "
            f"expected side heading={math.degrees(waypoint.expected_heading):.1f} deg"
        )
        controller = WaypointController(self.control_config)
        phase_start = time.monotonic()

        while rclpy.ok():
            now = self._tick(waypoint.lap, waypoint.side)
            if now - phase_start > self.args.segment_timeout:
                message = (
                    f"Waypoint timeout: lap {waypoint.lap}, "
                    f"side {waypoint.side}"
                )
                self.send()
                self._write_row(
                    event="TIMEOUT",
                    error_message=message,
                    now=now,
                )
                raise RuntimeError(message)

            pose = self.pose
            self._update_turn_pose(pose)
            decision = controller.step(pose, waypoint)
            transition = self._state_change(
                decision, pose, now, waypoint.lap, waypoint.side
            )
            if transition and decision.state == "TURN" and decision.previous_state:
                self.send()

            if decision.waypoint_reached:
                self.stop()
                self.state = "COMPLETE"
                self.motion_window = None
                self._write_row(
                    decision=decision,
                    event="WAYPOINT_REACHED",
                    transition_reason=decision.transition_reason,
                    now=now,
                )
                self.get_logger().info(
                    f"[{self.elapsed():.2f}s] LAP {waypoint.lap} "
                    f"SIDE {waypoint.side} waypoint reached "
                    f"({decision.distance:.3f} m remaining)"
                )
                return

            self._warn_if_heading_worsens(decision, now)
            self.send(
                decision.linear_velocity,
                decision.angular_velocity,
            )
            self._check_stall(decision.state, pose, now)
            self._write_row(
                decision=decision,
                event=transition,
                transition_reason=decision.transition_reason,
                now=now,
            )

        raise RuntimeError("ROS shutdown while following waypoint")

    def rotate_to(self, target, label, lap=0, side=0):
        started = time.monotonic()
        self.lap, self.side = lap, side
        self.side_state_transitions = 0
        self.motion_window = None
        self.state = "TURN"
        self._begin_turn(
            self.pose,
            target,
            wrap(target - self.pose[2]),
            started,
        )
        self.get_logger().info(
            f"{label}: rotate to {math.degrees(target):.1f} deg "
            f"from {math.degrees(self.pose[2]):.1f} deg"
        )

        while rclpy.ok():
            now = self._tick(lap, side)
            if now - started > self.args.rotation_timeout:
                message = f"{label} timeout"
                self.send()
                self._write_row(
                    event="TIMEOUT",
                    error_message=message,
                    now=now,
                )
                raise RuntimeError(message)

            pose = self.pose
            self._update_turn_pose(pose)
            error = wrap(target - pose[2])
            if abs(error) <= math.radians(self.args.rotation_tolerance_deg):
                self.stop()
                self._finish_turn(pose, now)
                self.state = "COMPLETE"
                self._write_row(
                    event="ROTATION_REACHED",
                    now=now,
                )
                return

            angular = max(
                -self.args.max_turn,
                min(self.args.max_turn, self.args.turn_gain * error),
            )
            self.send(0.0, angular)
            self._check_stall("TURN", pose, now)
            self._write_row(
                event="ROTATING",
                now=now,
            )

        raise RuntimeError(f"ROS shutdown during {label}")

    def freeze_mapping(self):
        if not self.freeze_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                "Freeze service unavailable; continuing with live mapping"
            )
            return

        request = SetBool.Request()
        request.data = True
        future = self.freeze_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if (
            not future.done()
            or future.result() is None
            or not future.result().success
        ):
            self.get_logger().warning("Map freeze was not confirmed")
        else:
            self.get_logger().info("Map frozen; localization continues")

    def run_rotation_test(self):
        self.start_time = time.monotonic()
        self.wait_for_inputs()
        self.start_pose = self.pose
        direction = 1 if self.args.rotation_direction == "ccw" else -1
        target = wrap(self.pose[2] + direction * math.pi / 2)
        starting_yaw = self.pose[2]
        self.rotate_to(target, "ISOLATED ROTATION TEST")
        estimated_rotation = wrap(self.pose[2] - starting_yaw)
        rotation_error = wrap(direction * math.pi / 2 - estimated_rotation)
        self.get_logger().info(
            "ISOLATED ROTATION RESULT: estimated rotation "
            f"{math.degrees(estimated_rotation):.1f} deg; "
            f"error from requested {direction * 90:+d} deg is "
            f"{math.degrees(rotation_error):.1f} deg"
        )

    def run_competition(self):
        self.start_time = time.monotonic()
        self.get_logger().info("Waiting for fresh SLAM pose and lidar...")
        self.wait_for_inputs()
        self.start_pose = self.pose
        x0, y0, h0 = self.start_pose
        direction = 1 if self.args.ccw else -1
        waypoints = make_square_waypoints(
            x0,
            y0,
            h0,
            self.args.side,
            direction,
            laps=self.args.laps,
            sides=self.args.sides,
        )

        for lap in range(1, self.args.laps + 1):
            self.get_logger().info(
                f"[{self.elapsed():.2f}s] Starting LAP {lap}/{self.args.laps} "
                f"({'counterclockwise' if direction > 0 else 'clockwise'})"
            )
            for waypoint in waypoints[
                (lap - 1) * self.args.sides : lap * self.args.sides
            ]:
                self.follow_waypoint(waypoint)
            if (
                lap == 1
                and self.args.sides == 4
                and self.args.freeze_after_first
            ):
                self.freeze_mapping()

        self.rotate_to(
            h0,
            "Final orientation",
            lap=self.args.laps,
            side=self.args.sides,
        )
        final_pose = self.pose
        position_error = math.hypot(final_pose[0] - x0, final_pose[1] - y0)
        heading_error = abs(wrap(final_pose[2] - h0))
        self.state = "COMPLETE"
        self.stop()
        self._write_row(
            lap=self.args.laps,
            side=self.args.sides,
            event="RUN_COMPLETE",
            event_detail=(
                f"position_error_m={position_error:.4f}; "
                f"heading_error_deg={math.degrees(heading_error):.2f}"
            ),
        )
        self.get_logger().info(
            "FINISHED: position error "
            f"{position_error:.3f} m, heading error "
            f"{math.degrees(heading_error):.1f} deg"
        )

    def write_failure(self, message):
        self._write_row(
            event="ERROR",
            safety_stop=True,
            error_message=message,
        )


def dry_run(args):
    direction = 1 if args.ccw else -1
    pose = (args.start_x, args.start_y, math.radians(args.start_yaw_deg))
    waypoints = make_square_waypoints(
        pose[0],
        pose[1],
        pose[2],
        args.side,
        direction,
        laps=args.laps,
        sides=args.sides,
    )
    config = ControlConfig(
        speed=args.speed,
        min_speed=args.min_speed,
        max_turn=args.max_turn,
        turn_gain=args.turn_gain,
        drive_gain=args.drive_gain,
        position_tolerance=args.position_tolerance,
        turn_enter=math.radians(args.turn_enter_deg),
        turn_exit=math.radians(args.turn_exit_deg),
        slowdown_distance=args.slowdown_distance,
    )
    print(
        "DRY RUN only: idealized pose updates, no ROS node and no velocity "
        "commands are created."
    )
    print(
        f"Route: {len(waypoints)} waypoints, side={args.side:.3f} m, "
        f"direction={'CCW' if direction > 0 else 'CW'}"
    )

    for waypoint in waypoints:
        controller = WaypointController(config)
        decision = controller.step(pose, waypoint)
        if decision.state == "TURN":
            print(
                f"LAP {waypoint.lap} SIDE {waypoint.side}: TURN "
                f"toward live bearing {math.degrees(decision.target_heading):.1f} deg"
            )
            pose = (pose[0], pose[1], decision.target_heading)
            decision = controller.step(pose, waypoint)
        print(
            f"LAP {waypoint.lap} SIDE {waypoint.side}: "
            f"{decision.state} toward ({waypoint.x:.3f}, {waypoint.y:.3f})"
        )
        pose = (waypoint.x, waypoint.y, waypoint.expected_heading)
        completion = controller.step(pose, waypoint)
        if not completion.waypoint_reached:
            raise RuntimeError("Dry-run waypoint did not enter position tolerance")

    print(
        f"DRY RUN complete: {len(waypoints)} waypoints; "
        "all idealized side lengths and state transitions passed."
    )
    return 0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", type=float, default=1.0)
    parser.add_argument("--laps", type=int, default=4)
    parser.add_argument("--sides", type=int, default=4)
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--min-speed", type=float, default=0.035)
    parser.add_argument("--max-turn", type=float, default=0.50)
    parser.add_argument("--turn-gain", type=float, default=1.8)
    parser.add_argument("--drive-gain", type=float, default=1.4)
    parser.add_argument("--position-tolerance", type=float, default=0.025)
    parser.add_argument("--turn-enter-deg", type=float, default=25.0)
    parser.add_argument("--turn-exit-deg", type=float, default=8.0)
    parser.add_argument("--rotation-tolerance-deg", type=float, default=3.0)
    parser.add_argument("--slowdown-distance", type=float, default=0.30)
    parser.add_argument("--stop-distance", type=float, default=0.20)
    parser.add_argument("--timeout", type=float, default=285.0)
    parser.add_argument("--segment-timeout", type=float, default=35.0)
    parser.add_argument("--rotation-timeout", type=float, default=30.0)
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--pose-timeout", type=float, default=0.7)
    parser.add_argument("--scan-timeout", type=float, default=0.7)
    parser.add_argument("--max-pose-jump", type=float, default=0.35)
    parser.add_argument("--max-yaw-jump-deg", type=float, default=80.0)
    parser.add_argument("--stall-timeout", type=float, default=3.0)
    parser.add_argument("--stall-position", type=float, default=0.015)
    parser.add_argument("--stall-heading-deg", type=float, default=2.0)
    parser.add_argument("--log-dir", type=Path, default=Path.home() / "mbot_logs/event_1")
    parser.add_argument("--ccw", action="store_true")
    parser.add_argument("--freeze-after-first", action="store_true")
    parser.add_argument("--rotation-test", action="store_true")
    parser.add_argument(
        "--rotation-direction",
        choices=("ccw", "cw"),
        default="ccw",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--start-yaw-deg", type=float, default=0.0)
    args = parser.parse_args()

    if args.side <= 0:
        parser.error("--side must be positive")
    if args.laps < 1 or args.laps > 4:
        parser.error("--laps must be between 1 and 4")
    if args.sides < 1 or args.sides > 4:
        parser.error("--sides must be between 1 and 4")
    if args.speed <= 0 or args.min_speed <= 0 or args.min_speed > args.speed:
        parser.error("require 0 < --min-speed <= --speed")
    if args.max_turn <= 0 or args.slowdown_distance <= 0:
        parser.error("--max-turn and --slowdown-distance must be positive")
    if args.turn_gain <= 0 or args.drive_gain <= 0:
        parser.error("--turn-gain and --drive-gain must be positive")
    if args.turn_exit_deg < 0 or args.turn_enter_deg <= args.turn_exit_deg:
        parser.error("require 0 <= --turn-exit-deg < --turn-enter-deg")
    if args.turn_enter_deg >= 180 or args.rotation_tolerance_deg >= 180:
        parser.error("heading thresholds must be below 180 degrees")
    if args.rotation_tolerance_deg <= 0:
        parser.error("--rotation-tolerance-deg must be positive")
    if args.position_tolerance <= 0 or args.stop_distance < 0:
        parser.error("position tolerance must be positive; stop distance nonnegative")
    if min(
        args.timeout,
        args.segment_timeout,
        args.rotation_timeout,
        args.startup_timeout,
        args.pose_timeout,
        args.scan_timeout,
        args.stall_timeout,
    ) <= 0:
        parser.error("timeouts must be positive")
    if (
        args.max_pose_jump <= 0
        or args.max_yaw_jump_deg <= 0
        or args.stall_position <= 0
        or args.stall_heading_deg <= 0
    ):
        parser.error("pose-jump and stall thresholds must be positive")
    if args.dry_run and args.rotation_test:
        parser.error("--dry-run and --rotation-test are mutually exclusive")
    return args


def main():
    args = parse_args()
    if args.dry_run:
        try:
            return dry_run(args)
        except Exception as error:
            print(f"DRY RUN FAILED: {error}")
            return 1
    if rclpy is None:
        print("ROS 2 Python packages are required unless --dry-run is used")
        return 1

    csv_log = None
    node = None
    exit_code = 0
    try:
        csv_log = CsvLogger(args.log_dir)
        print(f"Event 1 CSV log: {csv_log.path}")
        rclpy.init()
        node = Event1Controller(args, csv_log)
        if args.rotation_test:
            node.run_rotation_test()
        else:
            node.run_competition()
    except KeyboardInterrupt:
        exit_code = 130
        if node is not None:
            node.get_logger().warning("Run interrupted; stopping robot")
    except Exception as error:
        exit_code = 1
        message = str(error)
        if node is not None:
            node.get_logger().error(message)
            node.write_failure(message)
        else:
            print(f"Event 1 failed: {message}")
    finally:
        if node is not None:
            for _ in range(6):
                node.stop()
                rclpy.spin_once(node, timeout_sec=0.03)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if csv_log is not None:
            csv_log.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
