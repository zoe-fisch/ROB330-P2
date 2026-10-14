"""ROS-independent geometry, control, safety, and logging helpers for Event 1."""

import csv
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z),
    )


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    expected_heading: float
    lap: int
    side: int


def make_square_waypoints(
    x0, y0, heading, side_length, direction, laps=4, sides=4
):
    if side_length <= 0:
        raise ValueError("side_length must be positive")
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 (clockwise) or 1 (counterclockwise)")
    if laps <= 0:
        raise ValueError("laps must be positive")
    if sides < 1 or sides > 4:
        raise ValueError("sides must be between 1 and 4")
    if sides < 4 and laps > 1:
        raise ValueError("partial-square routes may run only one lap")

    ux, uy = math.cos(heading), math.sin(heading)
    vx, vy = -math.sin(heading) * direction, math.cos(heading) * direction
    corners = ((1, 0), (1, 1), (0, 1), (0, 0))
    waypoints = []

    for lap in range(1, laps + 1):
        for side_index, (i, j) in enumerate(corners[:sides]):
            waypoints.append(
                Waypoint(
                    x=x0 + side_length * (i * ux + j * vx),
                    y=y0 + side_length * (i * uy + j * vy),
                    expected_heading=wrap(
                        heading + direction * side_index * math.pi / 2
                    ),
                    lap=lap,
                    side=side_index + 1,
                )
            )

    return waypoints


@dataclass(frozen=True)
class ControlConfig:
    speed: float = 0.10
    min_speed: float = 0.035
    max_turn: float = 0.50
    turn_gain: float = 1.8
    drive_gain: float = 1.4
    position_tolerance: float = 0.025
    turn_enter: float = math.radians(25.0)
    turn_exit: float = math.radians(8.0)
    slowdown_distance: float = 0.30


@dataclass(frozen=True)
class ControlDecision:
    state: str
    previous_state: str
    transition_reason: str
    target_heading: float
    heading_error: float
    distance: float
    along_track_error: float
    cross_track_error: float
    linear_velocity: float
    angular_velocity: float
    waypoint_reached: bool


class WaypointController:
    """Heading-hysteresis controller using the live bearing to one waypoint."""

    def __init__(self, config):
        if config.position_tolerance <= 0:
            raise ValueError("position_tolerance must be positive")
        if config.turn_exit < 0 or config.turn_enter <= config.turn_exit:
            raise ValueError("turn thresholds must satisfy 0 <= exit < enter")
        if config.speed <= 0 or config.min_speed <= 0:
            raise ValueError("speed limits must be positive")
        if config.min_speed > config.speed:
            raise ValueError("min_speed cannot exceed speed")
        if config.max_turn <= 0 or config.slowdown_distance <= 0:
            raise ValueError("turn limit and slowdown distance must be positive")

        self.config = config
        self.state = None

    def step(self, pose, waypoint):
        px, py, yaw = pose
        dx, dy = waypoint.x - px, waypoint.y - py
        distance = math.hypot(dx, dy)
        if distance > self.config.position_tolerance:
            target_heading = math.atan2(dy, dx)
        else:
            target_heading = waypoint.expected_heading
        heading_error = wrap(target_heading - yaw)

        along = (
            dx * math.cos(waypoint.expected_heading)
            + dy * math.sin(waypoint.expected_heading)
        )
        cross = (
            -dx * math.sin(waypoint.expected_heading)
            + dy * math.cos(waypoint.expected_heading)
        )

        previous = self.state or ""
        reason = ""

        if distance <= self.config.position_tolerance:
            self.state = "COMPLETE"
            if previous != "COMPLETE":
                reason = "within position tolerance"
            return ControlDecision(
                state=self.state,
                previous_state=previous,
                transition_reason=reason,
                target_heading=target_heading,
                heading_error=heading_error,
                distance=distance,
                along_track_error=along,
                cross_track_error=cross,
                linear_velocity=0.0,
                angular_velocity=0.0,
                waypoint_reached=True,
            )

        if self.state is None:
            self.state = (
                "TURN"
                if abs(heading_error) >= self.config.turn_enter
                else "DRIVE"
            )
            reason = "initial heading alignment" if self.state == "TURN" else "heading aligned"
        elif self.state == "TURN":
            if abs(heading_error) <= self.config.turn_exit:
                self.state = "DRIVE"
                reason = "heading entered turn-exit band"
        elif self.state == "DRIVE":
            if abs(heading_error) >= self.config.turn_enter:
                self.state = "TURN"
                reason = "heading exceeded turn-entry threshold"

        if self.state == "TURN":
            angular = max(
                -self.config.max_turn,
                min(self.config.max_turn, self.config.turn_gain * heading_error),
            )
            linear = 0.0
        else:
            slowdown = min(1.0, distance / self.config.slowdown_distance)
            linear = max(
                self.config.min_speed,
                min(self.config.speed, self.config.speed * slowdown),
            )
            linear *= max(0.3, math.cos(heading_error))
            angular = max(
                -self.config.max_turn,
                min(self.config.max_turn, self.config.drive_gain * heading_error),
            )

        return ControlDecision(
            state=self.state,
            previous_state=previous,
            transition_reason=reason,
            target_heading=target_heading,
            heading_error=heading_error,
            distance=distance,
            along_track_error=along,
            cross_track_error=cross,
            linear_velocity=linear,
            angular_velocity=angular,
            waypoint_reached=False,
        )


def safety_error(
    pose_age,
    scan_age,
    elapsed,
    closest_range,
    pose_timeout,
    scan_timeout,
    run_timeout,
    stop_distance,
):
    if pose_age is None or pose_age > pose_timeout:
        return "No fresh /estimated_pose"
    if scan_age is None or scan_age > scan_timeout:
        return "No fresh /scan"
    if elapsed > run_timeout:
        return "Run timeout exceeded"
    if closest_range < stop_distance:
        return "Lidar obstacle detected"
    return ""


def pose_jump(previous_pose, current_pose, max_translation, max_rotation):
    if previous_pose is None:
        return ""
    translation = math.hypot(
        current_pose[0] - previous_pose[0],
        current_pose[1] - previous_pose[1],
    )
    rotation = abs(wrap(current_pose[2] - previous_pose[2]))
    if translation > max_translation:
        return f"Estimated pose jumped {translation:.3f} m"
    if rotation > max_rotation:
        return f"Estimated heading jumped {math.degrees(rotation):.1f} deg"
    return ""


CSV_FIELDS = (
    "wall_time",
    "elapsed_s",
    "ros_time_s",
    "lap",
    "side",
    "state",
    "transition_reason",
    "event",
    "event_detail",
    "pose_frame",
    "estimated_x_m",
    "estimated_y_m",
    "estimated_yaw_rad",
    "estimated_yaw_deg",
    "start_x_m",
    "start_y_m",
    "start_yaw_rad",
    "pose_age_s",
    "waypoint_x_m",
    "waypoint_y_m",
    "expected_side_heading_rad",
    "target_heading_rad",
    "distance_m",
    "along_track_error_m",
    "cross_track_error_m",
    "heading_error_deg",
    "waypoint_reached",
    "command_linear_mps",
    "command_angular_rad_s",
    "command_angular_deg_s",
    "command_type",
    "min_lidar_range_m",
    "scan_age_s",
    "odom_age_s",
    "imu_age_s",
    "safety_stop",
    "error_message",
    "turn_start_yaw_rad",
    "turn_target_heading_rad",
    "turn_initial_error_deg",
    "turn_estimated_change_deg",
    "turn_estimated_end_yaw_rad",
    "turn_commanded_abs_displacement_deg",
    "turn_state_transitions",
    "side_state_transitions",
    "odom_frame",
    "odom_child_frame",
    "odom_x_m",
    "odom_y_m",
    "odom_yaw_rad",
    "odom_twist_angular_rad_s",
    "imu_angular_velocity_z_rad_s",
    "dry_run",
)


class CsvLogger:
    def __init__(self, directory):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path = Path(directory).expanduser() / f"event_1_{stamp}.csv"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.file.flush()

    def write(self, row):
        self.writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})
        self.file.flush()

    def close(self):
        if not self.file.closed:
            self.file.close()
