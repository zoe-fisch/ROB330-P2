#!/usr/bin/env python3
"""Record and compare /odom and /gyrodom.

Run while the robot drives, then press Ctrl+C. The script saves a CSV and a
PNG containing the XY trajectories and x, y, and heading histories.
"""

import argparse
import csv
import math
import os
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def extract_planar_pose(msg):
    """Return (x, y, theta) for standard or MBot-style odometry messages."""
    if hasattr(msg, "pose"):
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        if hasattr(pose, "position") and hasattr(pose, "orientation"):
            return (
                float(pose.position.x),
                float(pose.position.y),
                quaternion_to_yaw(pose.orientation),
            )
        if all(hasattr(pose, field) for field in ("x", "y", "theta")):
            return float(pose.x), float(pose.y), float(pose.theta)

    if all(hasattr(msg, field) for field in ("x", "y", "theta")):
        return float(msg.x), float(msg.y), float(msg.theta)

    raise TypeError(
        f"Unsupported message layout: {type(msg).__module__}.{type(msg).__name__}"
    )


class OdometryRecorder(Node):
    def __init__(self, timeout):
        super().__init__("odom_gyrodom_plotter")
        self.timeout = timeout
        self.started_at = time.monotonic()
        self.data = {"odom": [], "gyrodom": []}
        self.subscriptions_ready = False
        self.timer = self.create_timer(0.25, self.discover_topics)

    def discover_topics(self):
        if self.subscriptions_ready:
            return

        available = dict(self.get_topic_names_and_types())
        missing = [name for name in ("/odom", "/gyrodom") if name not in available]
        if missing:
            if time.monotonic() - self.started_at > self.timeout:
                self.get_logger().error(
                    "Timed out waiting for topics: " + ", ".join(missing)
                )
                raise RuntimeError("Required odometry topics were not found")
            return

        for topic, label in (("/odom", "odom"), ("/gyrodom", "gyrodom")):
            type_name = available[topic][0]
            msg_type = get_message(type_name)
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, name=label: self.record(name, msg),
                20,
            )
            self.get_logger().info(f"Subscribed to {topic} [{type_name}]")

        self.subscriptions_ready = True
        self.destroy_timer(self.timer)

    def record(self, name, msg):
        try:
            x, y, theta = extract_planar_pose(msg)
        except TypeError as exc:
            self.get_logger().error(str(exc))
            raise

        t = time.monotonic() - self.started_at
        self.data[name].append((t, x, y, theta))


def save_csv(data, path):
    with open(path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["method", "time_s", "x_m", "y_m", "theta_rad"])
        for method in ("odom", "gyrodom"):
            for row in data[method]:
                writer.writerow([method, *row])


def closure_error(samples):
    values = np.asarray(samples, dtype=float)
    dx = values[-1, 1] - values[0, 1]
    dy = values[-1, 2] - values[0, 2]
    dtheta = math.atan2(
        math.sin(values[-1, 3] - values[0, 3]),
        math.cos(values[-1, 3] - values[0, 3]),
    )
    return math.hypot(dx, dy), abs(math.degrees(dtheta))


def save_plot(data, path):
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    colors = {"odom": "tab:blue", "gyrodom": "tab:orange"}

    for method in ("odom", "gyrodom"):
        values = np.asarray(data[method], dtype=float)
        t = values[:, 0] - values[0, 0]
        x = values[:, 1]
        y = values[:, 2]
        yaw_deg = np.degrees(np.unwrap(values[:, 3]))

        axes[0, 0].plot(x, y, label=method, color=colors[method])
        axes[0, 0].scatter(x[0], y[0], color=colors[method], marker="o", s=30)
        axes[0, 0].scatter(x[-1], y[-1], color=colors[method], marker="x", s=50)
        axes[0, 1].plot(t, x, label=method, color=colors[method])
        axes[1, 0].plot(t, y, label=method, color=colors[method])
        axes[1, 1].plot(t, yaw_deg, label=method, color=colors[method])

    axes[0, 0].set_title("XY trajectory (circle=start, x=end)")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    axes[0, 1].set_title("x position")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("x [m]")
    axes[1, 0].set_title("y position")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("y [m]")
    axes[1, 1].set_title("heading")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("heading [deg]")

    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
        axis.legend()

    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="odom_comparison")
    parser.add_argument("--topic-timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = OdometryRecorder(args.topic_timeout)
    print("Recording /odom and /gyrodom. Drive the test path, then press Ctrl+C.")

    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        data = node.data
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not all(data[name] for name in ("odom", "gyrodom")):
        print("No plot created because one or both topics had no samples.", file=sys.stderr)
        raise SystemExit(1)

    os.makedirs(args.output_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.output_dir, f"odom_gyrodom_{stamp}.csv")
    png_path = os.path.join(args.output_dir, f"odom_gyrodom_{stamp}.png")
    save_csv(data, csv_path)
    save_plot(data, png_path)

    print(f"Saved data: {csv_path}")
    print(f"Saved plot: {png_path}")
    for method in ("odom", "gyrodom"):
        distance_error, heading_error = closure_error(data[method])
        print(
            f"{method}: {len(data[method])} samples, "
            f"return-to-start error={distance_error:.4f} m, "
            f"heading error={heading_error:.2f} deg"
        )

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
