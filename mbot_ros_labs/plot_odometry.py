#!/usr/bin/env python3

import argparse
import csv
import math
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def quaternion_to_yaw(q):
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


class OdometryLogger(Node):
    def __init__(self, csv_path):
        super().__init__("odometry_logger")

        self.csv_path = csv_path
        self.csv_file = open(csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            "time_s",
            "x_m",
            "y_m",
            "theta_rad",
            "linear_x_mps",
            "linear_y_mps",
            "angular_z_radps",
        ])
        self.csv_file.flush()

        self.start_time = self.get_clock().now()
        self.sample_count = 0
        self.x_values = []
        self.y_values = []

        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Recording /odom to {csv_path}")

    def odom_callback(self, msg):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1_000_000_000.0

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = quaternion_to_yaw(msg.pose.pose.orientation)

        self.writer.writerow([
            elapsed,
            x,
            y,
            theta,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        ])

        # Save each message immediately.
        self.csv_file.flush()

        self.x_values.append(x)
        self.y_values.append(y)
        self.sample_count += 1

        if self.sample_count % 100 == 0:
            self.get_logger().info(
                f"Recorded {self.sample_count} samples "
                f"(x={x:.3f}, y={y:.3f})"
            )

    def close_file(self):
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()


def save_plot(x_values, y_values, output_path, title):
    if not x_values:
        print("No odometry samples were received, so no plot was created.")
        return

    plt.figure(figsize=(7, 7))
    plt.plot(
        x_values,
        y_values,
        color="blue",
        linewidth=2,
        label="Odometry trajectory",
    )

    plt.scatter(
        x_values[0],
        y_values[0],
        color="green",
        s=80,
        marker="o",
        label="Start",
    )

    plt.scatter(
        x_values[-1],
        y_values[-1],
        color="red",
        s=80,
        marker="x",
        label="End",
    )

    plt.xlabel("x position [m]")
    plt.ylabel("y position [m]")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=200)
    plt.close()

    print(f"Saved trajectory plot: {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--name",
        default="odom_run",
        help="Name used for the CSV and PNG files",
    )
    parser.add_argument(
        "--title",
        default="Robot Odometry Trajectory",
    )
    args = parser.parse_args()

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = Path(f"{args.name}_{timestamp}.csv").resolve()
    plot_path = Path(f"{args.name}_{timestamp}.png").resolve()

    rclpy.init()
    node = OdometryLogger(csv_path)

    print("Recording /odom. Run the RTR controller now.")
    print("Press Control+C after the robot finishes.")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_file()

        x_values = node.x_values
        y_values = node.y_values
        sample_count = node.sample_count

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    print(f"Recorded {sample_count} samples.")
    print(f"Saved CSV: {csv_path}")

    save_plot(
        x_values,
        y_values,
        plot_path,
        args.title,
    )


if __name__ == "__main__":
    main()