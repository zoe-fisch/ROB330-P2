# WRITTEN WITH AI - " Python script in the python-tests folder to log data from the cmd_vel and odom ROS topics"

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class VelocityLogger(Node):
    def __init__(self):
        super().__init__('velocity_logger')

        # Subscribe to commanded velocity
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Subscribe to actual velocity from odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Starting time
        self.start_time = self.get_clock().now()

        # Commanded velocity data
        self.cmd_times = []
        self.cmd_linear = []
        self.cmd_angular = []

        # Actual velocity data
        self.odom_times = []
        self.actual_linear = []
        self.actual_angular = []

        self.get_logger().info('Velocity logger started.')
        self.get_logger().info('Listening to /cmd_vel and /odom...')
        self.get_logger().info('Press Ctrl+C when the test is finished.')


    def get_elapsed_time(self):
        """Return time in seconds since the logger started."""
        now = self.get_clock().now()
        elapsed = now - self.start_time
        return elapsed.nanoseconds / 1e9


    def cmd_callback(self, msg):
        """Store commanded velocity data."""
        time = self.get_elapsed_time()

        self.cmd_times.append(time)
        self.cmd_linear.append(msg.linear.x)
        self.cmd_angular.append(msg.angular.z)


    def odom_callback(self, msg):
        """Store actual velocity data from odometry."""
        time = self.get_elapsed_time()

        self.odom_times.append(time)
        self.actual_linear.append(msg.twist.twist.linear.x)
        self.actual_angular.append(msg.twist.twist.angular.z)


def plot_data(node):
    """Create and save commanded vs actual velocity plots."""

    print()
    print("Recorded:")
    print(f"  /cmd_vel messages: {len(node.cmd_times)}")
    print(f"  /odom messages:    {len(node.odom_times)}")
    print()

    if len(node.cmd_times) == 0:
        print("No /cmd_vel data was recorded.")
        return

    if len(node.odom_times) == 0:
        print("No /odom data was recorded.")
        return

    # Save plots in the same directory as this Python script
    output_dir = os.path.dirname(os.path.abspath(__file__))

    linear_file = os.path.join(
        output_dir,
        "linear_velocity.png"
    )

    angular_file = os.path.join(
        output_dir,
        "angular_velocity.png"
    )


    # -----------------------------------------
    # Linear velocity plot
    # -----------------------------------------

    plt.figure(figsize=(10, 6))

    plt.plot(
        node.cmd_times,
        node.cmd_linear,
        label='Commanded Linear Velocity'
    )

    plt.plot(
        node.odom_times,
        node.actual_linear,
        label='Actual Linear Velocity'
    )

    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.title('Commanded vs Actual Linear Velocity')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(
        linear_file,
        dpi=300,
        bbox_inches='tight'
    )

    plt.close()


    # -----------------------------------------
    # Angular velocity plot
    # -----------------------------------------

    plt.figure(figsize=(10, 6))

    plt.plot(
        node.cmd_times,
        node.cmd_angular,
        label='Commanded Angular Velocity'
    )

    plt.plot(
        node.odom_times,
        node.actual_angular,
        label='Actual Angular Velocity'
    )

    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Commanded vs Actual Angular Velocity')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(
        angular_file,
        dpi=300,
        bbox_inches='tight'
    )

    plt.close()


    print("Plots saved successfully!")
    print()
    print(f"Linear velocity plot:")
    print(linear_file)
    print()
    print(f"Angular velocity plot:")
    print(angular_file)


def main(args=None):

    rclpy.init(args=args)

    node = VelocityLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\nTest finished.')
        print('Generating plots...')

    finally:
        plot_data(node)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()