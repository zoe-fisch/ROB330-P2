
#!/usr/bin/env python3
"""ROB 330 Event 1: four autonomous square laps."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class Looper(Node):
    def __init__(self, args):
        super().__init__('event_1_looper')
        self.args = args
        self.pose = None
        self.pose_time = 0.0
        self.scan_time = 0.0
        self.closest = float('inf')

        self.pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/estimated_pose', self.pose_cb, 10
        )
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10
        )
        self.freeze_client = self.create_client(
            SetBool, '/slam/freeze_mapping'
        )

    def pose_cb(self, m):
        p = m.pose.pose
        self.pose = (
            p.position.x,
            p.position.y,
            yaw(p.orientation)
        )
        self.pose_time = time.monotonic()

    def scan_cb(self, m):
        self.scan_time = time.monotonic()
        valid = [
            r for r in m.ranges
            if math.isfinite(r)
            and m.range_min <= r <= m.range_max
        ]
        self.closest = min(valid, default=float('inf'))

    def stop(self):
        self.pub.publish(Twist())

    def send(self, v=0., w=0.):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        now = time.monotonic()

        if self.pose is None or now - self.pose_time > 0.7:
            raise RuntimeError('No fresh /estimated_pose')

        if now - self.scan_time > 0.7:
            raise RuntimeError('No fresh /scan')

        if now - self.start_time > self.args.timeout:
            raise RuntimeError('Run timeout exceeded')

    def go(self, x, y, heading, label):
        self.get_logger().info(label)
        stage = 'turn'
        phase_start = time.monotonic()

        while rclpy.ok():
            self.tick()

            if time.monotonic() - phase_start > self.args.segment_timeout:
                raise RuntimeError('Waypoint timeout: ' + label)

            px, py, th = self.pose
            d = math.hypot(x - px, y - py)
            err = wrap(heading - th)

            if stage == 'turn':
                if abs(err) < self.args.angle_tolerance:
                    self.stop()
                    stage = 'drive'
                else:
                    self.send(
                        0.,
                        max(-self.args.max_turn,
                            min(self.args.max_turn, 2.0 * err))
                    )
                continue

            if d < self.args.position_tolerance:
                self.stop()
                return

            along = (
                (x - px) * math.cos(heading)
                + (y - py) * math.sin(heading)
            )
            lateral = (
                -(x - px) * math.sin(heading)
                + (y - py) * math.cos(heading)
            )

            steer = wrap(
                heading
                + math.atan2(1.3 * lateral, max(.08, abs(along)))
                - th
            )

            if abs(err) > 0.5 or abs(steer) > 0.8:
                stage = 'turn'
                continue

            if self.closest < self.args.stop_distance:
                raise RuntimeError('Lidar obstacle detected')

            if along < -self.args.position_tolerance:
                raise RuntimeError('Overshot waypoint')

            v = min(
                self.args.speed,
                max(.045, .85 * max(0., along))
            )

            self.send(
                v,
                max(-self.args.max_turn,
                    min(self.args.max_turn, 2.0 * steer))
            )

    def rotate(self, target):
        t0 = time.monotonic()

        while rclpy.ok():
            self.tick()

            if time.monotonic() - t0 > 20:
                raise RuntimeError('Final rotation timeout')

            e = wrap(target - self.pose[2])

            if abs(e) < self.args.angle_tolerance:
                self.stop()
                return

            self.send(
                0.,
                max(-self.args.max_turn,
                    min(self.args.max_turn, 2.0 * e))
            )

    def freeze(self):
        if not self.freeze_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'Freeze service unavailable; continuing live mapping'
            )
            return

        req = SetBool.Request()
        req.data = True
        future = self.freeze_client.call_async(req)

        rclpy.spin_until_future_complete(
            self, future, timeout_sec=3.0
        )

        if (
            not future.done()
            or future.result() is None
            or not future.result().success
        ):
            self.get_logger().warn('Map freeze not confirmed')
        else:
            self.get_logger().info(
                'Map frozen; localization continues'
            )

    def run(self):
        self.start_time = time.monotonic()
        self.get_logger().info(
            'Waiting for SLAM pose and lidar...'
        )

        while rclpy.ok() and (
            self.pose is None or self.scan_time == 0
        ):
            rclpy.spin_once(self, timeout_sec=.1)
            if time.monotonic() - self.start_time > 15:
                raise RuntimeError('Missing pose or scan')

        x0, y0, h0 = self.pose
        a = self.args.side
        direction = 1 if self.args.ccw else -1

        ux, uy = math.cos(h0), math.sin(h0)
        vx, vy = (
            -math.sin(h0) * direction,
            math.cos(h0) * direction
        )

        def pt(i, j):
            return (
                x0 + a * (i * ux + j * vx),
                y0 + a * (i * uy + j * vy)
            )

        corners = [(1, 0), (1, 1), (0, 1), (0, 0)]

        for lap in range(4):
            for k, (i, j) in enumerate(corners):
                heading = wrap(
                    h0 + direction * k * math.pi / 2
                )
                x, y = pt(i, j)
                self.go(
                    x, y, heading,
                    f'Lap {lap+1}/4, side {k+1}/4'
                )

            if lap == 0 and self.args.freeze_after_first:
                self.freeze()

        self.rotate(h0)

        position_error = math.hypot(
            self.pose[0] - x0,
            self.pose[1] - y0
        )
        heading_error = math.degrees(
            abs(wrap(self.pose[2] - h0))
        )

        self.get_logger().info(
            f'FINISHED: position error {position_error:.3f} m, '
            f'heading error {heading_error:.1f} deg'
        )


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--side', type=float, default=1.0)
    p.add_argument('--speed', type=float, default=.12)
    p.add_argument('--max-turn', type=float, default=.65)
    p.add_argument('--position-tolerance', type=float, default=.025)
    p.add_argument('--angle-tolerance', type=float, default=.055)
    p.add_argument('--stop-distance', type=float, default=.20)
    p.add_argument('--timeout', type=float, default=285.)
    p.add_argument('--segment-timeout', type=float, default=35.)
    p.add_argument('--ccw', action='store_true')
    p.add_argument('--freeze-after-first', action='store_true')
    args = p.parse_args()

    rclpy.init()
    n = Looper(args)

    try:
        n.run()
    except (KeyboardInterrupt, Exception) as e:
        n.get_logger().error(str(e))
    finally:
        for _ in range(6):
            n.stop()
            rclpy.spin_once(n, timeout_sec=.03)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
