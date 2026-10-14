
#!/usr/bin/env python3
"""ROB 330 Event 2: frontier exploration and return home."""

import argparse
import heapq
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y*q.y + q.z*q.z)
    )


class Explorer(Node):
    def __init__(self, args):
        super().__init__('event_2_explorer')
        self.args = args
        self.pose = None
        self.pose_time = 0
        self.scan_time = 0
        self.grid = None
        self.map_time = 0
        self.closest = float('inf')
        self.blocked_goals = set()
        self.home = None

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
        self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10
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
        rs = [
            r for r in m.ranges
            if math.isfinite(r)
            and m.range_min <= r <= m.range_max
        ]
        self.closest = min(rs, default=float('inf'))

    def map_cb(self, m):
        self.grid = m
        self.map_time = time.monotonic()

    def stop(self):
        self.pub.publish(Twist())

    def cmd(self, v=0, w=0):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def tick(self):
        rclpy.spin_once(self, timeout_sec=.05)
        t = time.monotonic()

        if self.pose is None or t-self.pose_time > .8:
            raise RuntimeError('Lost SLAM pose')
        if t-self.scan_time > .8:
            raise RuntimeError('Lost lidar scan')
        if self.grid is None or t-self.map_time > 3.:
            raise RuntimeError('Lost occupancy map')
        if t-self.t0 > self.args.timeout:
            raise RuntimeError('Competition time exceeded')

    def cell(self, x, y):
        m = self.grid
        return (
            math.floor(
                (x-m.info.origin.position.x)/m.info.resolution
            ),
            math.floor(
                (y-m.info.origin.position.y)/m.info.resolution
            )
        )

    def point(self, c):
        m = self.grid
        return (
            m.info.origin.position.x
            + (c[0]+.5)*m.info.resolution,
            m.info.origin.position.y
            + (c[1]+.5)*m.info.resolution
        )

    def inflated_free(self):
        m = self.grid
        w = m.info.width
        h = m.info.height
        data = m.data

        radius = math.ceil(
            self.args.clearance/m.info.resolution
        )
        blocked = set()

        for i, val in enumerate(data):
            if val < 0 or val >= self.args.occupied_threshold:
                x = i % w
                y = i // w

                for dy in range(-radius, radius+1):
                    for dx in range(-radius, radius+1):
                        if dx*dx + dy*dy <= radius*radius:
                            xx = x+dx
                            yy = y+dy
                            if 0 <= xx < w and 0 <= yy < h:
                                blocked.add((xx, yy))

        free = {
            (i % w, i // w)
            for i, val in enumerate(data)
            if 0 <= val < self.args.occupied_threshold
            and (i % w, i // w) not in blocked
        }
        return free

    def nearest_free(self, c, free):
        if c in free:
            return c

        for r in range(1, 12):
            options = [
                (x, y)
                for x in range(c[0]-r, c[0]+r+1)
                for y in range(c[1]-r, c[1]+r+1)
                if (x, y) in free
            ]
            if options:
                return min(
                    options,
                    key=lambda p:
                    (p[0]-c[0])**2 + (p[1]-c[1])**2
                )
        return None

    def path(self, start, goal, free):
        if start not in free or goal not in free:
            return None

        heap = [(0, start)]
        cost = {start: 0}
        parent = {}

        while heap:
            _, u = heapq.heappop(heap)

            if u == goal:
                p = [u]
                while p[-1] != start:
                    p.append(parent[p[-1]])
                return p[::-1]

            neighbors = (
                (u[0]+1, u[1]),
                (u[0]-1, u[1]),
                (u[0], u[1]+1),
                (u[0], u[1]-1)
            )

            for v in neighbors:
                if v not in free:
                    continue

                ng = cost[u]+1

                if ng < cost.get(v, float('inf')):
                    cost[v] = ng
                    parent[v] = u
                    h = (
                        abs(v[0]-goal[0])
                        + abs(v[1]-goal[1])
                    )
                    heapq.heappush(heap, (ng+h, v))
        return None

    def find_frontier(self, free, cur):
        m = self.grid
        w = m.info.width
        h = m.info.height
        frontier = []

        reach = (
            math.ceil(
                self.args.clearance/m.info.resolution
            ) + 2
        )

        for x, y in free:
            if (x, y) in self.blocked_goals:
                continue

            neighbors = (
                (x+reach, y),
                (x-reach, y),
                (x, y+reach),
                (x, y-reach)
            )

            if any(
                0 <= nx < w
                and 0 <= ny < h
                and m.data[ny*w+nx] < 0
                for nx, ny in neighbors
            ):
                frontier.append((x, y))

        frontier.sort(
            key=lambda c:
            (c[0]-cur[0])**2 + (c[1]-cur[1])**2
        )

        for goal in frontier[:350]:
            p = self.path(cur, goal, free)
            if p and len(p) > 2:
                return goal, p

        return None, None

    def follow(self, path):
        m = self.grid
        stride = max(
            1, round(.30/m.info.resolution)
        )

        selected = path[
            1:min(len(path), 1+stride):
            max(1, stride//3)
        ]

        if not selected:
            selected = [path[-1]]

        for c in selected:
            xg, yg = self.point(c)
            start = time.monotonic()

            while rclpy.ok():
                self.tick()

                if time.monotonic()-start > 14:
                    self.stop()
                    return False

                x, y, th = self.pose
                d = math.hypot(xg-x, yg-y)

                if d < max(.08, 1.7*m.info.resolution):
                    self.stop()
                    break

                ang = wrap(
                    math.atan2(yg-y, xg-x)-th
                )

                if abs(ang) > .25:
                    self.cmd(
                        0,
                        max(
                            -self.args.turn,
                            min(self.args.turn, 1.6*ang)
                        )
                    )
                else:
                    if self.closest < self.args.emergency_stop:
                        self.stop()
                        return False

                    self.cmd(
                        min(
                            self.args.speed,
                            max(.05, .8*d)
                        ),
                        max(
                            -self.args.turn,
                            min(self.args.turn, 1.5*ang)
                        )
                    )
        return True

    def home_heading(self):
        target = self.home[2]
        start = time.monotonic()

        while rclpy.ok():
            self.tick()

            if time.monotonic()-start > 18:
                return

            e = wrap(target-self.pose[2])

            if abs(e) < .09:
                self.stop()
                return

            self.cmd(
                0,
                max(
                    -self.args.turn,
                    min(self.args.turn, 1.7*e)
                )
            )

    def run(self):
        self.t0 = time.monotonic()
        self.get_logger().info(
            'Waiting for pose, map and scan...'
        )

        while rclpy.ok() and (
            self.pose is None
            or self.grid is None
            or not self.scan_time
        ):
            rclpy.spin_once(self, timeout_sec=.1)

            if time.monotonic()-self.t0 > 20:
                raise RuntimeError('Missing required ROS topics')

        self.home = self.pose
        self.get_logger().info(
            f'HOME: {self.home}'
        )

        phase = 'explore'
        failed = 0

        while rclpy.ok():
            self.tick()

            remaining = (
                self.args.timeout
                - (time.monotonic()-self.t0)
            )

            if remaining < self.args.return_reserve:
                phase = 'return'

            free = self.inflated_free()

            cur = self.nearest_free(
                self.cell(*self.pose[:2]), free
            )

            if cur is None:
                raise RuntimeError(
                    'Robot not in mapped safe space'
                )

            if phase == 'return':
                d = math.hypot(
                    self.pose[0]-self.home[0],
                    self.pose[1]-self.home[1]
                )

                if d < self.args.home_tolerance:
                    self.stop()
                    self.home_heading()
                    self.get_logger().info(
                        'RETURNED to start (estimated pose)'
                    )
                    return

                goal = self.nearest_free(
                    self.cell(*self.home[:2]), free
                )

                if goal is None:
                    raise RuntimeError(
                        'Home not in known safe map'
                    )

                p = self.path(cur, goal, free)

                if not p:
                    raise RuntimeError(
                        'No known route home'
                    )

            else:
                goal, p = self.find_frontier(free, cur)

                if p is None:
                    self.get_logger().info(
                        'No reachable frontiers; returning'
                    )
                    phase = 'return'
                    continue

            if len(p) < 2:
                self.stop()
                if phase == 'explore':
                    self.blocked_goals.add(goal)
                    continue
                raise RuntimeError(
                    'Near home cell but outside tolerance'
                )

            ok = self.follow(p)

            if not ok:
                failed += 1
                if phase == 'explore':
                    self.blocked_goals.add(goal)

                if failed > 18:
                    raise RuntimeError(
                        'Repeated path obstruction'
                    )
            else:
                failed = 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--speed', type=float, default=.13)
    p.add_argument('--turn', type=float, default=.65)
    p.add_argument('--clearance', type=float, default=.16)
    p.add_argument('--emergency-stop', type=float, default=.21)
    p.add_argument('--occupied-threshold', type=int, default=60)
    p.add_argument('--timeout', type=float, default=240)
    p.add_argument('--return-reserve', type=float, default=90)
    p.add_argument('--home-tolerance', type=float, default=.12)
    args = p.parse_args()

    rclpy.init()
    n = Explorer(args)

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
