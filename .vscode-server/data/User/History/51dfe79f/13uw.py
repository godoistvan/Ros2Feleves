#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute


class KochSnowflake(Node):
    """Draw a Koch snowflake in turtlesim (ROS 2)"""

    def __init__(self):
        super().__init__('koch_snowflake')

        # ── publishers & service clients ───────────────────────────
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        self.tel_client = self.create_client(TeleportAbsolute,
                                             'turtle1/teleport_absolute')
        self._wait_for_services()

        # ── reusable timing helper ─────────────────────────────────
        self.rate_100 = self.create_rate(100)          # 100 Hz

        # ── motion parameters (override with ROS params if desired) ─
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_speed',      2.0),   # turtle units / s
                ('angular_speed_deg', 180.0)  # deg / s
            ]
        )
        self.linear_speed  = float(self.get_parameter('linear_speed').value)
        self.angular_speed = math.radians(
            float(self.get_parameter('angular_speed_deg').value)
        )

    # ──────────── service helpers ──────────────────────────────────
    def _wait_for_services(self):
        for client in (self.pen_client, self.tel_client):
            while not client.wait_for_service(1.0):
                self.get_logger().info(f'Waiting for {client.srv_name} …')

    def _call_sync(self, client, request):
        """Send a service request and block until the future completes."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def _set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width, req.off = r, g, b, width, off
        self._call_sync(self.pen_client, req)

    def _teleport(self, x, y, theta=0.0):
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = x, y, theta
        self._call_sync(self.tel_client, req)

    # ──────────── low-level motion primitives ─────────────────────
    def _publish_for_duration(self, twist, duration_s: float):
        start = self.get_clock().now()
        end   = start + rclpy.time.Duration(seconds=duration_s)
        while self.get_clock().now() < end:
            self.pub.publish(twist)
            self.rate_100.sleep()
        self.pub.publish(Twist())                     # stop

    def move_forward(self, distance):
        twist = Twist()
        twist.linear.x = self.linear_speed * (1 if distance >= 0 else -1)
        self._publish_for_duration(twist,
                                   abs(distance) / self.linear_speed)

    def rotate(self, angle_deg):
        angle_rad = math.radians(angle_deg)
        twist = Twist()
        twist.angular.z = self.angular_speed * (1 if angle_rad >= 0 else -1)
        self._publish_for_duration(twist,
                                   abs(angle_rad) / self.angular_speed)

    # ──────────── Koch recursion ──────────────────────────────────
    def draw_koch(self, length, depth):
        if depth == 0:
            self.move_forward(length)
        else:
            sub = length / 3.0
            self.draw_koch(sub, depth - 1)
            self.rotate(60)
            self.draw_koch(sub, depth - 1)
            self.rotate(-120)
            self.draw_koch(sub, depth - 1)
            self.rotate(60)
            self.draw_koch(sub, depth - 1)

    # ──────────── public entry point ───────────────────────────────
    def run(self, side_length=4.0, depth=3):
        # lift pen, teleport to start pose
        self._set_pen(0, 0, 0, 1, 1)          # pen off
        self._teleport(3.0, 3.0, 0.0)

        # lower pen (bright blue) and draw
        self._set_pen(0, 0, 255, 2, 0)
        for _ in range(3):
            self.draw_koch(side_length, depth)
            self.rotate(-120)


def main(args=None):
    rclpy.init(args=args)
    node = KochSnowflake()
    node.get_logger().info('Drawing Koch snowflake…')
    node.run(side_length=4.0, depth=3)
    node.get_logger().info('Done!')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
