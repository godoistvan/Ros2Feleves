#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute
import math
import time

class KochSnowflake(Node):
    def __init__(self):
        super().__init__('koch_snowflake')

        # 1) create publisher for movement
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # 2) pen service client
        self.pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')

        # 3) teleport service client
        self.tel_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.tel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/teleport_absolute service...')

        # 4) lift pen so teleport doesn’t draw
        self.set_pen(0, 0, 0, 0, 1)
        time.sleep(0.1)

        # 5) teleport turtle to (3,3)
        ta_req = TeleportAbsolute.Request()
        ta_req.x = 3.0
        ta_req.y = 3.0
        ta_req.theta = 0.0
        self.tel_client.call_async(ta_req)
        time.sleep(0.1)

        # 6) drawing speeds
        self.linear_speed = 1.0              # units/sec
        self.angular_speed = math.radians(60)  # rad/sec

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_client.call_async(req)

    def move_forward(self, distance):
        twist = Twist()
        twist.linear.x = self.linear_speed
        duration = abs(distance / self.linear_speed)
        start = self.get_clock().now().nanoseconds
        rate = self.create_rate(100)
        while (self.get_clock().now().nanoseconds - start) < duration * 1e9:
            self.pub.publish(twist)
            rate.sleep()
        self.pub.publish(Twist())  # stop

    def rotate(self, angle_deg):
        twist = Twist()
        angle_rad = math.radians(angle_deg)
        twist.angular.z = self.angular_speed if angle_rad > 0 else -self.angular_speed
        duration = abs(angle_rad / self.angular_speed)
        start = self.get_clock().now().nanoseconds
        rate = self.create_rate(100)
        while (self.get_clock().now().nanoseconds - start) < duration * 1e9:
            self.pub.publish(twist)
            rate.sleep()
        self.pub.publish(Twist())  # stop

    def draw_koch(self, length, depth):
        if depth == 0:
            self.move_forward(length)
        else:
            self.draw_koch(length/3.0, depth-1)
            self.rotate(60)
            self.draw_koch(length/3.0, depth-1)
            self.rotate(-120)
            self.draw_koch(length/3.0, depth-1)
            self.rotate(60)
            self.draw_koch(length/3.0, depth-1)

    def run(self, side_length=4.0, depth=3):
        # lower the pen in blue
        self.set_pen(0, 0, 255, 2, 0)
        time.sleep(0.1)

        # draw the three sides of the snowflake
        for _ in range(3):
            self.draw_koch(side_length, depth)
            self.rotate(-120)

def main(args=None):
    rclpy.init(args=args)
    node = KochSnowflake()
    node.get_logger().info('Starting in 1 second…')
    time.sleep(1.0)
    node.run(side_length=4.0, depth=3)
    node.get_logger().info('Done drawing!')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
