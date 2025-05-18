#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import math
import time

class KochSnowflake(Node):
    def __init__(self):
        super().__init__('koch_snowflake')
        # publisher to move the turtle
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # client to lift/lower pen
        self.pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')

        # drawing parameters
        self.linear_speed = 1.0    # units/sec
        self.angular_speed = math.radians(60)  # rad/sec

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r, req.g, req.b = r, g, b
        req.width = width
        req.off = off
        self.pen_client.call_async(req)

    def move_forward(self, distance):
        """Move straight by publishing a Twist for the required duration."""
        twist = Twist()
        twist.linear.x = self.linear_speed
        duration = abs(distance / self.linear_speed)
        end_time = self.get_clock().now().to_msg().sec + duration
        t0 = self.get_clock().now().nanoseconds
        # publish for duration
        rate = self.create_rate(100)
        start = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start) < duration * 1e9:
            self.pub.publish(twist)
            rate.sleep()
        # stop
        self.pub.publish(Twist())

    def rotate(self, angle_deg):
        """Rotate in place by angle_deg (positive = left)."""
        twist = Twist()
        angle_rad = math.radians(angle_deg)
        twist.angular.z = self.angular_speed if angle_rad > 0 else -self.angular_speed
        duration = abs(angle_rad / self.angular_speed)
        start = self.get_clock().now().nanoseconds
        rate = self.create_rate(100)
        while (self.get_clock().now().nanoseconds - start) < duration * 1e9:
            self.pub.publish(twist)
            rate.sleep()
        self.pub.publish(Twist())

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
        # ensure pen is down
        self.set_pen(0, 0, 255, 2, 0)
        time.sleep(0.1)  # allow service to complete

        # draw three sides
        for _ in range(3):
            self.draw_koch(side_length, depth)
            self.rotate(-120)

def main(args=None):
    rclpy.init(args=args)
    node = KochSnowflake()
    node.get_logger().info('Starting in 1 second...')
    time.sleep(1.0)
    node.run(side_length=6.0, depth=4)
    node.get_logger().info('Done drawing!')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
