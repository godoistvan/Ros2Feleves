#!/usr/bin/env python3
"""
Koch snowflake drawer for turtlesim.

Quick demo
==========

Terminal 1
----------
    ros2 run turtlesim turtlesim_node

Terminal 2
----------
    # Make sure to build your workspace after saving this file
    # cd ~/ros2_ws
    # colcon build --packages-select ros2_course
    # source install/setup.bash

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash # Adjust if your workspace name is different
    ros2 run ros2_course koch_snowflake --ros-args \
        -p order:=3 -p side_length:=5.0
"""

import math
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, TeleportRelative, SetPen
import sys # Added for sys.exit in the entry point if this file is the script itself

class KochSnowflake(Node):
    """Node that makes a turtle draw a Koch snowflake."""

    # ------------------------------------------------------------------ #
    #   Construction                                                     #
    # ------------------------------------------------------------------ #
    def __init__(self) -> None:
        super().__init__('koch_snowflake')

        # ---- parameters users can override with -p name:=value --------
        self.declare_parameter('turtle',        'turtle1')
        self.declare_parameter('order',         3)
        self.declare_parameter('side_length',   5.0)
        self.declare_parameter('start_x',       2.0) # Adjusted for better visibility
        self.declare_parameter('start_y',       7.0) # Adjusted for better visibility
        self.declare_parameter('theta',         0.0)   # radians
        self.declare_parameter('pen_r',         255)
        self.declare_parameter('pen_g',         0)
        self.declare_parameter('pen_b',         0)
        self.declare_parameter('pen_width',     2)

        # ---- read them -------------------------------------------------
        p = self.get_parameter
        self.turtle   = p('turtle').get_parameter_value().string_value
        self.order    = p('order').get_parameter_value().integer_value
        self.L        = p('side_length').get_parameter_value().double_value
        self.x0       = p('start_x').get_parameter_value().double_value
        self.y0       = p('start_y').get_parameter_value().double_value
        self.theta0   = p('theta').get_parameter_value().double_value
        self.pen_r    = p('pen_r').get_parameter_value().integer_value
        self.pen_g    = p('pen_g').get_parameter_value().integer_value
        self.pen_b    = p('pen_b').get_parameter_value().integer_value
        self.pen_w    = p('pen_width').get_parameter_value().integer_value

        # ---- service clients ------------------------------------------
        base = f'/{self.turtle}'
        self.abs_cli = self.create_client(TeleportAbsolute, f'{base}/teleport_absolute')
        self.rel_cli = self.create_client(TeleportRelative, f'{base}/teleport_relative')
        self.pen_cli = self.create_client(SetPen,           f'{base}/set_pen')

        # wait until every service is present
        clients_to_wait_for = [
            (self.abs_cli, f'{base}/teleport_absolute'),
            (self.rel_cli, f'{base}/teleport_relative'),
            (self.pen_cli, f'{base}/set_pen')
        ]
        for cli, service_name in clients_to_wait_for:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service {service_name}...')

        self.get_logger().info('All services are available. KochSnowflake node initialized.')

    # ------------------------------------------------------------------ #
    #   Internal helpers                                                 #
    # ------------------------------------------------------------------ #
    def _sync_call(self, client, request):
        """
        Send `request` asynchronously and spin *this* node until the
        future completes. Works on every rclpy distribution.
        """
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            # Spin self to process service client callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            # You could add a small sleep here if CPU usage is a concern,
            # but for turtlesim, responses are usually fast.
            # import time
            # time.sleep(0.01)
        if future.done():
            if future.exception() is not None:
                self.get_logger().error(f'Service call failed with exception: {future.exception()}')
                return None # Or raise an exception
            return future.result()
        else: # rclpy not ok
            self.get_logger().warn('RCLPY is not OK, service call cannot complete.')
            return None


    # ---------- wrappers around turtlesim services ---------------------
    def _pen(self, *, off: int = 0) -> None:
        req = SetPen.Request()
        req.r = self.pen_r
        req.g = self.pen_g
        req.b = self.pen_b
        req.width = self.pen_w
        req.off   = off # 0 for pen down, 1 for pen up
        self.get_logger().debug(f"Setting pen: r={req.r}, g={req.g}, b={req.b}, width={req.width}, off={req.off}")
        response = self._sync_call(self.pen_cli, req)
        if response is None:
            self.get_logger().error('Failed to set pen.')


    def _teleport_abs(self, x: float, y: float, theta: float) -> None:
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.get_logger().debug(f"Teleporting absolute: x={req.x}, y={req.y}, theta={req.theta}")
        response = self._sync_call(self.abs_cli, req)
        if response is None:
            self.get_logger().error('Failed to teleport absolute.')

    def _teleport_rel(self, dist: float, ang: float) -> None:
        req = TeleportRelative.Request()
        req.linear = dist
        req.angular = ang
        self.get_logger().debug(f"Teleporting relative: linear={req.linear}, angular={req.angular}")
        response = self._sync_call(self.rel_cli, req)
        if response is None:
            self.get_logger().error('Failed to teleport relative.')

    # ---------- convenience -------------------------------------------
    def turn(self, angle_rad: float) -> None:
        self.get_logger().debug(f"Turning by {math.degrees(angle_rad)} degrees.")
        self._teleport_rel(0.0, angle_rad)

    def forward(self, dist: float) -> None:
        self.get_logger().debug(f"Moving forward by {dist} units.")
        self._teleport_rel(dist, 0.0)

    # ---------- Koch recursion ----------------------------------------
    def koch(self, length: float, order: int) -> None:
        if not rclpy.ok(): return # Exit recursion if ROS is shutting down

        if order == 0:
            self.forward(length)
            return

        new_length = length / 3.0
        self.koch(new_length, order - 1)
        self.turn(math.radians(60))
        self.koch(new_length, order - 1)
        self.turn(math.radians(-120))
        self.koch(new_length, order - 1)
        self.turn(math.radians(60))
        self.koch(new_length, order - 1)

    # ------------------------------------------------------------------ #
    #   Main drawing routine                                             #
    # ------------------------------------------------------------------ #
    def draw(self) -> None:
        self.get_logger().info(f"Starting Koch snowflake: order={self.order}, side_length={self.L}")
        self.get_logger().info(f"Initial turtle: {self.turtle}, position: ({self.x0}, {self.y0}, {self.theta0})")

        # Lift pen and go to starting position
        self._pen(off=1)
        self._teleport_abs(self.x0, self.y0, self.theta0)

        # Pen down to start drawing
        self._pen(off=0)

        # Draw the three sides of the snowflake
        for i in range(3):
            if not rclpy.ok():
                self.get_logger().info('ROS shutdown requested, stopping draw.')
                break
            self.get_logger().info(f"Drawing side {i+1} of 3...")
            self.koch(self.L, self.order)
            self.turn(math.radians(-120)) # Turn for the next side of the snowflake

        if rclpy.ok():
            self.get_logger().info('🎉 Finished drawing Koch snowflake')
        else:
            self.get_logger().info('Drawing interrupted due to ROS shutdown.')
        
        # Lift pen after drawing
        self._pen(off=1)


# ---------------------------------------------------------------------- #
def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        koch_node = KochSnowflake()
        koch_node.draw()
    except KeyboardInterrupt:
        koch_node.get_logger().info('KeyboardInterrupt, shutting down.')
    except Exception as e:
        if 'koch_node' in locals() and hasattr(koch_node, 'get_logger'):
            koch_node.get_logger().error(f"An error occurred: {e}")
        else:
            print(f"An error occurred before logger was available: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
            if 'koch_node' in locals() and hasattr(koch_node, 'get_logger'):
                 koch_node.get_logger().info('ROS2 shutdown complete.')
            else:
                print('ROS2 shutdown complete.')

if __name__ == '__main__':
    # This allows the script to be run directly,
    # but usually it's launched via a ROS 2 entry point from setup.py
    main()