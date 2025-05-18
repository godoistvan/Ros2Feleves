#!/usr/bin/env python3
"""
Draw a Koch snowflake with turtlesim.

Run --help for parameters, e.g.:

    ros2 run turtlesim turtlesim_node   # in one terminal
    ros2 run ros2_course koch_snowflake --ros-args -p order:=3 \
           -p side_length:=5.0 -p start_x:=3.0 -p start_y:=8.0  # another terminal
"""

import math
import rclpy
from rclpy.node import Node

from turtlesim.srv import TeleportAbsolute, TeleportRelative, SetPen


class KochSnowflake(Node):
    """Simple synchronous node that draws a Koch snowflake."""

    def __init__(self) -> None:
        super().__init__('koch_snowflake')

        # -------- parameters -------------------------------------------------
        self.declare_parameter('order',         3)     # recursion depth
        self.declare_parameter('side_length',   5.0)   # length of outer-triangle edge
        self.declare_parameter('start_x',       3.0)   # starting pose (must fit window)
        self.declare_parameter('start_y',       8.0)
        self.declare_parameter('theta',         0.0)   # initial heading in *radians*
        self.declare_parameter('pen_r',       255)     # pen colour & width
        self.declare_parameter('pen_g',         0)
        self.declare_parameter('pen_b',         0)
        self.declare_parameter('pen_width',     2)

        self.n      = int  (self.get_parameter('order'       ).value)
        self.L      = float(self.get_parameter('side_length' ).value)
        self.x0     = float(self.get_parameter('start_x'     ).value)
        self.y0     = float(self.get_parameter('start_y'     ).value)
        self.theta0 = float(self.get_parameter('theta'       ).value)

        self.pen_r  = int  (self.get_parameter('pen_r'       ).value)
        self.pen_g  = int  (self.get_parameter('pen_g'       ).value)
        self.pen_b  = int  (self.get_parameter('pen_b'       ).value)
        self.pen_w  = int  (self.get_parameter('pen_width'   ).value)

        # -------- service clients --------------------------------------------
        self.abs_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.rel_cli = self.create_client(TeleportRelative, '/turtle1/teleport_relative')
        self.pen_cli = self.create_client(SetPen,           '/turtle1/set_pen')

        for cli in (self.abs_cli, self.rel_cli, self.pen_cli):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'waiting for {cli.srv_name} ...')

    # ---- thin wrappers around turtlesim services ---------------------------

    def _pen(self, *, off: int = 0) -> None:
        """Enable/disable pen and/or set colour/width."""
        req = SetPen.Request()
        req.r, req.g, req.b = self.pen_r, self.pen_g, self.pen_b
        req.width = self.pen_w
        req.off   = off
        self.pen_cli.call(req)

    def _teleport_abs(self, x: float, y: float, theta: float) -> None:
        req        = TeleportAbsolute.Request()
        req.x, req.y, req.theta = x, y, theta
        self.abs_cli.call(req)

    def _teleport_rel(self, dist: float, ang: float) -> None:
        req        = TeleportRelative.Request()
        req.linear = dist
        req.angular = ang
        self.rel_cli.call(req)

    # ---- turtle convenience helpers ---------------------------------------

    def turn(self, angle_rad: float) -> None:
        self._teleport_rel(0.0, angle_rad)

    def forward(self, dist: float) -> None:
        self._teleport_rel(dist, 0.0)

    # ---- Koch-curve recursion ---------------------------------------------

    def koch(self, length: float, order: int) -> None:
        if order == 0:
            self.forward(length)
            return

        self.koch(length / 3.0, order - 1)
        self.turn( math.radians( 60))
        self.koch(length / 3.0, order - 1)
        self.turn( math.radians(-120))
        self.koch(length / 3.0, order - 1)
        self.turn( math.radians( 60))
        self.koch(length / 3.0, order - 1)

    # ---- public entry point ------------------------------------------------

    def draw(self) -> None:
        # move to starting pose with pen up
        self._pen(off=1)
        self._teleport_abs(self.x0, self.y0, self.theta0)

        # pen down and draw the three sides
        self._pen(off=0)
        for _ in range(3):
            self.koch(self.L, self.n)
            self.turn(math.radians(-120))

        self.get_logger().info('🎉 Finished drawing Koch snowflake')
        # node can safely shut down once everything is transmitted
        rclpy.shutdown()


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    KochSnowflake().draw()
