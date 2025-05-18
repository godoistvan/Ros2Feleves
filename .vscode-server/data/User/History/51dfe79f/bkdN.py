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
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run ros2_course koch_snowflake --ros-args \
        -p order:=4 -p side_length:=6.0
"""

import math
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, TeleportRelative, SetPen


class KochSnowflake(Node):
    """Node that makes a turtle draw a Koch snowflake."""

    # ------------------------------------------------------------------ #
    #  Construction                                                      #
    # ------------------------------------------------------------------ #
    def __init__(self) -> None:
        super().__init__('koch_snowflake')

        # ---- parameters users can override with -p name:=value --------
        self.declare_parameter('turtle',       'turtle1')
        self.declare_parameter('order',        3)
        self.declare_parameter('side_length',  5.0)
        self.declare_parameter('start_x',      3.0)
        self.declare_parameter('start_y',      8.0)
        self.declare_parameter('theta',        0.0)      # radians
        self.declare_parameter('pen_r',        255)
        self.declare_parameter('pen_g',          0)
        self.declare_parameter('pen_b',          0)
        self.declare_parameter('pen_width',      2)

        # ---- read them -------------------------------------------------
        p = self.get_parameter
        self.turtle   = p('turtle').value
        self.order    = int  (p('order').value)
        self.L        = float(p('side_length').value)
        self.x0       = float(p('start_x').value)
        self.y0       = float(p('start_y').value)
        self.theta0   = float(p('theta').value)
        self.pen_r    = int  (p('pen_r').value)
        self.pen_g    = int  (p('pen_g').value)
        self.pen_b    = int  (p('pen_b').value)
        self.pen_w    = int  (p('pen_width').value)

        # ---- service clients ------------------------------------------
        base = f'/{self.turtle}'
        self.abs_cli = self.create_client(TeleportAbsolute, f'{base}/teleport_absolute')
        self.rel_cli = self.create_client(TeleportRelative, f'{base}/teleport_relative')
        self.pen_cli = self.create_client(SetPen,           f'{base}/set_pen')

        # wait until every service is present
        for cli in (self.abs_cli, self.rel_cli, self.pen_cli):
            while not cli.wait_for_service(1.0):
                self.get_logger().info(f'Waiting for {cli.srv_name} …')

    # ------------------------------------------------------------------ #
    #  Internal helpers                                                  #
    # ------------------------------------------------------------------ #
    def _sync_call(self, client, request):
        """
        Send `request` asynchronously and spin *this* node until the
        future completes.  Works on every rclpy distribution.
        """
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result()     # turtlesim replies instantly

    # ---------- wrappers around turtlesim services ---------------------
    def _pen(self, *, off: int = 0) -> None:
        req = SetPen.Request()
        req.r, req.g, req.b = self.pen_r, self.pen_g, self.pen_b
        req.width = self.pen_w
        req.off   = off
        self._sync_call(self.pen_cli, req)

    def _teleport_abs(self, x: float, y: float, theta: float) -> None:
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = x, y, theta
        self._sync_call(self.abs_cli, req)

    def _teleport_rel(self, dist: float, ang: float) -> None:
        req = TeleportRelative.Request()
        req.linear, req.angular = dist, ang
        self._sync_call(self.rel_cli, req)

    # ---------- convenience -------------------------------------------
    def turn(self, angle_rad: float) -> None:
        self._teleport_rel(0.0, angle_rad)

    def forward(self, dist: float) -> None:
        self._teleport_rel(dist, 0.0)

    # ---------- Koch recursion ----------------------------------------
    def koch(self, length: float, order: int) -> None:
        if order == 0:
            self.forward(length)
            return
        self.koch(length / 3, order - 1)
        self.turn(math.radians(60))
        self.koch(length / 3, order - 1)
        self.turn(math.radians(-120))
        self.koch(length / 3, order - 1)
        self.turn(math.radians(60))
        self.koch(length / 3, order - 1)

    # ------------------------------------------------------------------ #
    #  Main drawing routine                                              #
    # ------------------------------------------------------------------ #
    def draw(self) -> None:
        self._pen(off=1)                            # lift pen
        self._teleport_abs(self.x0, self.y0, self.theta0)

        self._pen(off=0)                            # pen down
        for _ in range(3):                          # three sides
            self.koch(self.L, self.order)
            self.turn(math.radians(-120))

        self.get_logger().info('🎉 Finished drawing Koch snowflake')
        rclpy.shutdown()


# ---------------------------------------------------------------------- #
def main(args=None) -> None:
    rclpy.init(args=args)
    KochSnowflake().draw()
