import math
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, TeleportRelative, SetPen

class KochSnowflake(Node):
    def __init__(self) -> None:
        super().__init__('koch_snowflake_minimal')

        self.declare_parameter('turtle',        'turtle1')
        self.declare_parameter('order',         3)
        self.declare_parameter('side_length',   5.0)
        self.declare_parameter('start_x',       2.0)
        self.declare_parameter('start_y',       7.0)
        self.declare_parameter('theta',         0.0)
        self.declare_parameter('pen_r',         255)
        self.declare_parameter('pen_g',         0)
        self.declare_parameter('pen_b',         0)
        self.declare_parameter('pen_width',     2)

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

        base = f'/{self.turtle}'
        self.abs_cli = self.create_client(TeleportAbsolute, f'{base}/teleport_absolute')
        self.rel_cli = self.create_client(TeleportRelative, f'{base}/teleport_relative')
        self.pen_cli = self.create_client(SetPen,           f'{base}/set_pen')

        for cli in (self.abs_cli, self.rel_cli, self.pen_cli):
            while not cli.wait_for_service(timeout_sec=1.0):
                pass # Minimal: wait without logging

    def _sync_call(self, client, request):
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.done() and rclpy.ok():
            if future.exception() is not None:
                return None 
            return future.result()
        return None 

    def _pen(self, *, off: int = 0) -> None:
        req = SetPen.Request()
        req.r, req.g, req.b = self.pen_r, self.pen_g, self.pen_b
        req.width, req.off = self.pen_w, off
        self._sync_call(self.pen_cli, req)

    def _teleport_abs(self, x: float, y: float, theta: float) -> None:
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = x, y, theta
        self._sync_call(self.abs_cli, req)

    def _teleport_rel(self, dist: float, ang: float) -> None:
        req = TeleportRelative.Request()
        req.linear, req.angular = dist, ang
        self._sync_call(self.rel_cli, req)

    def turn(self, angle_rad: float) -> None:
        self._teleport_rel(0.0, angle_rad)

    def forward(self, dist: float) -> None:
        self._teleport_rel(dist, 0.0)

    def koch(self, length: float, order: int) -> None:
        if not rclpy.ok(): return
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

    def draw(self) -> None:
        self._pen(off=1)
        self._teleport_abs(self.x0, self.y0, self.theta0)
        self._pen(off=0)
        for _ in range(3):
            if not rclpy.ok(): break
            self.koch(self.L, self.order)
            self.turn(math.radians(-120))
        self._pen(off=1)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = KochSnowflake()
        node.draw()
    except KeyboardInterrupt:
        pass # Minimal handling for Ctrl+C
    except Exception:
        pass # Minimal handling for other errors
    finally:
        if rclpy.ok():
            rclpy.shutdown()

# Note: The 'if __name__ == "__main__":' block is omitted for strict minimality
# when run as a ROS 2 entry point (e.g., via ros2 run).
# If you need to run this script directly using "python script_name.py",
# you would add:
# if __name__ == '__main__':
#     main()