import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TriangleDrawer(Node):

    def __init__(self):
        super().__init__('triangle_drawer_node') # Changed node name for clarity
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Triangle parameters
        self.side_length = 2.0  # You can adjust this value
        self.angle_degrees = 120.0  # For an equilateral triangle, turn 120 degrees
        self.angle_radians = math.radians(self.angle_degrees)

        # Movement parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5 # rad/s - Adjust for faster/slower turns

        # Calculate durations
        self.time_to_move_side = self.side_length / self.linear_speed
        self.time_to_turn = self.angle_radians / self.angular_speed

        self.get_logger().info('Triangle Drawer Node has been started.')
        self.get_logger().info(f'Side length: {self.side_length}m, Turn angle: {self.angle_degrees} degrees')
        self.get_logger().info(f'Calculated move time per side: {self.time_to_move_side:.2f}s')
        self.get_logger().info(f'Calculated turn time: {self.time_to_turn:.2f}s')

        # Start drawing after a brief pause to allow turtlesim to fully initialize
        self.create_timer(1.0, self.draw_triangle_once) # Timer calls draw_triangle_once after 1s

    def draw_triangle_once(self):
        # This timer callback will run only once because we cancel it.
        if hasattr(self, 'draw_timer'): # Check if timer exists
            self.draw_timer.cancel()

        self.get_logger().info('Starting to draw the triangle...')
        for i in range(3):
            self.move_forward()
            self.turn()
        
        self.stop_turtle()
        self.get_logger().info('Triangle drawing complete!')
        # self.destroy_node() # Optional: self-destruct after drawing
        # rclpy.shutdown()   # Optional: shutdown rclpy if self-destructing

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving forward for {self.time_to_move_side:.2f} seconds...')
        time.sleep(self.time_to_move_side) # Blocking call

    def turn(self):
        twist = Twist()
        twist.linear.x = 0.0
        # For turtlesim, positive angular.z is counter-clockwise (left turn)
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)
        self.get_logger().info(f'Turning for {self.time_to_turn:.2f} seconds...')
        time.sleep(self.time_to_turn) # Blocking call

    def stop_turtle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Turtle stopped.')

    def on_shutdown(self): # Custom shutdown behavior
        self.get_logger().info('Triangle Drawer Node is shutting down.')
        self.stop_turtle()


def main(args=None):
    rclpy.init(args=args)
    triangle_drawer = TriangleDrawer()
    
    # Store the timer so it can be cancelled in draw_triangle_once
    # This is a bit of a workaround for a one-shot delayed action.
    # A more complex state machine would be used for continuous/reactive behavior.
    triangle_drawer.draw_timer = triangle_drawer.create_timer(0.1, lambda: None) # Dummy timer to be replaced
    triangle_drawer.draw_timer.cancel() # Cancel dummy immediately
    triangle_drawer.draw_timer = triangle_drawer.create_timer(1.0, triangle_drawer.draw_triangle_once)


    try:
        rclpy.spin(triangle_drawer)
    except KeyboardInterrupt:
        triangle_drawer.get_logger().info('Node interrupted by user.')
    finally:
        triangle_drawer.on_shutdown() # Call custom shutdown
        if rclpy.ok(): # Check if context is still valid
            triangle_drawer.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()