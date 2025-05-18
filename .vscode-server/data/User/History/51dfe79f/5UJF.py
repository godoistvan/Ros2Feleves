import rclpy
from rclpy.node import Node
from rclpy.task import Future # Ensure this import is present
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, Kill, Spawn
from std_srvs.srv import Empty as EmptyService
import time
import math

class KochSnowflakeNode(Node):
    def __init__(self):
        super().__init__('koch_snowflake_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Service Clients
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.reset_client = self.create_client(EmptyService, '/reset')
        self.kill_client = self.create_client(Kill, '/kill') # Not used in current flow but good to have
        self.spawn_client = self.create_client(Spawn, '/spawn') # Not used in current flow but good to have

        # Wait for all essential services to be available
        essential_clients = {
            "SetPen": self.pen_client,
            "TeleportAbsolute": self.teleport_client,
            "Reset": self.reset_client,
        }
        self.get_logger().info('Waiting for essential services...')
        for name, client in essential_clients.items():
            while not client.wait_for_service(timeout_sec=2.0): # Increased timeout slightly
                self.get_logger().warn(f'{name} service not available, waiting again...')
        self.get_logger().info('All essential services available. Koch Snowflake Node started.')

        # Declare and get parameters
        self.declare_parameter('order', 3)
        self.declare_parameter('length', 3.0)
        self.declare_parameter('speed', 2.5)
        self.declare_parameter('turn_speed', 1.5)
        self.declare_parameter('pen_color_r', 255)
        self.declare_parameter('pen_color_g', 255)
        self.declare_parameter('pen_color_b', 255)
        self.declare_parameter('pen_width', 2)
        self.declare_parameter('initial_x', 2.0)
        self.declare_parameter('initial_y', 7.5)
        self.declare_parameter('initial_theta', 0.0)

        self.order = self.get_parameter('order').get_parameter_value().integer_value
        self.length = self.get_parameter('length').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.pen_r = self.get_parameter('pen_color_r').get_parameter_value().integer_value
        self.pen_g = self.get_parameter('pen_color_g').get_parameter_value().integer_value
        self.pen_b = self.get_parameter('pen_color_b').get_parameter_value().integer_value
        self.pen_width = self.get_parameter('pen_width').get_parameter_value().integer_value
        self.initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.initial_theta = self.get_parameter('initial_theta').get_parameter_value().double_value

        # Run the main drawing logic
        # Use create_task to run async methods from sync constructor
        self.draw_task = self.create_task(self.run_drawing_sequence())


    async def run_drawing_sequence(self):
        self.get_logger().info("Starting drawing sequence...")
        setup_ok = await self.setup_turtle()
        if setup_ok:
            await self.draw_koch_snowflake()
            self.get_logger().info('Koch Snowflake drawing sequence complete.')
        else:
            self.get_logger().error("Failed to setup turtle. Drawing aborted.")
        
        # Since this node's purpose is singular, we can request shutdown.
        # If this node had other responsibilities, you might not do this here.
        self.get_logger().info("Requesting shutdown after drawing sequence.")
        rclpy.shutdown()


    async def call_service_async(self, client, request, service_name_for_log="service"):
        self.get_logger().info(f"Attempting to call {service_name_for_log}...")
        if not client.service_is_ready():
            self.get_logger().error(f"{service_name_for_log} is not available before call!")
            return None
        
        try:
            future: Future = client.call_async(request)
            self.get_logger().info(f"{service_name_for_log} call initiated, waiting for future...")

            max_wait_seconds = 5.0 # Total time to wait for the future
            check_interval_seconds = 0.1 # How often to spin and check
            elapsed_time = 0.0

            while rclpy.ok() and not future.done() and elapsed_time < max_wait_seconds:
                rclpy.spin_once(self, timeout_sec=check_interval_seconds)
                elapsed_time += check_interval_seconds
                # self.get_logger().debug(f"Spinning for {service_name_for_log}, elapsed: {elapsed_time:.1f}s") # DEBUG, very verbose

            if future.done():
                self.get_logger().info(f"Future for {service_name_for_log} is done.")
                response = future.result()
                if response is not None:
                    self.get_logger().info(f"{service_name_for_log} call successful.")
                    return response
                else:
                    # This case might happen if the service call itself didn't throw an exception,
                    # but the service implementer returned None, or for Empty services where the response object exists but might be evaluated as 'None' if not checked carefully.
                    # Check for specific exception in the future if result is None
                    if future.exception() is not None:
                         self.get_logger().error(f"Exception from {service_name_for_log}: {future.exception()}")
                         return None # Explicitly return None on exception
                    # For std_srvs.Empty, future.result() is an Empty.Response object, not Python None.
                    # So if it truly is None here, it's likely an issue or unexpected service behavior.
                    self.get_logger().warn(f"{service_name_for_log} future.result() is None. Exception details: {future.exception()}")
                    return response # Return the None (or valid Empty.Response)
            else:
                self.get_logger().error(f"Timeout while waiting for {service_name_for_log} to complete after {max_wait_seconds:.1f} seconds.")
                return None

        except Exception as e:
            self.get_logger().error(f'Exception during service call to {service_name_for_log}: {e}', exc_info=True)
            return None


    async def setup_turtle(self):
        self.get_logger().info('Executing setup_turtle...')

        # 1. Reset turtlesim environment
        self.get_logger().info('Attempting to reset Turtlesim environment via /reset service...')
        reset_req = EmptyService.Request()
        reset_response = await self.call_service_async(self.reset_client, reset_req, "/reset service")

        if reset_response is None:
            self.get_logger().error("Failed to reset Turtlesim environment. Aborting setup.")
            return False # Indicate setup failure
        self.get_logger().info('Turtlesim environment reset successfully.')
        await self.sleep_async(0.5) # Use async sleep

        # 2. Teleport to desired starting position and orientation
        self.get_logger().info(f'Teleporting turtle1 to ({self.initial_x}, {self.initial_y}, theta={self.initial_theta:.2f}).')
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = self.initial_x
        teleport_req.y = self.initial_y
        teleport_req.theta = self.initial_theta
        teleport_response = await self.call_service_async(self.teleport_client, teleport_req, "/turtle1/teleport_absolute service")
        if teleport_response is None:
            self.get_logger().error("Failed to teleport turtle. Aborting setup.")
            return False # Indicate setup failure
        self.get_logger().info("Turtle teleported successfully.")
        await self.sleep_async(0.2)

        # 3. Set pen properties
        self.get_logger().info(f'Setting pen: R={self.pen_r}, G={self.pen_g}, B={self.pen_b}, Width={self.pen_width}, Pen ON.')
        pen_req = SetPen.Request()
        pen_req.r = self.pen_r
        pen_req.g = self.pen_g
        pen_req.b = self.pen_b
        pen_req.width = self.pen_width
        pen_req.off = 0  # Pen ON
        pen_response = await self.call_service_async(self.pen_client, pen_req, "/turtle1/set_pen service")
        if pen_response is None:
            self.get_logger().error("Failed to set pen. Aborting setup.")
            return False # Indicate setup failure
        self.get_logger().info("Pen set successfully.")
        await self.sleep_async(0.2)

        self.get_logger().info('Turtle setup complete.')
        return True # Indicate setup success


    async def sleep_async(self, duration_sec):
        """Asynchronous sleep that allows other ROS2 tasks to run."""
        # This is a simple way to yield control. For more complex scenarios,
        # rclpy's timers or other async primitives might be better.
        # Here, we just want to pause without blocking the executor entirely.
        # For very short delays, time.sleep might be fine, but for anything
        # substantial in an async context, it's good to be non-blocking.
        # A proper async sleep would use asyncio.sleep if we were in a full asyncio loop.
        # In rclpy, a timer that fires once is a common pattern for delayed actions.
        # For this simple case, let's just log and use time.sleep, as spin_once
        # is handled elsewhere (like in service calls or the main spin if we had one).
        # If this was a long-running node, we'd integrate better with rclpy's event loop.
        # self.get_logger().debug(f"Sleeping for {duration_sec}s...") # Optional
        time.sleep(duration_sec) # For simplicity in this script.

    def move_forward(self, distance):
        if distance == 0:
            return
        msg = Twist()
        msg.linear.x = self.speed
        duration = abs(distance / self.speed)
        
        self.publisher_.publish(msg)
        # In a purely synchronous node, time.sleep() would be okay here.
        # Since we are moving towards async, ideally movement would also be async.
        # However, for turtlesim, command-and-wait is common.
        # The spin_once in service calls helps, but not during raw publishing sleeps.
        # For this project, direct time.sleep for movement is often tolerated.
        time.sleep(duration) 

        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.1) # Ensure stop command is processed

    def turn(self, angle_degrees):
        if angle_degrees == 0:
            return
        msg = Twist()
        angle_radians = math.radians(angle_degrees)
        msg.angular.z = self.turn_speed if angle_radians > 0 else -self.turn_speed
        
        duration = abs(angle_radians / self.turn_speed)
        self.publisher_.publish(msg)
        time.sleep(duration)

        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.1)

    def koch_curve(self, order, length):
        if order == 0:
            self.move_forward(length)
        else:
            length_third = length / 3.0
            self.koch_curve(order - 1, length_third)
            self.turn(60)
            self.koch_curve(order - 1, length_third)
            self.turn(-120)
            self.koch_curve(order - 1, length_third)
            self.turn(60)
            self.koch_curve(order - 1, length_third)

    async def draw_koch_snowflake(self):
        self.get_logger().info(f"Starting to draw Koch snowflake: Order={self.order}, Side Length={self.length}")
        for i in range(3):
            self.get_logger().info(f"Drawing side {i+1}/3 of the snowflake.")
            self.koch_curve(self.order, self.length)
            self.turn(-120)
        self.get_logger().info("Finished drawing Koch snowflake main structure.")

        # Lift pen after drawing
        self.get_logger().info("Lifting pen...")
        pen_req = SetPen.Request()
        pen_req.off = 1
        pen_req.r, pen_req.g, pen_req.b, pen_req.width = self.pen_r, self.pen_g, self.pen_b, self.pen_width # Keep other params
        await self.call_service_async(self.pen_client, pen_req, "/turtle1/set_pen (lift) service")
        self.get_logger().info("Pen lifted.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KochSnowflakeNode()
        # The node's __init__ now creates a task for the drawing sequence.
        # We need to spin the node to allow this task and its async calls to execute.
        # The task itself will call rclpy.shutdown() when it's done.
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('KeyboardInterrupt, shutting down.')
    except Exception as e:
        if node:
            node.get_logger().error(f"An error occurred in main: {e}", exc_info=True)
        else:
            print(f"An error occurred before node initialization: {e}")
    finally:
        # Shutdown is now initiated by the node itself.
        # If rclpy is already shut down, this won't do much.
        # If it was a KeyboardInterrupt before shutdown, this ensures cleanup.
        if rclpy.ok() and node: # Check if node exists
            node.destroy_node()
        if rclpy.ok(): # Check if rclpy is still initialized
             rclpy.shutdown() # Ensure it's shutdown if not already.
        print("ROS 2 shutdown complete from main.")

if __name__ == '__main__':
    main()