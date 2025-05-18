import rclpy
from rclpy.node import Node
from rclpy.task import Future # Ensure this import is present
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, Kill, Spawn
from std_srvs.srv import Empty as EmptyService
import time
import math
import asyncio # Import asyncio

class KochSnowflakeNode(Node):
    def __init__(self):
        super().__init__('koch_snowflake_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Service Clients
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.reset_client = self.create_client(EmptyService, '/reset')
        # self.kill_client = self.create_client(Kill, '/kill') # Not strictly needed for current flow
        # self.spawn_client = self.create_client(Spawn, '/spawn') # Not strictly needed for current flow

        self.get_logger().info('KochSnowflakeNode initialized. Waiting for services in run_drawing_sequence.')

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
        # The actual async drawing sequence will be called from main

    async def wait_for_services(self):
        essential_clients = {
            "SetPen": self.pen_client,
            "TeleportAbsolute": self.teleport_client,
            "Reset": self.reset_client,
        }
        self.get_logger().info('Waiting for essential services...')
        for name, client in essential_clients.items():
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'{name} service not available, waiting again...')
        self.get_logger().info('All essential services available.')
        return True


    async def run_drawing_sequence(self):
        if not await self.wait_for_services():
            self.get_logger().error("Essential services not available. Aborting.")
            return

        self.get_logger().info("Starting drawing sequence...")
        setup_ok = await self.setup_turtle()
        if setup_ok:
            await self.draw_koch_snowflake()
            self.get_logger().info('Koch Snowflake drawing sequence complete.')
        else:
            self.get_logger().error("Failed to setup turtle. Drawing aborted.")
        
        self.get_logger().info("Drawing sequence finished.")
        # Shutdown will be handled in main after this coroutine completes


    async def call_service_async(self, client, request, service_name_for_log="service"):
        self.get_logger().info(f"Attempting to call {service_name_for_log}...")
        if not client.service_is_ready(): # Should have been checked by wait_for_services, but good practice
            self.get_logger().error(f"{service_name_for_log} is not available before call!")
            return None
        
        try:
            future: Future = client.call_async(request)
            self.get_logger().info(f"{service_name_for_log} call initiated, waiting for future...")
            
            # rclpy.spin_until_future_complete is a blocking call in terms of async.
            # It spins the node's executor until the future is done.
            # This is suitable when an async method needs to await a ROS service call.
            self.executor.spin_until_future_complete(future, timeout_sec=5.0)

            if future.done():
                self.get_logger().info(f"Future for {service_name_for_log} is done.")
                response = future.result() # Get result after future is done
                exception = future.exception() # Get exception after future is done

                if exception is not None:
                    self.get_logger().error(f"Exception from {service_name_for_log}: {exception}")
                    return None
                
                if response is not None: # For Empty service, response is Empty.Response, not None
                    self.get_logger().info(f"{service_name_for_log} call successful.")
                    return response
                else:
                    # This should ideally not happen if no exception and service is valid (e.g. Empty)
                    self.get_logger().warn(f"{service_name_for_log} call returned None result without exception.")
                    return None # Or handle as appropriate for the specific service
            else: # Timeout occurred in spin_until_future_complete
                self.get_logger().error(f"Timeout waiting for {service_name_for_log} (spin_until_future_complete).")
                return None

        except Exception as e:
            self.get_logger().error(f'General exception during service call to {service_name_for_log}: {e}', exc_info=True)
            return None


    async def setup_turtle(self):
        self.get_logger().info('Executing setup_turtle...')

        self.get_logger().info('Attempting to reset Turtlesim environment via /reset service...')
        reset_req = EmptyService.Request()
        reset_response = await self.call_service_async(self.reset_client, reset_req, "/reset service")

        if reset_response is None:
            self.get_logger().error("Failed to reset Turtlesim environment. Aborting setup.")
            return False
        self.get_logger().info('Turtlesim environment reset successfully.')
        await asyncio.sleep(0.5) # Use asyncio.sleep

        self.get_logger().info(f'Teleporting turtle1 to ({self.initial_x}, {self.initial_y}, theta={self.initial_theta:.2f}).')
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = self.initial_x
        teleport_req.y = self.initial_y
        teleport_req.theta = self.initial_theta
        teleport_response = await self.call_service_async(self.teleport_client, teleport_req, "/turtle1/teleport_absolute service")
        if teleport_response is None:
            self.get_logger().error("Failed to teleport turtle. Aborting setup.")
            return False
        self.get_logger().info("Turtle teleported successfully.")
        await asyncio.sleep(0.2)

        self.get_logger().info(f'Setting pen: R={self.pen_r}, G={self.pen_g}, B={self.pen_b}, Width={self.pen_width}, Pen ON.')
        pen_req = SetPen.Request()
        pen_req.r = self.pen_r
        pen_req.g = self.pen_g
        pen_req.b = self.pen_b
        pen_req.width = self.pen_width
        pen_req.off = 0
        pen_response = await self.call_service_async(self.pen_client, pen_req, "/turtle1/set_pen service")
        if pen_response is None:
            self.get_logger().error("Failed to set pen. Aborting setup.")
            return False
        self.get_logger().info("Pen set successfully.")
        await asyncio.sleep(0.2)

        self.get_logger().info('Turtle setup complete.')
        return True


    def move_forward(self, distance): # This remains synchronous for now
        if distance == 0: return
        msg = Twist()
        msg.linear.x = self.speed
        duration = abs(distance / self.speed)
        self.publisher_.publish(msg)
        time.sleep(duration) # Blocking sleep
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.1)

    def turn(self, angle_degrees): # This remains synchronous for now
        if angle_degrees == 0: return
        msg = Twist()
        angle_radians = math.radians(angle_degrees)
        msg.angular.z = self.turn_speed if angle_radians > 0 else -self.turn_speed
        duration = abs(angle_radians / self.turn_speed)
        self.publisher_.publish(msg)
        time.sleep(duration) # Blocking sleep
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.1)

    async def koch_curve_async(self, order, length): # Renamed to avoid conflict if we had a sync one
        if order == 0:
            self.move_forward(length) # Still uses sync move_forward
            await asyncio.sleep(0) # Yield control briefly
        else:
            length_third = length / 3.0
            await self.koch_curve_async(order - 1, length_third)
            self.turn(60) # Still uses sync turn
            await asyncio.sleep(0)
            await self.koch_curve_async(order - 1, length_third)
            self.turn(-120)
            await asyncio.sleep(0)
            await self.koch_curve_async(order - 1, length_third)
            self.turn(60)
            await asyncio.sleep(0)
            await self.koch_curve_async(order - 1, length_third)

    async def draw_koch_snowflake(self):
        self.get_logger().info(f"Starting to draw Koch snowflake: Order={self.order}, Side Length={self.length}")
        for i in range(3):
            self.get_logger().info(f"Drawing side {i+1}/3 of the snowflake.")
            await self.koch_curve_async(self.order, self.length) # Use async version
            self.turn(-120) # Still uses sync turn
            await asyncio.sleep(0) # Yield control
        self.get_logger().info("Finished drawing Koch snowflake main structure.")

        self.get_logger().info("Lifting pen...")
        pen_req = SetPen.Request()
        pen_req.off = 1
        pen_req.r, pen_req.g, pen_req.b, pen_req.width = self.pen_r, self.pen_g, self.pen_b, self.pen_width
        await self.call_service_async(self.pen_client, pen_req, "/turtle1/set_pen (lift) service")
        self.get_logger().info("Pen lifted.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KochSnowflakeNode()
        # We need to run the async function run_drawing_sequence.
        # rclpy.spin() doesn't directly integrate with asyncio's event loop by default.
        # For a single async task like this, we can use asyncio.run()
        # but need to be careful as rclpy itself has an executor.

        # A common pattern for running an async ROS node method:
        # 1. Create the node.
        # 2. Use an executor that can handle the node's callbacks AND the asyncio task.
        # For simplicity here, since it's a single sequence:
        asyncio.run(node.run_drawing_sequence())

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('KeyboardInterrupt, shutting down.')
    except Exception as e:
        if node:
            node.get_logger().error(f"An error occurred in main: {e}", exc_info=True)
        else:
            print(f"An error occurred before node initialization or during async run: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 shutdown complete from main.")

if __name__ == '__main__':
    main()