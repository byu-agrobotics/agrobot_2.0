import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from agrobot_interfaces.action import Center
from agrobot_interfaces.msg import ToFData, DriveCommand

class CenterControllerTester(Node):
    """
    Test node for the 'Center' action in the drive_controller.

    This node creates a closed-loop test environment:
    1. It sends a 'Center' goal to the drive_controller.
    2. It subscribes to the 'cmd/drive' topic to listen for the controller's motor commands.
    3. It runs a simple physics simulation that updates mock ToF sensor data based on
       the received motor commands.
    4. It publishes the new mock ToF data, creating a feedback loop that truly
       tests the controller's PID logic.
    """
    def __init__(self):
        super().__init__('center_controller_tester')
        self._action_client = ActionClient(self, Center, 'control/center')
        self.mock_tof_pub = self.create_publisher(ToFData, 'tof/data', 10)
        
        # Subscribe to the controller's output to create a feedback loop
        self.drive_cmd_sub = self.create_subscription(
            DriveCommand,
            'cmd/drive',
            self.drive_cmd_callback,
            10
        )
        
        # -- Simulation Parameters --
        # These factors translate motor speed commands into simulated distance changes.
        # You can tune these to change the simulation's behavior.
        # This represents how many mm the robot moves forward per speed unit per timer tick.
        self.LINEAR_SPEED_FACTOR = 0.02
        # This represents how much the robot turns (affecting side sensors) per speed unit per timer tick.
        self.ANGULAR_SPEED_FACTOR = 0.04

        # -- Simulation State --
        # Start off-center in both axes
        self.current_left = 100.0
        self.current_right = 200.0
        self.current_front = 80.0
        self.current_back = 120.0
        
        # Last received motor commands
        self.commanded_left_speed = 0.0
        self.commanded_right_speed = 0.0
        
        # -- Timers --
        self.tof_timer = self.create_timer(0.05, self.publish_mock_data) # Faster update for better simulation
        self.goal_timer = self.create_timer(1.0, self.send_goal_once) # Wait 1s for nodes to connect
        
        self.get_logger().info('Center tester node started. Simulation is running.')
        self.get_logger().info(f"Initial State: L={self.current_left}, R={self.current_right}, F={self.current_front}, B={self.current_back}")

    def drive_cmd_callback(self, msg):
        """Stores the latest motor commands from the drive controller."""
        self.commanded_left_speed = float(msg.left_speed)
        self.commanded_right_speed = float(msg.right_speed)
        self.get_logger().debug(f"Received drive command: L={msg.left_speed}, R={msg.right_speed}")

    def publish_mock_data(self):
        """
        Runs the simulation loop and publishes the new mock ToF data.
        This now simulates a real robot responding to motor commands.
        """
        # Calculate overall forward and turning "effort" from motor commands
        forward_effort = (self.commanded_left_speed + self.commanded_right_speed) / 2.0
        turn_effort = (self.commanded_right_speed - self.commanded_left_speed) / 2.0

        # Calculate the change in distances based on the effort and simulation factors
        delta_forward = forward_effort * self.LINEAR_SPEED_FACTOR
        delta_lateral = turn_effort * self.ANGULAR_SPEED_FACTOR

        # Update the mock ToF readings based on the calculated changes
        # Moving forward (positive delta_forward) decreases front distance and increases back
        self.current_front -= delta_forward
        self.current_back += delta_forward
        
        # Turning right (positive delta_lateral) decreases right distance and increases left
        self.current_right -= delta_lateral
        self.current_left += delta_lateral

        # Publish the new state
        msg = ToFData()
        msg.front = self.current_front
        msg.back = self.current_back
        msg.left = self.current_left
        msg.right = self.current_right
        self.mock_tof_pub.publish(msg)

    def send_goal_once(self):
        """Sends the center goal and cancels the timer to ensure it only runs once."""
        self.goal_timer.cancel()
        self.get_logger().info('Timer fired, sending goal to center controller.')
        self.send_goal()

    def send_goal(self):
        """Sends the goal to the action server."""
        goal_msg = Center.Goal()
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Sending center goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # The Center action doesn't have feedback, but this is good practice to include.
        pass

    def goal_response_callback(self, future):
        """Handles the response from the action server after sending a goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handles the final result of the action and shuts down the node."""
        result = future.result().result
        self.get_logger().info(f'Result from drive_controller: Success = {result.success}')
        self.get_logger().info(f"Final State: L={self.current_left:.2f}, R={self.current_right:.2f}, F={self.current_front:.2f}, B={self.current_back:.2f}")
        self.get_logger().info('Test finished, shutting down test node.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = CenterControllerTester()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()

if __name__ == '__main__':
    main()