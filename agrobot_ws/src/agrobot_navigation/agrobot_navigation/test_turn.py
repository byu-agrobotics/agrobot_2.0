import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from agrobot_interfaces.action import Turn
from agrobot_interfaces.msg import ToFData, DriveCommand
import sys

# This sensitivity factor translates the controller's speed command into simulation progress.
# You may need to tune this value depending on the P-gain in your controller.
# A higher sensitivity means the simulated robot turns "faster" for a given command.
PROGRESS_SENSITIVITY = 0.0002 

class TurnControllerTester(Node):
    """
    A closed-loop test node for the 'Turn' action in the drive_controller.
    
    It simulates a robot's ToF sensors within an arena. Crucially, it SUBSCRIBES
    to the drive_controller's output ('cmd/drive') and updates the simulated
    sensor values based on the commanded speed. This creates a feedback loop
    to properly test the controller's P-gain and logic.
    """
    def __init__(self, turn_angle=90.0):
        super().__init__('turn_controller_tester')
        
        if turn_angle not in [90.0, -90.0]:
            self.get_logger().error(f"Invalid turn angle: {turn_angle}. Please use 90.0 or -90.0.")
            rclpy.shutdown()
            return

        self.goal_angle = turn_angle
        self._action_client = ActionClient(self, Turn, 'control/turn')
        
        # --- MOCK SENSORS AND PHYSICS ---
        self.mock_tof_pub = self.create_publisher(ToFData, 'tof/data', 10)
        self.turn_progress = 0.0  # Represents how far into the turn we are (0.0 to 1.0)
        
        # Initial sensor readings (e.g., in a 200mm x 400mm rectangular area)
        self.initial_state = {'front': 100.0, 'back': 100.0, 'left': 200.0, 'right': 200.0}
        
        # Define target state based on the desired turn direction
        if self.goal_angle > 0: # Right turn
            self.get_logger().info("--- CONFIGURING FOR 90 DEG RIGHT TURN ---")
            self.target_state = {'front': 200.0, 'back': 200.0, 'left': 100.0, 'right': 100.0}
        else: # Left turn
            self.get_logger().info("--- CONFIGURING FOR -90 DEG LEFT TURN ---")
            self.target_state = {'front': 200.0, 'back': 200.0, 'left': 100.0, 'right': 100.0}
            # For a left turn, the initial and target states are swapped in the interpolation
            self.initial_state, self.target_state = self.target_state, self.initial_state

        self.current_tof_state = self.initial_state.copy()

        # --- CLOSED-LOOP MECHANISM ---
        # This subscriber listens to the controller's commands
        self.drive_cmd_sub = self.create_subscription(
            DriveCommand,
            'cmd/drive',
            self.drive_cmd_callback,
            10
        )
        
        # This timer continuously publishes the current state of our mock sensors
        self.tof_publish_timer = self.create_timer(0.05, self.publish_mock_data) # 20Hz
        
        # Send the goal to the controller to kick things off
        self.send_goal_timer = self.create_timer(1.0, self.send_goal_once) # Wait 1s for nodes to start
        
        self.get_logger().info('Turn tester node started. Waiting to send goal...')

    def drive_cmd_callback(self, msg: DriveCommand):
        """
        This is the core of the closed-loop test. It receives the controller's
        speed command and updates the simulation's progress.
        """
        # For an in-place turn, left and right speeds are opposite. We use left_speed.
        # A positive speed from the controller should advance a right turn.
        # A negative speed from the controller should advance a left turn.
        delta_progress = msg.left_speed * PROGRESS_SENSITIVITY
        
        # Update the turn progress based on the controller's command
        self.turn_progress += delta_progress
        
        # Optional: Log the interaction
        # self.get_logger().info(f"Received speed: {msg.left_speed}, New progress: {self.turn_progress:.4f}")

    def publish_mock_data(self):
        """
        Calculates and publishes the current mock ToF data based on turn_progress.
        This simulates the robot's sensors as it physically turns.
        """
        # Clamp progress to allow for some overshoot but prevent runaway values
        clamped_progress = max(-0.1, min(1.1, self.turn_progress))

        msg = ToFData()
        
        # Linearly interpolate each sensor value from its start to end point
        for key in self.current_tof_state:
            start_val = self.initial_state[key]
            end_val = self.target_state[key]
            self.current_tof_state[key] = start_val + (end_val - start_val) * clamped_progress
        
        msg.front = self.current_tof_state['front']
        msg.back = self.current_tof_state['back']
        msg.left = self.current_tof_state['left']
        msg.right = self.current_tof_state['right']
        
        self.mock_tof_pub.publish(msg)

    def send_goal_once(self):
        """Sends the turn goal a single time, then cancels the timer."""
        self.send_goal_timer.cancel()
        self.send_goal(self.goal_angle)

    def send_goal(self, angle):
        """Sends a goal to the Turn action server."""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Turn.Goal()
        goal_msg.angle = float(angle)

        self.get_logger().info(f'Sending turn goal request for angle: {angle}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        final_progress = self.turn_progress
        self.get_logger().info(f'Result from drive_controller: Success = {result.success}')
        self.get_logger().info(f'Test finished with final turn progress: {final_progress:.3f} (target is 1.0)')
        
        if abs(1.0 - final_progress) < 0.1:
             self.get_logger().info("TEST PASSED: Final progress is close to 1.0.")
        else:
             self.get_logger().warn("TEST FAILED: Final progress is not close to 1.0.")

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Allows you to specify the turn angle from the command line
    # `ros2 run <your_pkg> <your_node> 90` for right turn
    # `ros2 run <your_pkg> <your_node> -90` for left turn
    turn_angle = 90.0 # Default to right turn
    if len(sys.argv) > 1:
        try:
            turn_angle = float(sys.argv[1])
        except ValueError:
            print("Invalid angle provided. Using default 90.0.")

    action_client = TurnControllerTester(turn_angle=turn_angle)
    
    # Check if node initialization failed
    if rclpy.ok():
        rclpy.spin(action_client)

    action_client.destroy_node()

if __name__ == '__main__':
    main()