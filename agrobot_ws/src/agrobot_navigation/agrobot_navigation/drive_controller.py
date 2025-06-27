import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from agrobot_interfaces.msg import ToFData, DriveCommand
from agrobot_interfaces.action import DriveStraight, Turn, Center
from action_msgs.msg import GoalStatus
from simple_pid import PID

# Thresholds for completing actions
STABILITY_THRESHOLD = 5
CENTERING_TOLERANCE_MM = 5
DRIVE_STRAIGHT_TOLERANCE_MM = 1
TURN_ERROR_TOLERANCE_MM = 20
TURN_TIMEOUT_S = 15.0

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        
        cb_group = ReentrantCallbackGroup()

        self.declare_parameter('p_forward', 0.5)
        self.declare_parameter('i_forward', 0.0)
        self.declare_parameter('d_forward', 0.1)
        self.declare_parameter('p_lateral', 0.02)
        self.declare_parameter('i_lateral', 0.0)
        self.declare_parameter('d_lateral', 0.05)
        self.declare_parameter('turn_speed', 50)
        self.declare_parameter('p_turn', 0.3) # Proportional gain for turning
        self.declare_parameter('max_turn_speed', 80) # Max speed for proportional turn

        p_forward = self.get_parameter('p_forward').get_parameter_value().double_value
        i_forward = self.get_parameter('i_forward').get_parameter_value().double_value
        d_forward = self.get_parameter('d_forward').get_parameter_value().double_value
        p_lateral = self.get_parameter('p_lateral').get_parameter_value().double_value
        i_lateral = self.get_parameter('i_lateral').get_parameter_value().double_value
        d_lateral = self.get_parameter('d_lateral').get_parameter_value().double_value
        self.p_turn = self.get_parameter('p_turn').get_parameter_value().double_value
        self.max_turn_speed = self.get_parameter('max_turn_speed').get_parameter_value().integer_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().integer_value

        self.pid_forward = PID(p_forward, i_forward, d_forward, setpoint=0)
        self.pid_lateral = PID(p_lateral, i_lateral, d_lateral, setpoint=0)
        self.pid_forward.output_limits = (-100, 100)
        self.pid_lateral.output_limits = (-50, 50)
        
        self.tof_data = None
        self.active_goal = False
        
        self.drive_pub = self.create_publisher(DriveCommand, 'cmd/drive', 10)

        self.tof_sub = self.create_subscription(
            ToFData, 'tof/data', self.tof_callback, 10, callback_group=cb_group
        )

        self.center_action_server = ActionServer(
            self, Center, 'control/center',
            execute_callback=self.execute_center_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )
        self.drive_straight_action_server = ActionServer(
            self, DriveStraight, 'control/drive_straight',
            execute_callback=self.drive_straight_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )
        self.turn_action_server = ActionServer(
            self, Turn, 'control/turn',
            execute_callback=self.execute_turn_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )
        self.get_logger().info("Drive Controller node is ready.")

    def tof_callback(self, msg):
        """Stores the latest ToF sensor data."""
        self.tof_data = msg
        self.get_logger().info(f"TOF Data: [F:{msg.front}, B:{msg.back}, L:{msg.left}, R:{msg.right}]") # <-- Optional: Uncomment for very verbose logging

    def stop_robot(self):
        drive_cmd = DriveCommand()
        drive_cmd.left_speed = 0
        drive_cmd.right_speed = 0
        self.drive_pub.publish(drive_cmd)
        self.get_logger().info("Robot stopped.")

    def goal_callback(self, goal_request):
        if self.active_goal:
            self.get_logger().warn('Another goal is active, rejecting new goal.')
            return GoalResponse.REJECT
        self.active_goal = True
        self.get_logger().info(f'Accepting goal for {goal_request.__class__.__name__}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal.')
        self.active_goal = False
        self.stop_robot()
        return CancelResponse.ACCEPT

    def execute_turn_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: Turn {goal_handle.request.angle} degrees')
        angle = goal_handle.request.angle

        if self.tof_data is None:
            self.get_logger().error("No ToF data available to start turn.")
            goal_handle.abort()
            self.active_goal = False
            return Turn.Result(success=False)

        # --- Initial and Target State ---
        initial_front = self.tof_data.front
        initial_back = self.tof_data.back
        initial_left = self.tof_data.left
        initial_right = self.tof_data.right

        if angle > 0: # Right Turn (+90 deg)
            target_front = initial_left
            target_right = initial_front
            target_back = initial_right
            target_left = initial_back
        else: # Left Turn (-90 deg)
            target_front = initial_right
            target_left = initial_front
            target_back = initial_left
            target_right = initial_back

        self.get_logger().info(f"Turn Initial -> Target: F({initial_front}->{target_front}), L({initial_left}->{target_left}), R({initial_right}->{target_right})")

        start_time = self.get_clock().now()
        stability_count = 0
        rate = self.create_rate(20)

        while rclpy.ok() and self.active_goal:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > TURN_TIMEOUT_S:
                self.get_logger().error("Turn action timed out!")
                goal_handle.abort()
                break

            if self.tof_data:
                # --- Proportional Control Logic ---

                # ================================================================= #
                # CORRECTED SIGNED ERROR LOGIC
                # ================================================================= #
                if angle > 0: # Right turn
                    # The error is the difference between the target sensor differential and the current one.
                    # This value is non-zero at the start and approaches zero at the goal.
                    target_differential = target_front - target_right # (initial_left - initial_front)
                    current_differential = self.tof_data.front - self.tof_data.right
                    signed_error = target_differential - current_differential
                else: # Left turn
                    target_differential = target_front - target_left # (initial_right - initial_front)
                    current_differential = self.tof_data.front - self.tof_data.left
                    signed_error = target_differential - current_differential
                # ================================================================= #
                
                turn_speed = self.p_turn * signed_error
                turn_speed = max(min(turn_speed, self.max_turn_speed), -self.max_turn_speed)

                drive_cmd = DriveCommand()
                drive_cmd.left_speed = int(turn_speed)
                drive_cmd.right_speed = int(-turn_speed)
                self.drive_pub.publish(drive_cmd)

                # --- Success Condition Check ---
                total_error = sum([
                    abs(self.tof_data.front - target_front),
                    abs(self.tof_data.back - target_back),
                    abs(self.tof_data.left - target_left),
                    abs(self.tof_data.right - target_right)
                ])

                self.get_logger().info(f"Turn Progress: SignedError={signed_error:.2f}, TotalError={total_error:.2f}, TurnSpeed={turn_speed:.2f}")

                if total_error < TURN_ERROR_TOLERANCE_MM:
                    stability_count += 1
                    if stability_count >= STABILITY_THRESHOLD:
                        self.get_logger().info("Turn successful!")
                        goal_handle.succeed()
                        break
                else:
                    stability_count = 0
            
            rate.sleep()

        self.stop_robot()
        self.active_goal = False
        return Turn.Result(success=(goal_handle.status == GoalStatus.STATUS_SUCCEEDED))

    def execute_center_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Center')
        
        if self.tof_data is None:
            self.get_logger().error('No ToF data available to start centering.')
            goal_handle.abort()
            self.active_goal = False
            return Center.Result(success=False)

        rate = self.create_rate(20)
        stability_count = 0

        # Reset both PID controllers to clear any prior state
        self.pid_forward.reset()
        self.pid_lateral.reset()

        while rclpy.ok() and self.active_goal:
            if self.tof_data is None:
                rate.sleep()
                continue
            
            # Calculate error for both axes
            # Note: If the robot's physical center isn't aligned with the sensors,
            # you may need to add offsets. For example:
            # lateral_error = (self.tof_data.left + LEFT_OFFSET) - (self.tof_data.right + RIGHT_OFFSET)
            lateral_error = self.tof_data.left - self.tof_data.right
            forward_error = self.tof_data.front - self.tof_data.back
            
            # Check if both axes are centered
            is_laterally_centered = abs(lateral_error) < CENTERING_TOLERANCE_MM
            is_forward_centered = abs(forward_error) < CENTERING_TOLERANCE_MM

            if is_laterally_centered and is_forward_centered:
                stability_count += 1
                if stability_count >= STABILITY_THRESHOLD:
                    self.get_logger().info("Centering successful in both axes.")
                    goal_handle.succeed()
                    break
            else:
                stability_count = 0

            # Get control output from each PID controller
            # A positive forward_error (front > back) means we need to move backward, hence the negation.
            forward_speed = -self.pid_forward(forward_error)
            
            # A positive lateral_error (left > right) means we need to turn right.
            # A positive turn_correction will increase right wheel speed and decrease left.
            turn_correction = self.pid_lateral(lateral_error)
            
            self.get_logger().info(f"Centering PID: FwdError={forward_error:.2f}, LatError={lateral_error:.2f} | FwdSpeed={forward_speed:.2f}, TurnCorrect={turn_correction:.2f}")

            # Combine forward speed and turn correction to drive the motors
            drive_cmd = DriveCommand()
            drive_cmd.left_speed = int(forward_speed - turn_correction)
            drive_cmd.right_speed = int(forward_speed + turn_correction)
            self.drive_pub.publish(drive_cmd)
            
            rate.sleep()
        
        self.stop_robot()
        self.active_goal = False
        return Center.Result(success=True if goal_handle.status == GoalStatus.STATUS_SUCCEEDED else False)

    def drive_straight_execute_callback(self, goal_handle):
        """
        Executes the drive straight action.
        Drives the robot forward until a target front distance is met, while maintaining
        the initial lateral distance between the left and right walls using PID control.
        """
        self.get_logger().info(f'Executing goal: Drive Straight to {goal_handle.request.front_distance}mm')
        
        # 1. --- Initial Setup and Sanity Checks ---
        if self.tof_data is None:
            self.get_logger().error('No ToF data available. Aborting drive straight.')
            goal_handle.abort()
            self.active_goal = False
            return DriveStraight.Result(success=False)

        # 2. --- Define Control Targets ---
        desired_front_distance = float(goal_handle.request.front_distance)
        
        # KEY IMPROVEMENT: Capture the initial difference between left and right sensors.
        # This will be the setpoint for our lateral PID controller.
        # The goal is to maintain this offset, effectively driving in a straight line.
        initial_lateral_offset = self.tof_data.left - self.tof_data.right
        self.get_logger().info(f'Target front distance: {desired_front_distance}mm')
        self.get_logger().info(f'Maintaining lateral offset: {initial_lateral_offset:.2f}mm (L-R)')

        # 3. --- Reset PID Controllers ---
        self.pid_forward.reset()
        self.pid_lateral.reset()
        
        # 4. --- Control Loop ---
        rate = self.create_rate(20)  # 20 Hz loop
        stability_count = 0

        while rclpy.ok() and self.active_goal:
            if self.tof_data is None:
                self.get_logger().warn('TOF data became unavailable during loop.')
                rate.sleep()
                continue

            # --- Forward Proportional Control ---
            # The error is the difference between where we want to be and where we are.
            # A positive error means we need to move forward.
            forward_error = self.tof_data.front - desired_front_distance
            
            # The PID controller's output is proportional to the error.
            # We negate the output because a positive error (we are too far)
            # should result in a negative command to move forward (assuming negative values drive your motors forward).
            # If positive values drive your motors forward, change this to:
            # forward_speed = self.pid_forward(desired_front_distance - self.tof_data.front)
            forward_speed = -self.pid_forward(forward_error)

            # --- Lateral Proportional Control ---
            # The goal is to keep the current L-R offset the same as the initial one.
            current_lateral_offset = self.tof_data.left - self.tof_data.right
            lateral_error = current_lateral_offset - initial_lateral_offset
            
            # The PID output provides a correction value.
            # If lateral_error > 0, it means (L-R) has increased, so the robot has drifted right.
            # The 'turn_correction' will be positive, causing a left turn to correct the path.
            turn_correction = self.pid_lateral(lateral_error)
            
            # 5. --- Success Condition Check ---
            # Check if the robot has reached the target front distance and is stable.
            if abs(forward_error) < DRIVE_STRAIGHT_TOLERANCE_MM:
                stability_count += 1
                if stability_count >= STABILITY_THRESHOLD:
                    self.get_logger().info("Drive straight successful: Target distance reached.")
                    goal_handle.succeed()
                    break # Exit the control loop
            else:
                stability_count = 0 # Reset if we move out of tolerance

            # 6. --- Publish Motor Commands ---
            drive_cmd = DriveCommand()
            # Combine forward speed with the turning correction.
            # A positive 'turn_correction' should cause a left turn (decrease left, increase right).
            drive_cmd.left_speed = int(forward_speed - turn_correction)
            drive_cmd.right_speed = int(forward_speed + turn_correction)
            self.drive_pub.publish(drive_cmd)
            
            # Optional: Log the PID states for debugging
            self.get_logger().info(f"DriveStraight PID: FwdErr={forward_error:.1f}, LatErr={lateral_error:.1f} | FwdSpd={forward_speed:.1f}, TurnCorr={turn_correction:.1f}")

            rate.sleep()

        # 7. --- Cleanup ---
        self.stop_robot()
        self.active_goal = False
        return DriveStraight.Result(success=(goal_handle.status == GoalStatus.STATUS_SUCCEEDED))


def main(args=None):
    rclpy.init(args=args)
    drive_controller = DriveController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(drive_controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        drive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
