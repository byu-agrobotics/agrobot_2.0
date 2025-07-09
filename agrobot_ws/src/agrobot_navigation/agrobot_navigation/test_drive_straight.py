import rclpy
import time
import random
from rclpy.action import ActionClient
from rclpy.node import Node

from agrobot_interfaces.msg import ToFData, DriveCommand
from agrobot_interfaces.action import DriveStraight

class DriveControllerTester(Node):
    """
    A test node that simulates a robot for drive_controller.py.
    - It simulates the robot's position and orientation based on drive commands.
    - It publishes noisy ToF data based on its simulated state.
    - It calls the /control/drive_straight action server to start the test.
    """

    def __init__(self):
        super().__init__('drive_controller_tester')

        # --- Simulation State ---
        # Start with an offset to test the lateral controller's correction
        self.sim_front = 300.0  # Start further away
        self.sim_back = 290.0
        self.sim_left = 148.0   # Start slightly drifted to the right
        self.sim_right = 152.0
        
        self.last_left_speed = 0.0
        self.last_right_speed = 0.0
        
        # --- Simulation Tuning Parameters ---
        # How much 1 unit of average speed changes front distance per tick
        self.SPEED_TO_DISTANCE_SCALE = 0.01 
        # How much 1 unit of differential speed changes lateral distances per tick
        self.TURN_TO_LATERAL_SCALE = 0.02 

        # --- ROS2 Communications ---
        self.tof_publisher = self.create_publisher(ToFData, 'tof/data', 10)
        self.drive_sub = self.create_subscription(
            DriveCommand,
            'cmd/drive',
            self.drive_cmd_callback,
            10)
        self.simulation_timer = self.create_timer(0.05, self.update_simulation) # 20Hz, matches controller
        self._action_client = ActionClient(self, DriveStraight, 'control/drive_straight')
        
        self.get_logger().info('Drive Controller Tester has started.')
        self.get_logger().info(f'Initial State: F:{self.sim_front}, L:{self.sim_left}, R:{self.sim_right}')


    def drive_cmd_callback(self, msg: DriveCommand):
        """Listen to the controller's speed commands and store them."""
        self.last_left_speed = float(msg.left_speed)
        self.last_right_speed = float(msg.right_speed)

    def update_simulation(self):
        """This is the core of our simulator, now with turning!"""
        
        # 1. Calculate average forward speed and differential turn speed
        forward_speed = (self.last_left_speed + self.last_right_speed) / 2.0
        # If right_speed > left_speed, it's a left turn. turn_speed will be positive.
        turn_speed = self.last_right_speed - self.last_left_speed
        
        # 2. Update simulated forward distance
        # A positive 'forward_speed' from the controller means move forward.
        # Moving forward should DECREASE the distance to the front wall.
        distance_change = forward_speed * self.SPEED_TO_DISTANCE_SCALE
        self.sim_front -= distance_change # <--- CORRECTED LOGIC

        # 3. Update simulated lateral distances based on turning
        lateral_change = turn_speed * self.TURN_TO_LATERAL_SCALE
        # A positive turn_speed (left turn) should decrease left distance and increase right
        self.sim_left -= lateral_change
        self.sim_right += lateral_change

        # 4. Publish the new ToF data with some noise
        msg = ToFData()
        msg.front = self.sim_front + random.uniform(-1.0, 1.0)
        msg.back = self.sim_back + random.uniform(-1.0, 1.0)
        msg.left = self.sim_left + random.uniform(-0.5, 0.5)
        msg.right = self.sim_right + random.uniform(-0.5, 0.5)
        self.tof_publisher.publish(msg)
        
        # 5. Log the detailed state
        self.get_logger().info(
            f'Sim Pos: [F:{self.sim_front:.1f} L:{self.sim_left:.1f} R:{self.sim_right:.1f}] | '
            f'Last Cmd: [L:{self.last_left_speed:.1f} R:{self.last_right_speed:.1f}]'
        )

    def send_goal(self):
        """Waits for connections and sends the action goal."""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = DriveStraight.Goal()
        goal_msg.front_distance = 100.0 # Target distance

        self.get_logger().info(f'Sending goal request: drive to {goal_msg.front_distance}mm.')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.shutdown()
            return
            
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result from drive_controller: Success = {result.success}')
        self.shutdown()

    def shutdown(self):
        """Cleanly shuts down the node."""
        self.get_logger().info('Test finished, shutting down test node.')
        self.destroy_node()
        # This is a bit of a hack to ensure the process exits in a simple script
        # In a larger application, you'd handle this more gracefully.
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    tester_node = DriveControllerTester()
    try:
        # Give a moment for the publisher and subscriber to connect
        time.sleep(1) 
        tester_node.send_goal()
        rclpy.spin(tester_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            if tester_node and tester_node.executor:
                 tester_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()