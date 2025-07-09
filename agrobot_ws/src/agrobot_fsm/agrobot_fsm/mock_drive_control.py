# Simulates centering using feedback messages defined in DriveControl.action

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Header
from agrobot_interfaces.action import DriveControl
from agrobot_interfaces.msg import ToFData
import time
import random

class MockDriveControlServer(Node):
    def __init__(self):
        super().__init__('mock_drive_control_server')
        self.server = ActionServer(
            self,
            DriveControl,
            'control/center',
            self.execute_callback
        )
        self.get_logger().info("Mock DriveControl server ready.")

    async def execute_callback(self, goal_handle):

        self.get_logger().info(f"Received centering goal")
        
        # Simulated starting error values
        forward_error = 1.7
        lateral_error = 2.4
        stability_count = 0
        
        while stability_count < 5:
            # Simulate small reductions in error
            forward_error = max(0.0, forward_error - random.uniform(0.05, 0.1))
            lateral_error = max(0.0, lateral_error - random.uniform(0.03, 0.08))

        # Check if robot is "stable" (below threshold)
            if forward_error < 0.05 and lateral_error < 0.05:
                stability_count += 1
            else:
                stability_count = 0  # reset if unstable
        
            feedback = DriveControl.Feedback()
            feedback.forward_error = float(forward_error)
            feedback.lateral_error = float(lateral_error)
            feedback.stability_count = int(stability_count)
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                f"[Feedback] fwd_error={forward_error:.2f}, lat_error={lateral_error:.2f}, stable={stability_count}"
            )

            time.sleep(0.5)

        # Once stable long enough, return success
        result = DriveControl.Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().info("Centering complete, robot is stable.")

        return result

def main(args=None):
    rclpy.init(args=args)
    node = MockDriveControlServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
