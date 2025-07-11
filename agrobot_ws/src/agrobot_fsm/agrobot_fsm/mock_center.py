# Simulates centering using feedback messages defined in DriveControl.action

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Header
from agrobot_interfaces.action import DriveControl, Center
from agrobot_interfaces.msg import ToFData
import time
import random

class MockDriveControlServer(Node):
    def __init__(self):
        super().__init__('mock_drive_control_server')
        self.server = ActionServer(
            self,
            Center,
            'control/center',
            self.execute_callback
        )
        self.get_logger().info("Mock DriveControl server ready.")

    async def execute_callback(self, goal_handle):

        self.get_logger().info(f"Received centering goal")
        
        time.sleep(1.0)

        result = Center.Result()

        result.success = True

        return result

def main(args=None):
    rclpy.init(args=args)
    node = MockDriveControlServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
