# mock_drive_straight.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Header
from agrobot_interfaces.action import DriveStraight
from agrobot_interfaces.msg import ToFData
import time

class MockDriveStraightServer(Node):
    def __init__(self):
        super().__init__('mock_drive_straight_server')
        self.server = ActionServer(
            self,
            DriveStraight,
            'control/drive_straight',
            self.execute_callback
        )
        self.get_logger().info("Mock DriveStraight server ready.")

        self._tof_pub = self.create_publisher(ToFData, 'tof/data', 10)

        self.get_logger().info('Mock DriveStraight server with TOF simulation started.')

    async def execute_callback(self, goal_handle):
        target_distance = goal_handle.request.front_distance
        self.get_logger().info(f"Received drive goal: {goal_handle.request.front_distance}")
        front_distance = 5.0
        
        while front_distance > target_distance:
            msg = ToFData()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'tof_sim'

            msg.front = round(front_distance, 2)
            msg.back = 0.5
            msg.left = 0.4
            msg.right = 0.6

            self._tof_pub.publish(msg)
            self.get_logger().info(f"Simulated TOF: front={msg.front}")

            # Simulate forward motion
            front_distance -= 0.9  # getting closer
            time.sleep(0.5)

        # Final publish (target reached)
        msg = ToFData()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tof_sim'
        msg.front = front_distance
        msg.back = 0.5
        msg.left = 0.4
        msg.right = 0.6
        self._tof_pub.publish(msg)

        result = DriveStraight.Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().info("DriveStraight simulation complete.")
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MockDriveStraightServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
