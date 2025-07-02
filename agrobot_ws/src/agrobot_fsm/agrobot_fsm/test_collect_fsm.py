import rclpy
import unittest
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from std_msgs.msg import String
from agrobot_interfaces.action import GenericTask
from agrobot_interfaces.srv import IdentifyEgg
from agrobot_fsm.collect_fsm import CollectFSM

class MockEggIdService(Node):
    def __init__(self):
        super().__init__('mock_egg_id_service')
        self.srv = self.create_service(IdentifyEgg, 'egg/identify', self.identify_egg_callback)

    def identify_egg_callback(self, request, response):
        self.get_logger().info('MockEggIdService: Received request')
        response.egg_type = 1  # 1: small, 2: large, 3: bad
        return response

class MockHandleFSM(Node):
    def __init__(self):
        super().__init__('mock_handle_fsm')
        self._action_server = ActionServer(
            self,
            GenericTask,
            'exec_handle_fsm',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('MockHandleFSM: Received goal')
        goal_handle.succeed()
        result = GenericTask.Result()
        return result

class TestCollectFSM(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.collect_fsm = CollectFSM()
        self.mock_egg_id_service = MockEggIdService()
        self.mock_handle_fsm = MockHandleFSM()

        self.executor = rclpy.executors.MultiThreadedExecutor(context=self.context)
        self.executor.add_node(self.collect_fsm)
        self.executor.add_node(self.mock_egg_id_service)
        self.executor.add_node(self.mock_handle_fsm)

    def tearDown(self):
        self.executor.shutdown()
        self.collect_fsm.destroy_node()
        self.mock_egg_id_service.destroy_node()
        self.mock_handle_fsm.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_collect_fsm_execution(self):
        action_client = ActionClient(self.collect_fsm, GenericTask, 'exec_collect_fsm')
        action_client.wait_for_server()

        goal_msg = GenericTask.Goal()
        send_goal_future = action_client.send_goal_async(goal_msg)

        self.executor.spin_until_future_complete(send_goal_future)

        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted, "Goal was not accepted")

        result_future = goal_handle.get_result_async()
        self.executor.spin_until_future_complete(result_future)

        result = result_future.result().result
        self.assertIsNotNone(result, "Did not receive a result")


if __name__ == '__main__':
    unittest.main()