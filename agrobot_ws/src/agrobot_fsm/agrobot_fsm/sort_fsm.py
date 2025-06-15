import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from enum import Enum, auto
import asyncio

from agrobot_interfaces.action import GenericTask

class State(Enum):
    IDLE = auto()
    NAVIGATING_TO_BINS = auto()
    ALIGNING_AND_DUMPING = auto()
    RETURNING_TO_ARENA = auto()

class SortFSM(Node):
    """
    SortFSM controls the final egg sorting and dumping process.
    """
    def __init__(self):
        super().__init__('sort_fsm')
        cb_group = ReentrantCallbackGroup()
        self.navigate_client = ActionClient(self, GenericTask, 'exec_navigate_fsm', callback_group=cb_group)
        self.action_server = ActionServer(
            self, GenericTask, 'exec_sort_fsm', self.execute_callback,
            callback_group=cb_group, cancel_callback=self.cancel_callback
        )
        self.state = State.IDLE
        self.get_logger().info('Sort FSM node is ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing sort eggs task...')
        self.state = State.NAVIGATING_TO_BINS
        self.run_state_machine()

        goal_handle.succeed()
        result = GenericTask.Result()
        return result
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancelling sort task.")
        self.state = State.IDLE
        return CancelResponse.ACCEPT
    
    def run_state_machine(self):
        while self.state != State.IDLE:
            self.get_logger().info(f"Transitioned to state: {self.state}")

            if self.state == State.NAVIGATING_TO_BINS:
                # NOTE: This assumes the navigate_fsm can handle a "bins" target.
                # You will need to implement this logic in the navigate_fsm.
                # asyncio.run(self.navigate_to("bins"))
                self.get_logger().info("Navigation to bins complete (simulated).")
                self.state = State.ALIGNING_AND_DUMPING

            elif self.state == State.ALIGNING_AND_DUMPING:
                for i in range(1, 4):
                    self.get_logger().info(f"Aligning with bin {i}...")
                    # TODO: Implement alignment logic using sensors.
                    
                    self.get_logger().info(f"Dumping eggs for bin {i}...")
                    # TODO: Implement servo control to dump eggs.
                    # self.control_servo(f"dumper_{i}", "dump")
                self.state = State.RETURNING_TO_ARENA

            elif self.state == State.RETURNING_TO_ARENA:
                # asyncio.run(self.navigate_to("arena"))
                self.get_logger().info("Return to arena complete (simulated).")
                self.state = State.IDLE

        self.get_logger().info("Sorting task complete.")

    async def navigate_to(self, target):
        goal_msg = GenericTask.Goal()
        # Custom action would be better here to pass the target
        await self.navigate_client.wait_for_server()
        send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        await send_goal_future

def main(args=None):
    rclpy.init(args=args)
    sort_fsm = SortFSM()
    rclpy.spin(sort_fsm)

if __name__ == '__main__':
    main()