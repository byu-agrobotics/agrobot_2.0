import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from enum import Enum, auto

from agrobot_interfaces.action import GenericTask

class State(Enum):
    IDLE = auto()
    LOWERING_COLLECTOR = auto()
    POSITIONING = auto()
    EJECTING = auto()
    HOMING = auto()

class HandleFSM(Node):
    """
    HandleFSM controls the robot's egg manipulation hardware.
    NOTE: This is a template. You will need to implement the hardware control logic.
    """
    def __init__(self):
        super().__init__('handle_fsm')
        self.action_server = ActionServer(
            self, GenericTask, 'exec_handle_fsm', self.execute_callback,
            callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_callback
        )
        self.state = State.IDLE
        self.get_logger().info('Handle FSM node is ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing handle egg task...')
        # NOTE: A custom action should be used to pass the egg_type here.
        # For now, we assume a default handling process.
        
        self.state = State.LOWERING_COLLECTOR
        self.run_state_machine()

        goal_handle.succeed()
        result = GenericTask.Result()
        return result
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancelling handle egg task.")
        self.state = State.IDLE
        return CancelResponse.ACCEPT

    def run_state_machine(self):
        while self.state != State.IDLE:
            self.get_logger().info(f"Transitioned to state: {self.state}")
            
            if self.state == State.LOWERING_COLLECTOR:
                # TODO: Implement hardware control to lower the collector
                self.get_logger().info("Lowering collector mechanism...")
                # self.control_actuator("collector", "down")
                self.state = State.POSITIONING
            
            elif self.state == State.POSITIONING:
                # TODO: Implement hardware control for the linear rod
                self.get_logger().info("Positioning egg for sorting...")
                # self.control_actuator("linear_rod", "position_x")
                self.state = State.EJECTING

            elif self.state == State.EJECTING:
                # TODO: Implement hardware control for the ejector servo
                self.get_logger().info("Ejecting egg into storage...")
                # self.control_servo("ejector", "push")
                self.state = State.HOMING

            elif self.state == State.HOMING:
                # TODO: Implement hardware control to home all mechanisms
                self.get_logger().info("Homing all mechanisms...")
                # self.control_actuator("collector", "up")
                # self.control_actuator("linear_rod", "home")
                self.state = State.IDLE

        self.get_logger().info("Handling task complete.")

def main(args=None):
    rclpy.init(args=args)
    handle_fsm = HandleFSM()
    rclpy.spin(handle_fsm)

if __name__ == '__main__':
    main()