import asyncio
import rclpy
import time
from enum import Enum, auto
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from agrobot_interfaces.action import GenericTask, DriveStraight, DriveControl
from agrobot_interfaces.srv import IdentifyEgg
from agrobot_interfaces.msg import ToFData
from threading import RLock
from typing import Any


class State(Enum):
    INIT = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    DRIVE_STRAIGHT = auto()
    DRIVE_BACKWARDS = auto()
    DISPENSE = auto()

class PatchRclpyIssue1123(ActionClient):
    """
    ActionClient patch for rclpy timing issue when multi-threading
    https://github.com/ros2/rclpy/issues/1123
    """

    _lock: RLock = None  # type: ignore

    @property
    def _cpp_client_handle_lock(self) -> RLock:
        if self._lock is None:
            self._lock = RLock()
        return self._lock

    async def execute(self, *args: Any, **kwargs: Any) -> None:
        with self._cpp_client_handle_lock:
            return await super().execute(*args, **kwargs)  # type: ignore

    def send_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super().send_goal_async(*args, **kwargs)

    def _cancel_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._cancel_goal_async(*args, **kwargs)

    def _get_result_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._get_result_async(*args, **kwargs)


class HandleFSM(Node):
    """
    Class for executing the navigating task

    Note: Modified from the BYU Mars Rover Team state machine. (See that for more documentation.)

    :author: Nelson Durrant
    :date: Jun 2025
    
    Publishers:
        - drive/command (agrobot_interfaces/msg/DriveCommand)
    Subscribers:
        - tof/data (sensor_msgs/Range) [norm_callback_group] (TODO: Check this topic name)
            -> mostly just left as an example right now, might not need this in specific
    Clients:
        - egg/identify (agrobot_interfaces/IdentifyEgg) [norm_callback_group]
    Action Clients:
        - control/drive_straight (agrobot_interfaces/DriveStraight) [nested_action_callback_group]
    - TODO: Add more action clients
    Action Servers:
        - control/center (agrobot_interfaces/action/DriveControl)
    """

    def __init__(self):

        super().__init__("handle_fsm")

        #################################
        ### ROS 2 OBJECT DECLARATIONS ###
        #################################

        # Set up publishers

        # PUBlishers
        self.LED_pub = self.create_publisher(Int8, "/LED", 10)
        self.servo_pub = self.create_publisher(ServoCommand, "/servo", 10)
        self.combine_pub = self.create_publisher(Bool, '/combine', 10)
        self.conveyor_pub = self.create_publisher(Bool, '/conveyor', 10)
        self.feeder_pub = self.create_publisher(Bool, '/feeder', 10)
        # self.carriage_pub = self.create_publisher(Bool, '/carriage', 10)
        
        # SUBscribers
        # self.stepper_position
        # self.feeder_position

        # TOF subscriber
        self.tof_subscriber = self.create_subscription(
            ToFData,
            "tof/data", # TODO: Check this topic name
            self.tof_callback,
            10,
            callback_group=norm_callback_group,
        )
        self.tof_subscriber  # prevent unused variable warning

        print("Setting up")
        # Create a timer to call `state_loop` every 0.1 seconds (10 Hz)
        self.state = State.INIT

        self.create_timer(.1, self.run_sort_sm)

        # Callback groups (for threading)
        norm_callback_group = MutuallyExclusiveCallbackGroup()
        nested_action_callback_group = MutuallyExclusiveCallbackGroup()
        action_callback_group = (
            ReentrantCallbackGroup()
        )  # needed to monitor cancel requests

        # service clients
        self.identifyegg = self.create_client(IdentifyEgg, "egg/identify")

        self.drive_straight_client = PatchRclpyIssue1123(
            self,
            DriveStraight,
            "control/drive_straight",
            callback_group=nested_action_callback_group,
        )

        self.turn_client = PatchRclpyIssue1123(
            self,
            DriveStraight,
            "control/drive_straight",
            callback_group=nested_action_callback_group,
        )

        self.turn_action_server = ActionServer(
            self, 
            Turn, 
            'control/turn',
            callback_group=nested_action_callback_group
        )

        # Action server to run the task executor
        self.action_server = ActionServer(
            self,
            GenericTask,
            "exec_handle_fsm",
            self.action_server_callback,
            callback_group=action_callback_group,
            cancel_callback=self.cancel_callback,
        )

        # Initialize variables
        self.name = "handle_fsm"
        self.task_goal_handle = None
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.last_tof_data = None
        self.drive_result = None
        self.egg_in_feeder = False
        self.turn_result = False
        self.drive_result = False

        self.servo_msg = ServoCommand()
        self.flip_timer = None
        self.prev_state = None
        self.init_logged = False

        #####################################
        ### END ROS 2 OBJECT DECLARATIONS ###
        #####################################

        self.get_logger().info("Handle FSM node initialized")

    ###################################
    ### NESTED ACTION HANDLING CODE ###
    ###################################

    async def send_turn_left_goal(self, degrees):
        """
        NOTE: Call this with the asyncio.run() function
        """
        self.get_logger().debug("Waiting for 'Turn Left' action server")
        while not self.center_client.wait_for_server(timeout_sec=1.0):
            self.get_logger.info("'Turn Left' action server not available, waiting...")
        goal_msg = Turn.Goal()
        goal_msg.angle = degrees

        self.get_logger().info("Turning Left until: " + str(degrees))
        send_goal_future = self.center_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        await send_goal_future  # fix for iron/humble threading bug
        self.goal_handle = send_goal_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Turn Left request was rejected!")
            return False
        self.result_future = self.goal_handle.get_result_async()
        await self.result_future

        self.turn_result = self.result_future.result().result
        self.get_logger().info(f"Turn Left result received: success = {self.turn_result.success}")

        return True

    async def send_turn_right_goal(self, degrees):
        """
        NOTE: Call this with the asyncio.run() function
        """
        self.get_logger().debug("Waiting for 'Turn Right' action server")
        while not self.center_client.wait_for_server(timeout_sec=1.0):
            self.get_logger.info("'Turn Right' action server not available, waiting...")
        goal_msg = Turn.Goal()
        goal_msg.angle = degrees

        self.get_logger().info("Turning Right until: " + str(degrees))
        send_goal_future = self.center_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        await send_goal_future  # fix for iron/humble threading bug
        self.goal_handle = send_goal_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Turn Right request was rejected!")
            return False
        self.result_future = self.goal_handle.get_result_async()
        await self.result_future

        self.turn_result = self.result_future.result().result
        self.get_logger().info(f"Turn Right result received: success = {self.turn_result.success}")

        return True

    async def send_drive_straight_goal(self, front_distance):
        """
        Function to drive straight, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        self.get_logger().debug("Waiting for 'DriveStraight' action server")
        while not self.drive_straight_client.wait_for_server(timeout_sec=1.0):
            self.get_logger.info("'DriveStraight' action server not available, waiting...")
        goal_msg = DriveStraight.Goal()
        goal_msg.front_distance = front_distance

        self.get_logger().info("Driving straight until front distance: " + str(front_distance))
        send_goal_future = self.drive_straight_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        await send_goal_future  # fix for iron/humble threading bug
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error("DriveStraight request was rejected!")
            return False

        self.result_future = self.goal_handle.get_result_async()
        await self.result_future

        self.drive_result = self.result_future.result().result
        self.get_logger().info(f"Drive Straight result received: success = {self.drive_result.success}")
        
        return True

    async def cancelTask(self):
        """
        Cancel pending task request of any type, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        self.get_logger().info("Canceling current task.")
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            await future  # fix for iron/humble threading bug
        time.sleep(0.5)  # fix for bug, give time to cancel
        return

    async def isTaskComplete(self):
        """
        Check if the task request of any type is complete yet, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        if not self.result_future:
            # task was cancelled or completed
            return True

        # Fix for iron/humble threading bug (with timeout)
        # https://docs.python.org/3/library/asyncio-task.html#asyncio.wait_for
        try:
            await asyncio.wait_for(self.isTaskCompleteHelper(), timeout=0.1)
        except asyncio.TimeoutError:
            self.get_logger().debug("Timed out waiting for async future to complete")

        if self.result_future.result():
            return True
        else:
            # Timed out, still processing, not complete yet
            return False

    async def isTaskCompleteHelper(self):
        """
        Helper function for async 'wait_for' wrapping
        """

        await self.result_future

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        self.get_logger().debug(f"DriveControl Feedback — forward_error={self.feedback.forward_error}, "
        f"lateral_error={self.feedback.lateral_error}, stability={self.feedback.stability_count}"
        )
        return

    #######################################
    ### END NESTED ACTION HANDLING CODE ###
    #######################################

    ############################
    ### ACTION HANDLING CODE ###
    ############################

    def cancel_callback(self, goal_handle):
        """
        Callback function for the action server cancel request
        """

        self.cancel_flag = True
        return CancelResponse.ACCEPT

    async def async_service_call(self, client, request):
        """
        Fix for iron/humble threading bug - https://github.com/ros2/rclpy/issues/1337
        NOTE: Call this with the asyncio.run() function (and all other async functions)
        """

        future = client.call_async(request)
        await future

        return future.result()

    def action_server_callback(self, goal_handle):
        """
        Callback function for the action server
        """

        result = GenericTask.Result()
        self.task_goal_handle = goal_handle
        self.cancel_flag = False

        try:
            self.run_state_machine()
            result.msg = "We have our heading!"
            self.task_goal_handle.succeed()
        except Exception as e:
            self.task_fatal(str(e))
            result.msg = "**sad jack sparrow noises** (nav is not working)"
            self.task_goal_handle.abort()

        self.task_goal_handle = None
        return result

    ################################
    ### END ACTION HANDLING CODE ###
    ################################

    ###############################
    ### GENERAL ROS 2 CALLBACKS ###
    ###############################

    def tof_callback(self, msg):
        """
        Callback function for the TOF subscriber
        """
        self.last_tof_data = msg
        right = msg.right
        left = msg.left
        front = msg.front
        back = msg.back
        self.get_logger().info(f"I heard: Front={front}, Left={left}, Right={right}, Back={back}")

    ###################################
    ### END GENERAL ROS 2 CALLBACKS ###
    ###################################

    ###############################
    ### TASK FEEDBACK FUNCTIONS ###
    ###############################

    def task_info(self, string):
        """
        Function to write info back to the GenericTask action client
        """

        self.get_logger().info("[" + self.name + "] " + string)
        task_feedback = GenericTask.Feedback()
        task_feedback.status = "[INFO] [" + self.name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    def task_warn(self, string):
        """
        Function to write warnings back to the GenericTask action client
        """

        self.get_logger().warn("[" + self.name + "] " + string)
        task_feedback = GenericTask.Feedback()
        task_feedback.status = "[WARN] [" + self.name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    def task_error(self, string):
        """
        Function to write errors back to the GenericTask action client
        """

        self.get_logger().error("[" + self.name + "] " + string)
        task_feedback = GenericTask.Feedback()
        task_feedback.status = "[ERROR] [" + self.name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    def task_fatal(self, string):
        """
        Function to write fatal errors back to the GenericTask action client
        """

        self.get_logger().fatal("[" + self.name + "] " + string)
        task_feedback = GenericTask.Feedback()
        task_feedback.status = "[FATAL] [" + self.name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    def task_success(self, string):
        """
        Function to write success back to the GenericTask action client
        """

        self.get_logger().info("[" + self.name + "] " + string)
        task_feedback = GenericTask.Feedback()
        task_feedback.status = "[SUCCESS] [" + self.name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    ###################################
    ### END TASK FEEDBACK FUNCTIONS ###
    ###################################

    #########################
    ### General Functions ###
    #########################

    def turn_on_combine(self):
        count = 0
        combine_msg = Bool()

        while count < 50:
        combine_msg.data = True
        self.get_logger().info(f'Combine publishing message: "{combine_msg.data}"')
        self.conveyor_pub.publish(combine_msg)
        count += 1

    def turn_off_combine(self):
        count = 0
        combine_msg = Bool()

        while count < 50:
            combine_msg.data = False
            self.get_logger().info(f'Combine publishing message: "{combine_msg.data}"')
            self.conveyor_pub.publish(combine_msg)
            count += 1
    
    def turn_on_conveyor(self):
        count = 0
        conveyor_msg = Bool()

        while count < 50:
            conveyor_msg.data = True
            self.get_logger().info(f'Conveyor publishing message: "{conveyor_msg.data}"')
            self.conveyor_pub.publish(conveyor_msg)
            count += 1

    def turn_off_conveyor(self):
        count = 0
        conveyor_msg = Bool()

        while count < 50:
            conveyor_msg.data = False
            self.get_logger().info(f'Conveyor publishing message: "{conveyor_msg.data}"')
            self.conveyor_pub.publish(conveyor_msg)
            count += 1

    def LED_alert(self, egg):
        led_msg = Int8()
        count = 0

        while count < 50:
            led_msg.data = egg
            self.get_logger().info(f'LED publishing message: "{led_msg.data}"')
            self.LED_pub.publish(led_msg)
            count += 1

    def move_servo(self):
        return

    def move_feeder_one_position(self):
        return

    def OpenBin(self, egg):
        # Set all servos to 90 (neutral)
        self.servo_msg.servo1 = 90
        self.servo_msg.servo2 = 90
        self.servo_msg.servo3 = 90
        if egg == "Large":
            self.servo_msg.servo1 = 0        # directional servo means 180 is forward, 0 is backward, and 90 does nothing
        elif egg == "Medium":
            self.servo_msg.servo2 = 0
        elif egg == "Bad":
            self.servo_msg.servo3 = 0
        
        self.servo_pub.publish(self.servo_msg)

    def CloseBin(self, egg):
        # Set all servos to 90 (neutral)
        self.servo_msg.servo1 = 90
        self.servo_msg.servo2 = 90
        self.servo_msg.servo3 = 90
        if egg == "Large":
            self.servo_msg.servo1 = 180        # directional servo means 180 is forward, 0 is backward, and 90 does nothing
        elif egg == "Medium":
            self.servo_msg.servo2 = 180
        elif egg == "Bad":
            self.servo_msg.servo3 = 180
        
        self.servo_pub.publish(self.servo_msg)

    def FlipEgg(self):
        
        # Set all servos to 90 (neutral)
        self.servo_msg.servo1 = 90
        self.servo_msg.servo2 = 90
        self.servo_msg.servo3 = 90

        # Flip the egg
        self.servo_msg.servo4 = 180  # positional servo (tune if needed)
        self.servo_pub.publish(self.servo_msg)

        # Start a 2-second timer to reset
        self.flip_timer = self.create_timer(2.0, self._reset_servo)

    def _reset_servo(self):
        # Reset the flipper
        self.servo_msg.servo4 = 0  # adjust if needed
        self.servo_pub.publish(self.servo_msg)

        # Destroy the timer so it doesn't repeat
        self.flip_timer.cancel()
        self.flip_timer = None


    ################################
    ### END OF General Functions ###
    ################################

    #####################
    ### STATE MACHINE ###
    #####################

    def run_state_machine(self):
        """
        Function to run the state machine
        """
        self.complete_flag = False
        self.state = State.INIT

        while not self.complete_flag:
            self.get_logger().info("Transitioned to state: " + str(self.state))
            match self.state:
                case State.INIT:
                    self.handle_init()
                case State.TURN_LEFT:
                    self.turn_left()
                case State.TURN_RIGHT:
                    self.turn_right()
                case State.DRIVE_STRAIGHT():
                    self.drive_straight_forwards()
                case State.DRIVE_BACKWARDS():
                    self.drive_straight_backwards()
                case State.DISPENSE:
                    self.dispense()
                case _:
                    raise Exception("Invalid state: " + str(self.state))

    def handle_init(self):
        """
        Function to handle the initialization state
        """

        self.task_info("Handle task started")

        self.turn_on_combine()

        self.turn_on_conveyor()

        self.state = State.TURN_LEFT

    def turn_left(self):
        self.task_info("Turning Robot LEFT <------")

        self.task_info("Requesting turn left action")
        response = asyncio.run(self.send_turn_left_goal(-90.0))
        while not asyncio.run(self.isTaskComplete()):
            time.sleep(0.1)
            self.task_info("In isTaskComplete loop")
            if self.task_goal_handle.is_cancel_requested:
                asyncio.run(self.cancelTask())
                raise Exception("Task execution canceled by action client")

        if response and self.turn_result.success:
            self.task_success("Robot turned left.")

            self.state = State.DRIVE_STRAIGHT
            self.prev_state = State.TURN_LEFT
        else:
            self.task_warn("Robot did not turn left! Staying in TURN_LEFT state.")

        

    def turn_right(self):
        self.task_info("Turning Robot -------> RIGHT")

        self.task_info("Requesting turn right action")
        response = asyncio.run(self.send_turn_right_goal(90.0))
        while not asyncio.run(self.isTaskComplete()):
            time.sleep(0.1)
            self.task_info("In isTaskComplete loop")
            if self.task_goal_handle.is_cancel_requested:
                asyncio.run(self.cancelTask())
                raise Exception("Task execution canceled by action client")

        if response and self.turn_result.success:
            self.task_success("Robot turned right.")

            self.state = State.DRIVE_STRAIGHT
            self.prev_state = State.TURN_RIGHT
        else:
            self.task_warn("Robot did not turn right! Staying in TURN_RIGHT state.")



    def drive_straight_forwards(self):
        self.task_info("Driving Robot STRAIGHT")

        self.task_info("Requesting drive straight action")
        response = asyncio.run(self.send_drive_straight_goal(0.25))
        while not asyncio.run(self.isTaskComplete()):
            time.sleep(0.1)
            self.task_info("In isTaskComplete loop")
            if self.task_goal_handle.is_cancel_requested:
                asyncio.run(self.cancelTask())
                raise Exception("Task execution canceled by action client")

        if response and self.drive_result.success:
            self.task_success("Robot drove straight.")

            # TODO: Insert egg collection publisher calls

            # This is logic to see what state the robot should move into depending on its previous state.
            if self.prev_state = State.TURN_LEFT:
                self.state = State.TURN_RIGHT
            if self.prev_state = State.TURN_RIGHT:
                self.state = State.DRIVE_BACKWARDS
            else:
                self.state = State.DRIVE_BACKWARDS

            self.prev_state = State.DRIVE_STRAIGHT

        else:
            self.task_warn("Robot did NOT drive straight correctly! Staying in DRIVE_STRAIGHT state.")

    def drive_straight_backwards(self):
        self.task_info("Driving Robot BACKWARDS")

        self.task_info("Requesting drive backwards action")
        response = asyncio.run(self.send_drive_straight_goal(1.1))
        while not asyncio.run(self.isTaskComplete()):
            time.sleep(0.1)
            self.task_info("In isTaskComplete loop")
            if self.task_goal_handle.is_cancel_requested:
                asyncio.run(self.cancelTask())
                raise Exception("Task execution canceled by action client")

        if response and self.drive_result.success:
            self.task_success("Robot drove backwards.")

            self.state = State.DISPENSE
            self.prev_state = State.DRIVE_BACKWARDS

        else:
            self.task_warn("Robot did NOT drive backwards correctly! Staying in DRIVE_BACKWARDS state.")

    def dispense(self):
        # TODO Kick the eggs out with a servo

        self.state = State.DRIVE_STRAIGHT
        self.prev_state = State.DISPENSE


    #########################
    ### END STATE MACHINE ###
    #########################


def main(args=None):
    rclpy.init(args=args)

    state_machine = HandleFSM()
    # Create a multi-threaded node executor for callback-in-callback threading
    executor = MultiThreadedExecutor()
    executor.add_node(state_machine)

    executor.spin()


if __name__ == "__main__":
    main()