import asyncio
import rclpy
import time
from enum import Enum, auto
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from agrobot_interfaces.action import GenericTask, DriveStraight
from agrobot_interfaces.srv import IdentifyEgg
from agrobot_interfaces.msg import ToFData
from threading import RLock
from typing import Any


class State(Enum):
    INIT = auto()
    RED = auto()
    GREEN = auto()
    BLUE = auto()


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


class CollectFSM(Node):
    """
    Class for executing the collecting task

    Note: Modified from the BYU Mars Rover Team state machine. (See that for more documentation.)

    :author: Nelson Durrant
    :date: Jun 2025
    
    Publishers:
    - ADD HERE
    Subscribers:
    - tof/data (sensor_msgs/Range) [norm_callback_group] (TODO: Check this topic name)
        -> mostly just left as an example right now, might not need this in specific
    Clients:
    - egg/identify (agrobot_interfaces/IdentifyEgg) [norm_callback_group]
    Action Clients:
    - control/drive_straight (agrobot_interfaces/DriveStraight) [nested_action_callback_group]
    - TODO: Add more action clients
    Action Servers:
    - exec_collect_fsm (agrobot_interfaces/GenericTask) [action_callback_group]
    """

    def __init__(self):

        super().__init__("collect_fsm")

        #################################
        ### ROS 2 OBJECT DECLARATIONS ###
        #################################

        # Callback groups (for threading)
        norm_callback_group = MutuallyExclusiveCallbackGroup()
        nested_action_callback_group = MutuallyExclusiveCallbackGroup()
        action_callback_group = (
            ReentrantCallbackGroup()
        )  # needed to monitor cancel requests

        # TOF subscriber
        self.tof_subscriber = self.create_subscription(
            ToFData,
            "tof/data", # TODO: Check this topic name
            self.tof_callback,
            10,
            callback_group=norm_callback_group,
        )
        self.tof_subscriber  # prevent unused variable warning

        # Client to identify eggs
        self.egg_id_client = self.create_client(
            IdentifyEgg,
            "egg/identify",
            callback_group=norm_callback_group,
        )
        while not self.egg_id_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Egg identification service not available, waiting again..."
            )
        self.egg_id_request = IdentifyEgg.Request()

        # Action client to drive straight
        self.drive_straight_client = PatchRclpyIssue1123(
            self, DriveStraight, "control/drive_straight", callback_group=nested_action_callback_group,
        )

        # Action server to run the task executor
        self.action_server = ActionServer(
            self,
            GenericTask,
            "exec_collect_fsm",
            self.action_server_callback,
            callback_group=action_callback_group,
            cancel_callback=self.cancel_callback,
        )

        # Initialize variables
        self.name = "collect_fsm"
        self.task_goal_handle = None
        self.goal_handle = None
        self.result_future = None
        self.feedback = None

        #####################################
        ### END ROS 2 OBJECT DECLARATIONS ###
        #####################################

        self.get_logger().info("Collect FSM node initialized")

    ###################################
    ### NESTED ACTION HANDLING CODE ###
    ###################################

    async def drive_straight(self, front_distance):
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
        return True

    async def cancelTask(self):
        """
        Cancel pending task request of any type, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        self.self.get_logger().info("Canceling current task.")
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
        self.get_logger().debug("Received action feedback message")
        self.feedback = msg.feedback
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
            result.msg = "Eggggg-cellent"
            self.task_goal_handle.succeed()
        except Exception as e:
            self.task_fatal(str(e))
            result.msg = "** furious chicken noises **"
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

        # TODO: Add here

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

        self.get_logger().info("[" + self.self.name + "] " + string)
        task_feedback = GenericTask.Feedback()
        task_feedback.status = "[SUCCESS] [" + self.self.name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    ###################################
    ### END TASK FEEDBACK FUNCTIONS ###
    ###################################

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
                case State.RED:
                    self.handle_red()
                case State.GREEN:
                    self.handle_green()
                case State.BLUE:
                    self.handle_blue()
                case _:
                    raise Exception("Invalid state: " + str(self.state))

    def handle_init(self):
        """
        Function to handle the initialization state
        """

        self.task_info("Collecting task started")

        # TODO: Add here

        self.state = State.RED

    def handle_red(self):
        """
        TODO: Add here
        """

        self.task_info("RED")

        # Quick examples of how to use actions and services in the state machine
        self.task_info("Requesting drive straight action")
        asyncio.run(self.drive_straight(0.5))
        while not asyncio.run(self.isTaskComplete()):
            time.sleep(0.1)

            if self.task_goal_handle.is_cancel_requested:
                asyncio.run(self.cancelTask())
                raise Exception("Task execution canceled by action client")

        self.task_info("Requesting egg identification service")
        response = asyncio.run(self.async_service_call(self.egg_id_client, self.egg_id_request))
        self.task_info("Egg type identified: " + str(response.egg_type))

        self.state = State.GREEN

    def handle_green(self):
        self.task_info("GREEN")
        self.state = State.BLUE

    def handle_blue(self):
        self.task_info("BLUE")
        self.complete_flag = True

    #########################
    ### END STATE MACHINE ###
    #########################


def main(args=None):
    rclpy.init(args=args)

    state_machine = CollectFSM()
    # Create a multi-threaded node executor for callback-in-callback threading
    executor = MultiThreadedExecutor()
    executor.add_node(state_machine)

    executor.spin()


if __name__ == "__main__":
    main()