# Created by Alyssa Fielding
# Based on the mars rover state machine 
# Designed to work independently as a sort task
# This state machine should control the conveyr belt going into the black box for the camera detection,
# the camera detection itself, controlling the LED color indiciating the type of egg, and sorting the eggs
# into the different bins 

import asyncio
import rclpy
# import tf2_geometry_msgs
# import tf2_ros
import time
# import utm
# from action_msgs.msg import GoalStatus
# from aruco_opencv_msgs.msg import ArucoDetection
# from builtin_interfaces.msg import Duration
from enum import Enum, auto
# from geometry_msgs.msg import Pose, PoseStamped
# from lifecycle_msgs.srv import ChangeState, GetState
# from lifecycle_msgs.msg import Transition
# from nav2_msgs.action import FollowWaypoints, Spin
# from nav2_simple_commander.robot_navigator import TaskResult
# from rclpy.action import ActionServer, ActionClient, CancelResponse
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
# from rclpy.task import Future
# from rover_interfaces.action import AutonomyTask
# from sensor_msgs.msg import NavSatFix
from agrobot_interfaces.msg import ServoCommand
from agrobot_interfaces.srv import IdentifyEgg
from std_msgs.msg import Bool, Int8
# from std_srvs.srv import Trigger, SetBool
# from threading import RLock
# from typing import Any
# from zed_msgs.msg import ObjectsStamped
# from rover_navigation.utils.gps_utils import (
#     latLonYaw2Geopose,
#     meters2LatLon,
#     latLon2Meters,
# )
# from rover_navigation.utils.plan_utils import (
#     basicPathPlanner,  # plan a straight path between two GPS coordinates
#     basicOrderPlanner,  # use brute force to find the best order of legs (based on distance)
# )
# from rover_navigation.utils.terrain_utils import (
#     terrainPathPlanner,  # plan a path between two GPS coordinates using terrain data
# )


class State(Enum):
    INIT = auto()           # initializes the conveyor belt, turns on the camera
    READ_EGG = auto()       # camera reads the egg
    MOVE_EGG = auto()       # moves the linear actuator to the correct position for the egg to be flipped
    SORT_EGG = auto()       # flips the egg
    RESET = auto()          # resets the linear actuator for the egg, inserts a new egg to be read 


class Result(Enum):
    # FOUND = auto()
    # SUCCEEDED = auto()
    # FAILED = auto()
    pass


class SortFSM(Node):
    """
    Class for executing the sort task using ROS2

    :author: Alyssa Fielding
    :date: Jun 2025

    Subscribers:
    - 

    Publishers:
    - /servo (agrobot_interfaces/msg/ServoCommand)
    - /LED (agrobot_interfaces/msg/LEDCommand)
    """

    def __init__(self):

        super().__init__("sort_fsm")

        #################################
        ### ROS2 OBJECT DECLARATIONS ###
        #################################
        # Set up publishers

        # publishers
        self.LED_pub = self.create_publisher(Int8, "/LED", 10)
        self.servo_pub = self.create_publisher(ServoCommand, "/servo", 10)
        # self.combine_pub = self.create_publisher(Bool, '/combine', 10)
        self.conveyor_pub = self.create_publisher(Bool, '/conveyor', 10)
        self.feeder_pub = self.create_publisher(Bool, '/feeder', 10)
        # self.carriage_pub = self.create_publisher(Bool, '/carriage', 10)
        

        # subscribers
        self.eggdetect_sub = self.create_subscription(Bool, '/egg_detect', self.eggdetect_callback, 10)
        # self.stepper_position
        # self.feeder_position


        # service clients
        self.identifyegg = self.create_client(IdentifyEgg, "egg/identify")

       
        self.servo_msg = ServoCommand()
        self.flip_timer = None
        self.egg_detected = False
        self.prev_state = None

        print("Setting up")
        # Create a timer to call `state_loop` every 0.1 seconds (10 Hz)
        self.state = State.INIT

        self.create_timer(.1, self.run_sort_sm)

        # # TODO: Make a launch that ensure the IdentifyEgg script is going in agrobot_perception
        # # Set up clients
        # self.egg_id_client = self.create_client(IdentifyEgg, 'egg/identify')
        # # Wait for the service to be available
        # while not self.egg_id_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for egg identification service...')

        #####################################
        ### END ROS2 OBJECT DECLARATIONS ###
        #####################################

    #######################################
    ### HELPER FUNCTIONS ###
    #######################################

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

    def LED_alert(self, egg):
        led_msg = Int8()
        if egg == "Large":
            led_msg.data = 2       # 1: small, 2: large, 3: bad
        elif egg == "Medium":
            led_msg.data = 1
        elif egg == "Bad":
            led_msg.data = 3
        self.get_logger().info(f'LED publishing message: "{led_msg.data}"')
        self.LED_pub.publish(led_msg)

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

    # def identify_egg(self):
    #     request = IdentifyEgg.Request()

    #     # Send the request asynchronously
    #     future = self.egg_id_client.call_async(request)

    #     # Optional: spin until the service completes
    #     rclpy.spin_until_future_complete(self, future)

    #     if future.result() is not None:
    #         egg_type = future.result().egg_type
    #         if egg_type == 0:
    #             self.get_logger().info("Received result: No egg detected")
    #         elif egg_type == 1:
    #             self.get_logger().info("Received result: Small Egg")
    #         elif egg_type == 2:
    #             self.get_logger().info("Received result: Large Egg")
    #         elif egg_type == 3:
    #             self.get_logger().info("Received result: Bad Egg")
    #         else:
    #             self.get_logger().warn(f"Unknown egg_type: {egg_type}")
    #         return egg_type
    #     else:
    #         self.get_logger().error('Service call failed')
    #         return None

    ###########################################
    ### END HELPER FUNCTIONS ###
    ###########################################

    #######################
    ### ROS2 CALLBACKS ###
    #######################

    def eggdetect_callback(self, msg):
        self.egg_detected = msg.data
        self.get_logger().info(f'Eggdetect received message: "{msg.data}"')


    def send_identifyegg_req(self):
        while not self.identifyegg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IdentifyEgg service not available, waiting again...')

        req = IdentifyEgg.Request()

        future = self.identifyegg.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Result: {future.result()}')
            return future.result()
        else:
            self.get_logger().error('Service call failed')
            return None


    ###########################
    ### END ROS2 CALLBACKS ###
    ###########################

    #####################
    ### STATE MACHINE ###
    #####################

    def run_sort_sm(self):
        """
        Function to run the state machine
        """
        if self.state != self.prev_state:
            self.get_logger().info("Transitioned to state: " + str(self.state))
            self.prev_state = self.state

        match self.state:
            case State.INIT:
                self.handle_init()
            case State.READ_EGG:
                self.handle_read_egg()
            case State.MOVE_EGG:
                self.handle_move_egg()
            case State.SORT_EGG:
                self.handle_sort_egg()
            case State.RESET:
                self.handle_reset()
            case _:
                raise Exception("Invalid state: " + str(self.state))
            
        # self.state = State.SORT_EGG


    def handle_init(self):
        """
        Function to handle the initialization state
        initializes the conveyor belt, turns on the camera
        """
        self.get_logger().info("Starting conveyor belt and combine")
        # Start combine belt
        # combine_msg = Bool()
        # combine_msg.data = True
        # self.combine_pub.publish(combine_msg)

        # Start conveyor belt
        conveyor_msg = Bool()
        conveyor_msg.data = True
        self.conveyor_pub.publish(conveyor_msg)

        # wait until an egg is detected
        if self.egg_detected:
            # pause conveyor belt
            conveyor_msg.data = False
            self.conveyor_pub.publish(conveyor_msg)
            # read the egg
            self.state = State.READ_EGG

        
        # egg_type = self.identify_egg()
        # if egg_type == 0:
        #     # this means it is empty, and egg needs to be loaded into the camera area
        #     # TODO: ned to make it such that if the egg isn't detected properly, but the area is there that it returns
        #     # something before another egg gets kicked into the box
        #     pass
        # else:
        #     self.state = State.READ_EGG

        # self.state = State.RED_EGG

    def handle_read_egg(self):
        """
        Function to handle reading the egg using the camera        
        """
        self.get_logger().info("Sendiing identifyegg request")
        egg_type = self.send_identifyegg_req()
        # egg_type = "Large"
        # egg_type = self.identify_egg()
        # self.moving_egg = egg_type
        self.LED_alert(egg_type)
        # self.state = State.MOVE_EGG

    def handle_move_egg(self):
        """
        Function to move the egg to the right bin position
        """
        # feed the egg into the carriage
        feeder_msg = Bool()
        feeder_msg.data = True
        self.feeder_pub.publish(feeder_msg)

        # TODO: wait either on a timer OR wait on the hall effect sensor

        # move the carriage
        carriage_msg = Bool()
        carriage_msg.data = True
        self.carriage_pub.publish(carriage_msg)

        # if self.moving_egg == 1:
        #     # move linear actuator to position 1
        #     pass
        # elif self.moving_egg == 2:
        #     # move linear actuator to position 2
        #     pass
        # elif self.moving_egg == 3:
        #     # move linear actuator to position 3
        #     pass
        # else:
        #     # assume the egg is bad, sort to position 3
        #     pass

        # TODO: ensure that the linear actuator got to the right position, either through timers or something else?
        # self.state = State.SORT_EGG

    def handle_sort_egg(self):
        """
        Function to move the egg into the bin
        """
        self.FlipEgg() 

        # self.state = State.RESET
        

    def handle_reset(self):
        """
        Function to reset the linear actuator and kick in a new egg
        """

        print("Completed flip")

        # # TODO: Reset the linear actuator, probably will need to include a limit switch to ensure it is homed
        
        # # start spinning the motor to put an egg in 
        # rotation = 0
        # egg_type = self.identify_egg()

        # while egg_type == 0:
        #     self.get_logger().info(f"No egg detected. Rotation attempt {rotation}. Retrying in 1s...")
        #     time.sleep(1)  #  Wait 1 second
        #     rotation += 1
        #     egg_type = self.identify_egg()  # Call service again
        #     # TODO: add it such that if it isn't empty, don't icnreae rotation, it doesn't have to be fully detected yet

        # self.get_logger().info(f"Egg detected: type {egg_type}. Proceeding to next state.")
        # self.state = State.READ_EGG()
        

    #########################
    ### END STATE MACHINE ###
    #########################


def main(args=None):
    rclpy.init(args=args)

    sort_fsm = SortFSM()
    # Create a multi-threaded node executor for callback-in-callback threading
    executor = MultiThreadedExecutor()
    executor.add_node(sort_fsm)

    executor.spin()


if __name__ == "__main__":
    main()