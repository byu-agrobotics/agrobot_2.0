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
        self.combine_pub = self.create_publisher(Bool, '/combine', 10)
        self.conveyor_pub = self.create_publisher(Bool, '/conveyor', 10)
        self.feeder_pub = self.create_publisher(Bool, '/feeder', 10)
        self.carriage_pub = self.create_publisher(Bool, '/carriage', 10)
        

        # subscribers
        self.eggdetect_sub = self.create_subscription(Bool, '/egg_detect', self.eggdetect_callback, 10)
        self.carriag_pos_sub = self.create_subscription(Float32, '/stepper_position', self.carriage_pos_callback, 10)
        # self.stepper_position
        # self.feeder_position


        # service clients
        self.identifyegg = self.create_client(IdentifyEgg, "egg/identify")

       
        self.servo_msg = ServoCommand()
        self.flip_timer = None
        self.egg_detected = False
        self.prev_state = None
        self.init_logged = False

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
        count = 0

        while count < 50:
            led_msg.data = egg
            self.get_logger().info(f'LED publishing message: "{led_msg.data}"')
            self.LED_pub.publish(led_msg)
            count += 1

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


    # def turn_off_conveyor(self):
    #     count = 0
    #     conveyor_msg = Bool()

    #     while count < 50:
    #         conveyor_msg.data = False
    #         self.conveyor_pub.publish(conveyor_msg)
    #         count += 1

    def turn_off_conveyor(self):
        conveyor_msg = Bool()
        conveyor_msg.data = False

        # Publish once, give time for Teensy to receive and act
        self.conveyor_pub.publish(conveyor_msg)
        rclpy.spin_once(self, timeout_sec=0.01)
        time.sleep(0.1)  # 100 ms delay to ensure it's received

    # def feed_egg(self):
    #     count1 = 0
    #     count2 = 0
    #     feeder_msg = Bool()

    #     while count1 < 50:
    #         feeder_msg.data = True
    #         self.feeder_pub.publish(feeder_msg)
    #         count1 += 1
        
    #     while count2 < 50:
    #         feeder_msg.data = False
    #         self.feeder_pub.publish(feeder_msg)
    #         count2 += 1


    def feed_egg(self):
        feeder_msg = Bool()

        # Send 'True' once (rising edge), then keep it for a short duration
        feeder_msg.data = True
        self.feeder_pub.publish(feeder_msg)
        rclpy.spin_once(self, timeout_sec=0.01)
        time.sleep(0.1)  # 100 ms pulse

        # Then send 'False' to reset (falling edge)
        feeder_msg.data = False
        self.feeder_pub.publish(feeder_msg)
        rclpy.spin_once(self, timeout_sec=0.01)

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
        # self.get_logger().info(f'Eggdetect received message: "{msg.data}"')


    def carriage_pos_callback(self,msg):
        self.carriage_pos = msg.data


    # def send_identifyegg_req(self):
    #     self.get_logger().info("Attempting to call IdentifyEgg service")

    #     while not self.identifyegg.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('IdentifyEgg service not available, waiting again...')

    #     req = IdentifyEgg.Request()
    #     future = self.identifyegg.call_async(req)

    #     self.get_logger().info("Service called, waiting for result...")
    #     rclpy.spin_until_future_complete(self, future)
    #     self.get_logger().info("Finished waiting on future")

    #     if future.result() is not None:
    #         result = future.result()
    #         self.get_logger().info(f'Result egg_type: {result.egg_type}')
    #         return result.egg_type
    #     else:
    #         self.get_logger().error('Service call failed')
    #         return None
        

    def handle_egg_response(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f"Result egg_type: {response.egg_type}")
                self.LED_alert(response.egg_type)
                self.state = State.MOVE_EGG
            else:
                self.get_logger().error("Service call returned None")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


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
        if not self.init_logged:
            self.get_logger().info("Starting conveyor belt and combine")
            self.init_logged = True

        # start conveyor belt
        conveyor_msg = Bool()
        conveyor_msg.data = True
        self.conveyor_pub.publish(conveyor_msg)

        # start combine
        combine_msg = Bool()
        combine_msg.data = True
        self.combine_pub.publish(combine_msg)

        # wait until IR detects an egg in the slot
        if self.egg_detected:
            # turn off conveyor belt
            self.turn_off_conveyor()
            self.init_logged = False
            # feed egg to camera detection spot
            self.feed_egg()
            self.state = State.READ_EGG


    def handle_read_egg(self):
        """
        Function to handle reading the egg using the camera        
        """

        # send request for camera to identify egg
        if not self.init_logged:
            self.get_logger().info("Sending identifyegg request")
            self.init_logged = True

            req = IdentifyEgg.Request()
            future = self.identifyegg.call_async(req)
            future.add_done_callback(self.handle_egg_response)
        
        

    def handle_move_egg(self):
        """
        Function to move the egg to the right bin position
        """

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