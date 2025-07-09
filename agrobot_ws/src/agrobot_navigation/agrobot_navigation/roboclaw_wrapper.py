import time
import rclpy
from rclpy.node import Node
from roboclaw_3 import Roboclaw
from agrobot_interfaces.msg import DriveCommand

# Roboclaw/Robot Constants
# TODO: Make these parameters?
ROBOCLAW_ADDR = 0x80
ROBOCLAW_BAUD = 115200
# ROBOCLAW_NAME = "/dev/roboclaw"  # From udev rule
# ROBOCLAW_NAME = "/dev/ttyACM0"
ROBOCLAW_NAME = "/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x7A-if00"  # For testing
TIMEOUT_THRESHOLD = 2  # seconds
ROBOCLAW_MAX_CMD_VAL = 127  # Maximum command value for Roboclaw (0-127)


class RoboclawWrapper(Node):
    """
    :author: Nelson Durrant (w Google Gemini 2.5 Pro)
    :date: May 2024

    Node that interfaces with the roboclaw motor controller to drive the robot.

    Subscribers:
        - cmd/drive (agrobot_interfaces/msg/DriveCommand)
    """

    def __init__(self):
        super().__init__("roboclaw_wrapper")

        self.drive_command_sub = self.create_subscription(
            DriveCommand,
            "cmd/drive",
            self.drive_command_callback,
            10,
        )
        self.timer = self.create_timer(0.5, self.timer_callback)

        try:
            self.roboclaw = Roboclaw(ROBOCLAW_NAME, ROBOCLAW_BAUD)
            if self.roboclaw.Open():
                self.get_logger().info("Roboclaw port opened successfully.")
                self.roboclaw.ForwardM1(ROBOCLAW_ADDR, 0)  # left side
                self.roboclaw.ForwardM2(ROBOCLAW_ADDR, 0)  # right side
            else:
                self.get_logger().error(
                    "Failed to open Roboclaw port. MOTORS DISABLED."
                )
                self.roboclaw = None  # Ensure roboclaw is None if not opened
        except Exception as e:
            self.get_logger().error(
                f"Exception opening Roboclaw: {str(e)}. MOTORS DISABLED."
            )
            self.roboclaw = None

        self.last_time_cmd_received = time.time()  # Initialize with current time

        self.get_logger().info(f"RoboclawWrapper initialized")

    def drive_command_callback(self, msg):
        """
        Callback function for the cmd/drive subscriber.
        Receives DriveCommand messages and sends appropriate commands to Roboclaw.

        :param msg: The DriveCommand message.
        :type msg: agrobot_interfaces.msg.DriveCommand
        """

        if self.roboclaw is None:
            self.get_logger().warn("Roboclaw not initialized, ignoring drive_command...")
            return

        left_speed_cmd = msg.left_speed
        right_speed_cmd = msg.right_speed

        self.get_logger().info(
            f"DriveCommand: Left={left_speed_cmd:.2f}, Right={right_speed_cmd:.2f}"
        )

        # Determine direction and prepare command value for Roboclaw for Left Motor (M1)
        if left_speed_cmd >= 0:
            left_command_val = min(int(left_speed_cmd), ROBOCLAW_MAX_CMD_VAL)
            self.roboclaw.ForwardM1(ROBOCLAW_ADDR, left_command_val)
        else:
            left_command_val = min(
                int(abs(left_speed_cmd)), ROBOCLAW_MAX_CMD_VAL
            )
            self.roboclaw.BackwardM1(ROBOCLAW_ADDR, left_command_val)

        # Determine direction and prepare command value for Roboclaw for Right Motor (M2)
        if right_speed_cmd >= 0:
            right_command_val = min(int(right_speed_cmd), ROBOCLAW_MAX_CMD_VAL)
            self.roboclaw.ForwardM2(ROBOCLAW_ADDR, right_command_val)
        else:
            right_command_val = min(
                int(abs(right_speed_cmd)), ROBOCLAW_MAX_CMD_VAL
            )
            self.roboclaw.BackwardM2(ROBOCLAW_ADDR, right_command_val)

        self.get_logger().info(
            f"Roboclaw Cmds: L={left_command_val}, R={right_command_val} (Directions: L_fwd={left_speed_cmd>=0}, R_fwd={right_speed_cmd>=0})"
        )

        self.last_time_cmd_received = time.time()

    def timer_callback(self):
        """
        Timer callback function to check for expired commands.
        If no command is received within TIMEOUT_THRESHOLD, motors are stopped.
        """

        if self.roboclaw is None:
            return

        if time.time() - self.last_time_cmd_received > TIMEOUT_THRESHOLD:
            self.get_logger().warn(
                f"Drive command timeout ({TIMEOUT_THRESHOLD}s), stopping motors."
            )
            self.roboclaw.ForwardM1(ROBOCLAW_ADDR, 0)
            self.roboclaw.ForwardM2(ROBOCLAW_ADDR, 0)
            # Reset last_time_cmd_received to prevent continuous warnings until a new command arrives
            self.last_time_cmd_received = time.time()


def main(args=None):
    rclpy.init(args=args)

    roboclaw_wrapper_node = RoboclawWrapper()
    rclpy.spin(roboclaw_wrapper_node)

    # Shutdown sequence
    if (
        roboclaw_wrapper_node.roboclaw is not None
    ):  # Ensure motors are stopped on shutdown
        roboclaw_wrapper_node.get_logger().info("Shutting down, stopping motors.")
        roboclaw_wrapper_node.roboclaw.ForwardM1(ROBOCLAW_ADDR, 0)
        roboclaw_wrapper_node.roboclaw.ForwardM2(ROBOCLAW_ADDR, 0)

    roboclaw_wrapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
