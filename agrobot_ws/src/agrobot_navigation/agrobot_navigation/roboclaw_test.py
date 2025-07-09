import time
from roboclaw_3 import Roboclaw

# Raspberry Pi serial port where the Roboclaw is connected
# This is typically /dev/ttyACM0 or /dev/ttyS0
# To find the correct port, you can run `ls /dev/tty*` in the terminal
# before and after plugging in the Roboclaw.
SERIAL_PORT = "/dev/ttyACM1"  
BAUD_RATE = 115200
ROBOCLAW_ADDRESS = 0x80  # Default Roboclaw address

# Create a Roboclaw object
roboclaw = Roboclaw(SERIAL_PORT, BAUD_RATE)

# Start communication
if not roboclaw.Open():
    print("Error: Could not open serial port.")
    exit()

try:
    print("Roboclaw communication established.")
    print("Running motor 1 forward...")
    roboclaw.ForwardM1(ROBOCLAW_ADDRESS, 64)  # 64 is half speed
    time.sleep(2)
    roboclaw.ForwardM1(ROBOCLAW_ADDRESS, 0)  # Stop motor 1
    time.sleep(1)

    print("Running motor 1 backward...")
    roboclaw.BackwardM1(ROBOCLAW_ADDRESS, 70)
    time.sleep(3)
    roboclaw.BackwardM1(ROBOCLAW_ADDRESS, 0)  # Stop motor 1
    time.sleep(1)

    print("Running motor 2 forward...")
    roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 64)
    time.sleep(2)
    roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 0)  # Stop motor 2
    time.sleep(1)

    print("Running motor 2 backward...")
    roboclaw.BackwardM2(ROBOCLAW_ADDRESS, 64)
    time.sleep(2)
    roboclaw.BackwardM2(ROBOCLAW_ADDRESS, 0)  # Stop motor 2
    time.sleep(1)

    print("Test complete.")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Stop all motors
    roboclaw.ForwardM1(ROBOCLAW_ADDRESS, 0)
    roboclaw.ForwardM2(ROBOCLAW_ADDRESS, 0)
    print("Motors stopped.")
