import rclpy
from rclpy.node import Node
from gpiozero import Servo      # look into most recent 
from agrobot_interfaces.srv import Servo

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Map each servo to a GPIO pin (BCM numbering)
        self.servos = [
            Servo(17),  # GPIO17
            # Servo(18),  # GPIO18
            # Servo(22),  # GPIO22
            # Servo(23),  # GPIO23
            # Servo(24),  # GPIO24
            # Servo(25),  # GPIO25
        ]

        self.srv = self.create_service(Servo, 'set_servos', self.set_servos_callback)

    def set_servos_callback(self, request, response):
        # Example: move all servos to max (True) or min (False)
        value = 1.0 if request.data else -1.0
        for servo in self.servos:
            servo.value = value  # value between -1 (min angle) to 1 (max angle)
        response.success = True
        response.message = f'Servos set to {"max" if request.data else "min"} position.'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
