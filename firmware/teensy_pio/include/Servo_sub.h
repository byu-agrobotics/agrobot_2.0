#ifndef SERVO_SUB
#define SERVO_SUB

#include "subscriber.h"  // Your base subscriber class
#include <Wire.h>
#include <agrobot_interfaces/msg/servo_command.h>

/**
 * @brief Subscriber for /servo/command topic using agrobot_interfaces/msg/ServoCommand
 */
class ServoSub : public Subscriber {
public:
  void setup(rcl_node_t* node, rclc_executor_t* executor) override;

  float get_servo1_angle();
  float get_servo2_angle();
  float get_servo3_angle();
  float get_servo4_angle();

private:
  agrobot_interfaces__msg__ServoCommand msg;

  static void callback(const void* msgin);

  static float servo1_angle;
  static float servo2_angle;
  static float servo3_angle;
  static float servo4_angle;
};

#endif  // SERVO_SUB
