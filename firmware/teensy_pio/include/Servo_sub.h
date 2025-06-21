#ifndef SERVO_SUB
#define SERVO_SUB

#include "subscription.h"
#include <Wire.h>
#include <agrobot_interfaces/msg/servo_command.h>



class ServoSub {
public:
  void setup(rcl_node_t* node, rclc_executor_t* executor);

  float get_servo1_angle();
  float get_servo2_angle();
  float get_servo3_angle();
  float get_servo4_angle();

private:
  rcl_subscription_t subscriber;
  agrobot_interfaces__msg__ServoCommand msg;

  static void callback(const void* msgin);

  static float servo1_angle;
  static float servo2_angle;
  static float servo3_angle;
  static float servo4_angle;
};

#endif  // SERVO_SUB_H