#include "Servo_sub.h"

// Initialize static storage
float ServoSub::servo1_angle = 0.0;
float ServoSub::servo2_angle = 0.0;
float ServoSub::servo3_angle = 0.0;
float ServoSub::servo4_angle = 0.0;

void ServoSub::setup(rcl_node_t* node, rclc_executor_t* executor) {
  RCCHECK(rclc_subscription_init_best_effort(
    &subscription, 
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, ServoCommand),
    "/servo/command"
  ));

  RCCHECK(rclc_executor_add_subscription(
    executor,
    &subscription, 
    &msg,
    &ServoSub::callback,
    ON_NEW_DATA
  ));
}

void ServoSub::callback(const void* msgin) {
  const agrobot_interfaces__msg__ServoCommand* msg = 
    (const agrobot_interfaces__msg__ServoCommand*)msgin;

  servo1_angle = msg->servo1;
  servo2_angle = msg->servo2;
  servo3_angle = msg->servo3;
  servo4_angle = msg->servo4;
}

// Optional getters if used
float ServoSub::get_servo1_angle() { return servo1_angle; }
float ServoSub::get_servo2_angle() { return servo2_angle; }
float ServoSub::get_servo3_angle() { return servo3_angle; }
float ServoSub::get_servo4_angle() { return servo4_angle; }
