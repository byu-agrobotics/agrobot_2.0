/**
 * @file main.cpp
 * @author Nelson Durrant
 * @date September 2024
 *
 * This node is the micro-ROS node for the agrobot. It controls the actuators
 * and LEDs and reads the sensor data (TOF sensors, etc). The node communicates
 * with the Raspberry Pi over micro-ROS.
 *
 * Subscribes:
 * - TODO: Add other subscribers
 *
 * Publishes:
 * - battery_status (agrobot_interfaces/msg/BatteryStatus)
 * - TODO: Add other publishers
 *
 *
 * IMPORTANT! For an example of a larger micro-ROS project that follows this
 * approach, see: https://github.com/BYU-FRoSt-Lab/cougars-teensy.git
 */

#include "battery_pub.h"
#include "tof_pub.h"
#include "DFRobot_TMF8x01.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#include <FastLED.h>
#include "agrobot_interfaces/msg/servo_command.h"
#include "agrobot_interfaces/msg/led_command.h"
#include "std_msgs/msg/bool.h"


// #include <frost_interfaces/msg/u_command.h>

// #define ENABLE_ACTUATORS
// #define ENABLE_TOF_SENSORS
#define ENABLE_LED
// #define ENABLE_BATTERY
#define ENABLE_BT_DEBUG
#define ENABLE_SERVOS

#define EN1       2                      // EN pin for left TMF8801
#define EN2       3                      // EN pin for right TMF8801
#define EN3       4                      // EN pin for front TMF8801
#define EN4       5                      // EN pin for back TMF8801
#define INT      -1                      // INT pin is floating, not used in this demo

#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

// micro-ROS config values
#define BAUD_RATE 6000000
#define CALLBACK_TOTAL 5
#define SYNC_TIMEOUT 1000

// hardware pin values
#define BT_MC_RX 34
#define BT_MC_TX 35
#define VOLT_PIN 18
#define CURRENT_PIN 17
// #define LED_PIN 13 // Built-in Teensy LED
#define SERVO_PIN1 2
#define SERVO_PIN2 3
#define SERVO_PIN3 4
#define SERVO_PIN4 23
#define LED_PIN 22
#define NUM_LEDS 64
#define BRIGHTNESS  32
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

// default servo positions
#define DEFAULT_SERVO 90

// servo conversion values
#define SERVO_OUT_HIGH 2500
#define SERVO_OUT_LOW 500

// sensor baud rates
#define BT_DEBUG_RATE 9600

// sensor update rates
#define BATTERY_MS 1000 // arbitrary
#define TOF_MS 500     // arbitrary

// time of last received command (used as a fail safe)
unsigned long last_received = 0;

// TOF Calibration data
uint8_t caliDataBuf[14] = {0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// message objects
agrobot_interfaces__msg__ServoCommand servo_msg;
agrobot_interfaces__msg__LEDCommand LED_msg;
agrobot_interfaces__msg__LEDCommand LED_msg1;
std_msgs__msg__Bool combine_msg;

// subscriber objects
rcl_subscription_t combine_sub;
rcl_subscription_t servo_sub;
rcl_subscription_t LED_sub;
rcl_subscription_t LED_sub1;

// publisher objects
BatteryPub battery_pub;

// TOF publisher object
TofPub tof_pub;
 

// sensor objects
SoftwareSerial BTSerial(BT_MC_RX, BT_MC_TX);
DFRobot_TMF8801 tofLeft(/*enPin =*/EN1, /*intPin=*/INT);
DFRobot_TMF8801 tofRight(/*enPin =*/EN2, /*intPin=*/INT);
DFRobot_TMF8801 tofFront(/*enPin =*/EN3, /*intPin=*/INT);
DFRobot_TMF8801 tofBack(/*enPin =*/EN4, /*intPin=*/INT);

// global values for ToF sensor
float left_distance = -1;
float right_distance = -1;
float front_distance = -1;
float back_distance = -1;

// servo objects
Servo myServo1; // Large egg
Servo myServo2; // Small egg
Servo myServo3; // Bad egg
Servo myServo4; // sorting servo

unsigned long lastChange = 0;
const unsigned long interval = 3000;  // 3 seconds
bool forward = true;


// states for state machine in loop function
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} static state;

void error_loop() {
  while (1) {
    delay(10000);

#ifdef ENABLE_BT_DEBUG
    BTSerial.println("[ERROR] In error loop");
#endif // ENABLE_BT_DEBUG
  }
}


// Add a helper macro for debug prints if enabled
#ifdef ENABLE_BT_DEBUG
  #define DBG_PRINT(msg) BTSerial.println(msg)
  #define DBG_PRINTF(fmt, ...) BTSerial.printf(fmt "\n", ##__VA_ARGS__)
#else

  #define DBG_PRINT(msg)
  #define DBG_PRINTF(fmt, ...)
#endif


// Add debug print in callback to verify it triggers
void servo_sub_callback(const void *servo_msgin) {
  DBG_PRINT("[CALLBACK] servo_sub_callback triggered");

  last_received = millis();

  const agrobot_interfaces__msg__ServoCommand *servo_msg =
      (const agrobot_interfaces__msg__ServoCommand *)servo_msgin;

#ifdef ENABLE_SERVOS
  myServo1.write(servo_msg->servo1);
  myServo2.write(servo_msg->servo2);
  myServo3.write(servo_msg->servo3);
  myServo4.write(servo_msg->servo4);
#endif

  DBG_PRINTF("[CALLBACK] Received servo commands: %d %d %d %d",
             servo_msg->servo1, servo_msg->servo2, servo_msg->servo3,
             servo_msg->servo4);
}


void LED_sub_callback1(const void *LED_msgin) {
  DBG_PRINT("[CALLBACK] LED_sub_callback triggered");
  CRGB color;
  last_received = millis();

  const agrobot_interfaces__msg__LEDCommand *LED_msg =
      (const agrobot_interfaces__msg__LEDCommand *)LED_msgin;

  DBG_PRINT("In sub3 callback");

}


void LED_sub_callback(const void *LED_msgin) {
  DBG_PRINT("[CALLBACK] LED_sub_callback triggered");
  CRGB color;
  last_received = millis();

  const agrobot_interfaces__msg__LEDCommand *LED_msg =
      (const agrobot_interfaces__msg__LEDCommand *)LED_msgin;

#ifdef ENABLE_LED
  if (LED_msg->command == 1) {
    DBG_PRINTF("[CALLBACK] Received LED command: %d",
             LED_msg->command);
    color = CRGB::Green;
  } 
  else if (LED_msg->command == 2) {
    DBG_PRINTF("[CALLBACK] Received LED command: %d",
             LED_msg->command);
    color = CRGB::Blue;
  }
  else if (LED_msg->command == 3) {
    DBG_PRINTF("[CALLBACK] Received LED command: %d",
             LED_msg->command);
    color = CRGB::Red;
  }
  else {
    DBG_PRINTF("[CALLBACK] Received LED command: %d",
             LED_msg->command);
    color = CRGB::Black;
  }


  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
  delay(100);  // update rate
#endif
}



void combine_sub_callback(const void *combine_msgin) {
  DBG_PRINT("[CALLBACK] combine_sub_callback triggered");
  CRGB color;
  last_received = millis();

  const std_msgs__msg__Bool *combine_msg =
      (const std_msgs__msg__Bool *)combine_msgin;

  if (combine_msg->data) {
    DBG_PRINTF("[CALLBACK] Received combine command: %d",
             combine_msg->data);
    color = CRGB::Green;
  } 
  else{
    DBG_PRINTF("[CALLBACK] Received combine command: %d",
             combine_msg->data);
    color = CRGB::Blue;
  }

  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
  delay(100);  // update rate

}



bool create_entities() {
  DBG_PRINT("[CREATE_ENTITIES] Starting micro-ROS entities creation");

  allocator = rcl_get_default_allocator();
  rcl_ret_t rc;

  rc = rclc_support_init(&support, 0, NULL, &allocator);
  DBG_PRINTF("[CREATE_ENTITIES] rclc_support_init returned: %d", rc);
  RCCHECK(rc);

  rc = rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support);
  DBG_PRINTF("[CREATE_ENTITIES] rclc_node_init_default returned: %d", rc);
  RCCHECK(rc);

  DBG_PRINT("[CREATE_ENTITIES] Synchronizing timestamps with agent");
  rc = rmw_uros_sync_session(SYNC_TIMEOUT);
  DBG_PRINTF("[CREATE_ENTITIES] rmw_uros_sync_session returned: %d", rc);

  if (!rmw_uros_epoch_synchronized()) {
    DBG_PRINT("[CREATE_ENTITIES][ERROR] Timestamp sync failed!");
  } else {
    DBG_PRINT("[CREATE_ENTITIES] Timestamps synchronized with agent");
  }

//   battery_pub.setup(node);
//   tof_pub.setup(node);



  // setting up subscribers 

  rcl_ret_t rc1, rc2, rc3;

  // DBG_PRINT("[CREATE_ENTITIES] Before servo_sub");
  rc1 = rclc_subscription_init_default(
      &servo_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, ServoCommand),
      "/servo");
  DBG_PRINTF("[CREATE_ENTITIES] servo_sub returned: %d", rc1);

  DBG_PRINT("[CREATE_ENTITIES] Before led_sub");

  rc2 = rclc_subscription_init_default(
      &LED_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, LEDCommand),
      "/LED");
  DBG_PRINTF("[CREATE_ENTITIES] led_sub returned: %d", rc2);

  // rc3 = rclc_subscription_init_default(
  //     &combine_sub,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  //     "/combine");
  // DBG_PRINTF("[CREATE_ENTITIES] combine_sub returned: %d", rc3);

  rc3 = rclc_subscription_init_default(
      &LED_sub1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, LEDCommand),
      "/LED1");
  DBG_PRINTF("[CREATE_ENTITIES] combine_sub returned: %d", rc3);

  // if (rc1 != RCL_RET_OK || rc2 != RCL_RET_OK || rc3 != RCL_RET_OK) {
  //   DBG_PRINT("[CREATE_ENTITIES][ERROR] One or more subscriptions failed to initialize");
  //   return false;
  // }
  if (rc3 != RCL_RET_OK) {
      DBG_PRINTF("[CREATE_ENTITIES][ERROR] LED_sub1 init failed: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }

  rc = rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL, &allocator);
  RCSOFTCHECK(rc);


  rcl_ret_t a1 = rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg,
                                              &servo_sub_callback, ON_NEW_DATA);
  DBG_PRINTF("[CREATE_ENTITIES] Added servo_sub to executor: %d", a1);


  rcl_ret_t a2 = rclc_executor_add_subscription(&executor, &LED_sub, &LED_msg,
                                              &LED_sub_callback, ON_NEW_DATA);
  DBG_PRINTF("[CREATE_ENTITIES] Added LED_sub to executor: %d", a2);


  // rcl_ret_t a3 = rclc_executor_add_subscription(&executor, &combine_sub, &combine_msg,
  //                                             &combine_sub_callback, ON_NEW_DATA);
  // DBG_PRINTF("[CREATE_ENTITIES] Added combine_sub to executor: %d", a3);

  rcl_ret_t a3 = rclc_executor_add_subscription(&executor, &LED_sub1, &LED_msg1,
                                              &LED_sub_callback1, ON_NEW_DATA);
  DBG_PRINTF("[CREATE_ENTITIES] Added combine_sub to executor: %d", a3);



  if (a2 != RCL_RET_OK || a3 != RCL_RET_OK) {
    DBG_PRINT("[CREATE_ENTITIES][ERROR] Failed to add one or more subscriptions to executor");
    return false;
  }

  DBG_PRINT("[CREATE_ENTITIES] micro-ROS entities created successfully");
  return true;




//     DBG_PRINT("[CREATE_ENTITIES] Before servo_sub");
//     rc = rclc_subscription_init_default(
//         &servo_sub,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, ServoCommand),
//         "/servo");
//     DBG_PRINTF("[CREATE_ENTITIES] servo_sub returned: %d", rc);

//     DBG_PRINT("[CREATE_ENTITIES] Before led_sub");
//     rc = rclc_subscription_init_default(
//         &LED_sub,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, LEDCommand),
//         "/LED");
//     DBG_PRINTF("[CREATE_ENTITIES] led_sub returned: %d", rc);

//   RCCHECK(rc);

//   rc = rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL, &allocator);
// //   DBG_PRINTF("[CREATE_ENTITIES] rclc_executor_init returned: %d", rc);
//   RCSOFTCHECK(rc);

//   rc = rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg,
//                                      &servo_sub_callback, ON_NEW_DATA);

//   rc = rclc_executor_add_subscription(&executor, &LED_sub, &LED_msg,
//                                       &LED_sub_callback, ON_NEW_DATA);


//   if (rc != RCL_RET_OK) {
//     DBG_PRINTF("[CREATE_ENTITIES][ERROR] Failed to add subscription: %d", rc);
//   } else {
//     DBG_PRINT("[CREATE_ENTITIES] Subscription added successfully");
//   }










  DBG_PRINT("[CREATE_ENTITIES] micro-ROS entities created successfully");
  return true;
}

void destroy_entities() {
  DBG_PRINT("[DESTROY_ENTITIES] Destroying micro-ROS entities");

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  battery_pub.destroy(node);
  tof_pub.destroy(node);

  if (rcl_node_fini(&node) != RCL_RET_OK) {
    DBG_PRINT("[DESTROY_ENTITIES][WARN] Failed to destroy node");
  }
  rclc_support_fini(&support);

  DBG_PRINT("[DESTROY_ENTITIES] Micro-ROS entities destroyed");
}

void setup() {
  Serial.begin(BAUD_RATE);
  set_microros_serial_transports(Serial);

  pinMode(LED_PIN, OUTPUT);

#ifdef ENABLE_BT_DEBUG
  BTSerial.begin(BT_DEBUG_RATE);
  DBG_PRINT("[SETUP] Bluetooth debug serial started");
  BTSerial.println("Hello from Teensy!");
#endif

#ifdef ENABLE_SERVOS
  pinMode(SERVO_PIN1, OUTPUT);
  pinMode(SERVO_PIN2, OUTPUT);
  pinMode(SERVO_PIN3, OUTPUT);

  myServo1.attach(SERVO_PIN1, SERVO_OUT_LOW, SERVO_OUT_HIGH);
  myServo2.attach(SERVO_PIN2, SERVO_OUT_LOW, SERVO_OUT_HIGH);
  myServo3.attach(SERVO_PIN3, SERVO_OUT_LOW, SERVO_OUT_HIGH);
  myServo4.attach(SERVO_PIN4, SERVO_OUT_LOW, SERVO_OUT_HIGH);

  myServo1.write(DEFAULT_SERVO);
  myServo2.write(DEFAULT_SERVO);
  myServo3.write(DEFAULT_SERVO);
  myServo4.write(DEFAULT_SERVO);

#endif // ENABLE_SERVOS

  // add LEDs
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  state = WAITING_AGENT;
  DBG_PRINT("[SETUP] Initial state: WAITING_AGENT");
}

void loop() {
  // LED blink for status
  if (millis() % 1000 < 250) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  if (millis() - last_received > 50000) {
    last_received = millis();
    DBG_PRINT("[LOOP] No command received in last 5 seconds - fail safe activated");
    // TODO: actuator stop code here
  }

  switch (state) {
  case WAITING_AGENT:
    // DBG_PRINT("[STATE] WAITING_AGENT: Pinging agent...");
    EXECUTE_EVERY_N_MS(500, {
      int ping_res = rmw_uros_ping_agent(100, 1);
      // DBG_PRINTF("[STATE] rmw_uros_ping_agent returned: %d", ping_res);
      state = (ping_res == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
      // DBG_PRINTF("[STATE] Transitioning to state: %d", state);
    });
    break;

  case AGENT_AVAILABLE:
    DBG_PRINT("[STATE] AGENT_AVAILABLE: Creating entities...");
    if (create_entities()) {
      state = AGENT_CONNECTED;
      DBG_PRINT("[STATE] AGENT_CONNECTED");
    } else {
      DBG_PRINT("[STATE] Failed to create entities, destroying and retrying");
      destroy_entities();
      state = WAITING_AGENT;
    }
    break;

  case AGENT_CONNECTED:
    // DBG_PRINT("[STATE] AGENT_CONNECTED: Pinging agent...");
    EXECUTE_EVERY_N_MS(200, {
      int ping_res = rmw_uros_ping_agent(100, 1);
    //   DBG_PRINTF("[STATE] rmw_uros_ping_agent returned: %d", ping_res);
      state = (ping_res == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    //   DBG_PRINTF("[STATE] Agent connectivity state: %d", state);
    });

    if (state == AGENT_CONNECTED) {
#ifdef ENABLE_BATTERY
      EXECUTE_EVERY_N_MS(BATTERY_MS, read_battery());
#endif

#ifdef ENABLE_TOF_SENSORS
      EXECUTE_EVERY_N_MS(TOF_MS, read_tof_sensor());
#endif

    //   DBG_PRINT("[STATE] Spinning executor...");
      rcl_ret_t exec_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    //   DBG_PRINTF("[STATE] rclc_executor_spin_some returned: %d", exec_ret);
    }
    break;

  case AGENT_DISCONNECTED:
    DBG_PRINT("[STATE] AGENT_DISCONNECTED: Destroying entities and returning to WAITING_AGENT");
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    DBG_PRINTF("[STATE][ERROR] Unknown state: %d", state);
    break;
  }
}

