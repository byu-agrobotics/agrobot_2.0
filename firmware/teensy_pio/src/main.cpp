/**
 * @file main.cpp
 * @author Nelson Durrant, Brighton Anderson
 * @date September 2024
 *
 * This node is the micro-ROS node for the agrobot. It controls the actuators
 * and LEDs and reads the sensor data (TOF sensors, etc). The node communicates
 * with the Raspberry Pi over micro-ROS.
 *
 * Subscribes:
 * - /stepper_cmd_vel (geometry_msgs/msg/Twist)
 * - /hbridge_motor_one_command (std_msgs/msg/Bool)
 *
 * Publishes:
 * - /tof/data (frost_interfaces/msg/TofData)
 * - /stepper_position (std_msgs/msg/Float32)
 *
 *
 * IMPORTANT! For an example of a larger micro-ROS project that follows this
 * approach, see: https://github.com/BYU-FRoSt-Lab/cougars-teensy.git
 */

// #include "battery_pub.h"
#include "tof_pub.h"
#include "DFRobot_TMF8x01.h"
#include <SoftwareSerial.h>

#include <Servo.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <FastLED.h>
#include "agrobot_interfaces/msg/servo_command.h"
#include "std_msgs/msg/bool.h"
#include <stdbool.h>
#include <std_srvs/srv/trigger.h>

// enable definitions
#define ENABLE_ACTUATORS
#define ENABLE_TOF_SENSORS
// #define ENABLE_BATTERY
#define ENABLE_LED
#define ENABLE_BT_DEBUG
#define ENABLE_STEPPER_1
#define ENABLE_STEPPER_2
#define ENABLE_SERVOS
#define ENABLE_CONVEYOR
#define ENABLE_FEEDER
#define ENABLE_FEEDER_POS
// #define ENABLE_HBRIDGE
#define ENABLE_IR_SENSOR
// #define ENABLE_DCMOTOR


// function set up, pins, and variables

#define LED_PIN 13 // Built-in Teensy LED

#ifdef ENABLE_FEEDER_POS
  #define pinFeederIR 7 // IR sensor pin
  #define IR_MS 500    // Read the IR sensor every 500ms

#endif //ENABLE_FEEDER_POS

#ifdef ENABLE_IR_SENSOR
  #define pinIRd 6 // IR sensor pin
  #define IR_MS 500    // Read the IR sensor every 500ms
#endif // ENABLE_IR_SENSOR

#ifdef ENABLE_TOF_SENSORS
  // TOF enable pins
  #define EN1       2                   // LEFT
  #define EN2       3                   // RIGHT
  #define EN3       4                   // FRONT
  #define EN4       5                   // BACK
  #define INT       -1                  // INT pin is floating, not used in this demo
#endif // ENABLE_TOF_SENSORS

#ifdef ENABLE_STEPPER_1
  // Stepper motors
  #define STEPPER1_STEP_PIN 26
  #define STEPPER1_DIR_PIN 27
  #define STEPS_PER_REV 200
  #define STEPPER2_STEP_PIN 28
  #define STEPPER2_DIR_PIN 29
#endif // ENABLE_STEPPER_1

// L298N1 H-Bridge pins
// #define HBRIDGE_IN1 22
// #define HBRIDGE_IN2 23
// #define HBRIDGE_IN3 24
// #define HBRIDGE_IN4 25
// #define HBRIDGE_ENA 16
// #define HBRIDGE_ENB 17
// Servo pins
// #define SERVO_1_PIN 11 // Pin for servo 1
// #define SERVO_2_PIN 12 // Pin for servo 2
// #define SERVO_3_PIN 14 // Pin for servo 3
// // Big Servo
// #define BIG_SERVO_PIN 29 // Pin for the big servo
// hardware pin values
// #define VOLT_PIN 18
// #define CURRENT_PIN 17
//LED matrix
// #define LED_MOSI_PIN 13 // Pin for the LED matrix (if used)
// #define LED_CLOCK_PIN 38 // Pin for the LED matrix clock (if used)
// #define LED_CS_PIN 28 // Pin for the LED matrix chip select (if used)


#ifdef ENABLE_SERVOS
  #define SERVO_PIN1 9
  #define SERVO_PIN2 10
  #define SERVO_PIN3 12
  #define SERVO_PIN4 23
  // default servo positions
  #define DEFAULT_SERVO 90
  // servo conversion values
  #define SERVO_OUT_HIGH 2500
  #define SERVO_OUT_LOW 500
#endif // ENABLE_SERVOS


#ifdef ENABLE_FEEDER
  bool motor_is_on = false;
  #define DC_IN1 30
#endif // ENABLE_FEEDER


#ifdef ENABLE_CONVEYOR
  // TODO: Add motor driver capabilites here
  const int conveyor_speed = 200;
  #define DC_IN3 31
#endif //ENABLE_CONVEYOR

#ifdef ENABLE_LED
  #define RGB_PIN 11
  #define NUM_LEDS 64
  #define BRIGHTNESS  32
  #define LED_TYPE    WS2812B
  #define COLOR_ORDER GRB
#endif //ENABLE_LED

#ifdef ENABLE_BT_DEBUG
  #define BT_DEBUG_RATE 9600
  #define BT_MC_RX 34
  #define BT_MC_TX 35
#endif // ENABLE_BT_DEBUG



#define EXECUTE_EVERY_N_MS(MS, X)                                             \
  do {                                                                        \
    static volatile int64_t init = -1;                                        \
    if (init == -1) {                                                         \
      init = uxr_millis();                                                    \
    }                                                                         \
    if (uxr_millis() - init > MS) {                                           \
      X;                                                                      \
      init = uxr_millis();                                                    \
    }                                                                         \
  } while (0)

// micro-ROS config values
#define BAUD_RATE 6000000
#define CALLBACK_TOTAL 9
#define SYNC_TIMEOUT 1000

// #define VOLT_PIN 18
// #define CURRENT_PIN 17
#define LED_PIN 13 // Built-in Teensy LED

// sensor update rates
#define BATTERY_MS 1000 // arbitrary
#define TOF_MS 5     // arbitrary


// time of last received command (used as a fail safe)
unsigned long last_received = 0;

// TOF Calibration data
uint8_t caliDataBuf[14] = {0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor; // Added executor declaration


// Subscriber and publisher set up 

// Egg detection publisher
rcl_publisher_t egg_publisher;
std_msgs__msg__Bool egg_msg;

// Stepper motor ROS objects
#ifdef ENABLE_STEPPER_1
  rcl_subscription_t stepper_subscriber;
  rcl_publisher_t stepper_publisher;
  geometry_msgs__msg__Twist twist_msg;
  std_msgs__msg__Float32 position_msg;

  // Add these lines for the new position control
  rcl_subscription_t stepper_position_subscriber;
  std_msgs__msg__Int8 stepper_position_msg;

  volatile float motor_position = 0.0; // In revolutions

  rcl_service_t home_stepper_service;
  std_srvs__srv__Trigger_Request home_stepper_request;
  std_srvs__srv__Trigger_Response home_stepper_response;
#endif

#ifdef ENABLE_STEPPER_2
  rcl_subscription_t stepper2_subscriber;
  std_msgs__msg__Float32 stepper2_speed_msg; // Use a Float32 for speed control
  
  // Stepper 2 state variables
  volatile bool stepper2_is_running = false;
  volatile long stepper2_step_delay = 0; // Delay between steps in microseconds
  unsigned long stepper2_last_step_time = 0;
#endif

#ifdef ENABLE_HBRIDGE
  rcl_subscription_t hbridge_subscriber;
  std_msgs__msg__Bool hbridge_msg;
#endif

// LED subscriber
#ifdef ENABLE_LED
  rcl_subscription_t LED_sub;
  std_msgs__msg__Int8 LED_msg;
  // initialize LEDs
  CRGB leds[NUM_LEDS];
#endif  // ENABLE_LED


// Servo subscriber
#ifdef ENABLE_SERVOS
  agrobot_interfaces__msg__ServoCommand servo_msg;
  rcl_subscription_t servo_sub;

  // servo objects
  Servo myServo1; // Large egg
  Servo myServo2; // Small egg
  Servo myServo3; // Bad egg
  Servo myServo4; // sorting servo

#endif // ENABLE_SERVOS

// Conveyor subscriber
#ifdef ENABLE_CONVEYOR
  std_msgs__msg__Bool conveyor_msg;
  rcl_subscription_t conveyor_sub;

#endif // ENABLE_CONVEYOR

// Feeder subscriber
#ifdef ENABLE_FEEDER
  std_msgs__msg__Bool feeder_msg;
  rcl_subscription_t feeder_sub;
#endif // ENABLE_FEEDER

// // Egg_detect publisher
// #ifdef ENABLE_FEEDER_POS
//   // rcl_publisher_t eggdetect_pub;
//   // std_msgs__msg__Bool egg_msg;
// #endif // ENABLE_FEEDER_POS


// publisher objects
// BatteryPub battery_pub;

// TOF publisher object
TofPub tof_pub;

// IR sensor value
volatile int IRvalueD = 0;

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

// states for state machine in loop function
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} static state;

// Added forward declaration for the callback function
#ifdef ENABLE_STEPPER_1
  void stepper_callback(const void *msgin);
  void stepper_position_callback(const void *msgin);
  void home_stepper_callback(const void *request, void *response);
#endif

#ifdef ENABLE_STEPPER_2
  void stepper2_speed_callback(const void *msgin);
#endif

#ifdef ENABLE_HBRIDGE
  void hbridge_callback(const void *msgin);
#endif

// Helper function to blink the LED a specific number of times
void blink_led(int count, int duration_ms) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration_ms);
    digitalWrite(LED_PIN, LOW);
    delay(duration_ms);
  }
}

#ifdef ENABLE_SERVOS
void servo_sub_callback(const void *servo_msgin) {
  CRGB color;
  last_received = millis();

  const agrobot_interfaces__msg__ServoCommand *servo_msg =
      (const agrobot_interfaces__msg__ServoCommand *)servo_msgin;

  myServo1.write(servo_msg->servo1);
  myServo2.write(servo_msg->servo2);
  myServo3.write(servo_msg->servo3);
  myServo4.write(servo_msg->servo4);

  if (servo_msg->servo4 == 90){
    color = CRGB::Green;
  }
  if (servo_msg->servo4 == 180){
    color = CRGB::Blue;
  }
  if (servo_msg->servo4 == 0){
    color = CRGB::Black;
  }

  fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
    delay(100);  // update rate
}
#endif //ENABLES_SERVOS



#ifdef ENABLE_LED
void LED_sub_callback(const void *LED_msgin) {
  static int current_mode = -1;
  static bool led_on = false;
  static unsigned long last_toggle_time = 0;

  const std_msgs__msg__Int8 *LED_msg =
      (const std_msgs__msg__Int8*)LED_msgin;
  int new_mode = LED_msg->data;
  unsigned long now = millis();

  // Only update mode if it changed
  if (new_mode != current_mode) {
    current_mode = new_mode;
    last_toggle_time = now;
    led_on = false;  // Reset blink state
  }

  CRGB color = CRGB::Black;

  if (current_mode == 1) {
    color = CRGB::Green;
  } else if (current_mode == 2) {
    color = CRGB::Blue;
  } else if (current_mode == 3) {
    color = CRGB::Red;
  } else if (current_mode == 4) {
    // Blink green every 500ms
    if (now - last_toggle_time >= 500) {
      last_toggle_time = now;
      led_on = !led_on;
    }
    color = led_on ? CRGB::Green : CRGB::Black;
  } else {
    color = CRGB::Black;
  }

  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
}
#endif  // ENABLE_LED



#ifdef ENABLE_CONVEYOR
void conveyor_sub_callback(const void *conveyor_msgin) {
  CRGB color;
  last_received = millis();

  const std_msgs__msg__Bool *conveyor_msg =
      (const std_msgs__msg__Bool*)conveyor_msgin;

  if (conveyor_msg->data && !motor_is_on) {
    analogWrite(DC_IN3, conveyor_speed);  // Full speed forward
    motor_is_on = true;
    color = CRGB::Green;
  }

  if (!conveyor_msg->data && motor_is_on) {
    analogWrite(DC_IN3, 0);  // Stop motor
    motor_is_on = false;
    color = CRGB::Black;
  }


  // if (conveyor_msg->data == true) {
  //   color = CRGB::Green;
  //   // analogWrite(DC_IN1, 200);  // Speed for motor A (0–255)
  //   // analogWrite(DC_IN3, 200);  // Speed for motor B
  //   digitalWrite(DC_IN3, HIGH);
  // } 
  // else{
  //   color = CRGB::Black;
  //   // analogWrite(DC_IN1, 0);  // turn off motor
  //   // analogWrite(DC_IN3, 0);  // turn off motor
  //   digitalWrite(DC_IN3, LOW);
  // }
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
  delay(100);  // update rate
}
#endif // ENBLE_CONVEYOR

#ifdef ENABLE_FEEDER
bool last_feeder_state = false;  // tracks previous state

void move_30_degrees() {
  int feeder_ir = digitalRead(pinFeederIR);
  while (feeder_ir == LOW) {
    analogWrite(DC_IN1, conveyor_speed);  // Full speed forward
    feeder_ir = digitalRead(pinFeederIR); // re-check sensor
  }
  analogWrite(DC_IN1, 0);  // stop motor
}

void feeder_sub_callback(const void *feeder_msgin) {
  static bool last_state = false;  // stores last received state
  last_received = millis();

  const std_msgs__msg__Bool *feeder_msg = (const std_msgs__msg__Bool*)feeder_msgin;
  bool current_state = feeder_msg->data;

  if (current_state != last_state) {
    // only trigger action if state changed
    CRGB color = current_state ? CRGB::Green : CRGB::Black;

    if (current_state) {
      move_30_degrees();
    }

    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
  }

  last_state = current_state;  // update state
}

#endif  // ENABLE_FEEDER

void error_loop() {
  while (1) {
    delay(100);

#ifdef ENABLE_BT_DEBUG
    BTSerial.println("[ERROR] In error loop");
#endif // ENABLE_BT_DEBUG
  }
}

/**
 * Creates micro-ROS entities. This function initializes the micro-ROS
 * entities (node, publishers, subscribers, and executor) and synchronizes the
 * timestamps with the Raspberry Pi.
 *
 * @return true if the entities were created successfully, false otherwise
 */
bool create_entities() {

  // the allocator object wraps the dynamic memory allocation and deallocation
  // methods used in micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // synchronize timestamps with the Raspberry Pi
  // after sync, timing should be able to be accessed with "rmw_uros_epoch"
  // functions
  RCCHECK(rmw_uros_sync_session(SYNC_TIMEOUT));

#ifdef ENABLE_BT_DEBUG
  if (!rmw_uros_epoch_synchronized()) {
    BTSerial.println("[ERROR] Could not synchronize timestamps with agent");
  } else {
    BTSerial.println("[INFO] Timestamps synchronized with agent");
  }
#endif // ENABLE_BT_DEBUG



  // Initialize the executor
  RCCHECK(rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL, &allocator));

  // create publishers
  // battery_pub.setup(node);
  //  blink_led(2, 250);

  tof_pub.setup(node);
  // blink_led(3, 250);

#ifdef ENABLE_IR_SENSOR
  RCCHECK(rclc_publisher_init_default(
      &egg_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "egg_detect")); // This is the topic name
#endif // ENABLE_IR_SENSOR

#ifdef ENABLE_STEPPER_1
  // Initialize stepper publisher
  RCCHECK(rclc_publisher_init_default(
    &stepper_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/stepper_position"));

  // Initialize stepper subscriber
  RCCHECK(rclc_subscription_init_default(
    &stepper_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/stepper_cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &stepper_position_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/stepper_position_cmd"));

  // Add the new subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &stepper_position_subscriber, &stepper_position_msg, &stepper_position_callback, ON_NEW_DATA));

  // Initialize the homing service
  RCCHECK(rclc_service_init_default(
    &home_stepper_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "/home_stepper"));

  // Add the service to the executor
  RCCHECK(rclc_executor_add_service(&executor, &home_stepper_service, &home_stepper_request, &home_stepper_response, &home_stepper_callback));

  // Add subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &stepper_subscriber, &twist_msg, &stepper_callback, ON_NEW_DATA));
#endif

#ifdef ENABLE_STEPPER_2
  // Initialize stepper 2 subscriber with a different message type for simple speed control
  RCCHECK(rclc_subscription_init_default(
    &stepper2_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/stepper2_speed")); // Listen on a new topic for speed

  // Add subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &stepper2_subscriber, &stepper2_speed_msg, &stepper2_speed_callback, ON_NEW_DATA));
#endif

#ifdef ENABLE_HBRIDGE
  // Initialize H-bridge subscriber
  RCCHECK(rclc_subscription_init_default(
    &hbridge_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/hbridge_motor_one_command"));

  // Add subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &hbridge_subscriber, &hbridge_msg, &hbridge_callback, ON_NEW_DATA));
#endif
#ifdef ENABLE_LED
  RCCHECK(rclc_subscription_init_default(
    &LED_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/LED"));

  // Add subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &LED_sub, &LED_msg, &LED_sub_callback, ON_NEW_DATA));
#endif // ENABLE_LED

#ifdef ENABLE_SERVOS
  RCCHECK(rclc_subscription_init_default(
      &servo_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, ServoCommand),
      "/servo"));

  RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_sub_callback, ON_NEW_DATA));

#endif // ENABLE_SERVOS

#ifdef ENABLE_CONVEYOR
  RCCHECK(rclc_subscription_init_default(
      &conveyor_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/conveyor"));

  RCCHECK(rclc_executor_add_subscription(&executor, &conveyor_sub, &conveyor_msg, &conveyor_sub_callback, ON_NEW_DATA));
#endif //ENBLE_CONVEYOR

#ifdef ENABLE_FEEDER
  RCCHECK(rclc_subscription_init_default(
      &feeder_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/feeder"));

  RCCHECK(rclc_executor_add_subscription(&executor, &feeder_sub, &feeder_msg, &feeder_sub_callback, ON_NEW_DATA));
#endif //ENBLE_CONVEYOR

// #ifdef ENABLE_EGGDETECT
//   RCCHECK(rclc_publisher_init_default(
//     &eggdetect_pub,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
//     "/egg_detect"));

// #endif //ENABLE_EGGDETECT

#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Micro-ROS entities created successfully");
#endif // ENABLE_BT_DEBUG

  return true;
}

/**
 * Destroys micro-ROS entities. This function destroys the micro-ROS
 * entities (node, publishers, subscribers, and executor).
 */
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // destroy publishers
  // battery_pub.destroy(node);
  tof_pub.destroy(node);

#ifdef ENABLE_HBRIDGE
  rcl_subscription_fini(&hbridge_subscriber, &node);
#endif

  // <<< FIXED: Clean up the executor
  rclc_executor_fini(&executor);

  if (rcl_node_fini(&node) != RCL_RET_OK) {
#ifdef ENABLE_BT_DEBUG
    BTSerial.println("[WARN] Failed to destroy node");
#endif // ENABLE_BT_DEBUG
  }
  rclc_support_fini(&support);

#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Micro-ROS entities destroyed successfully");
#endif // ENABLE_BT_DEBUG
}

/**
 * Sets up the micro-ROS serial transports. This function sets up the
 * micro-ROS serial transports for communication with the Raspberry Pi.
 */
void setup() {

  Serial.begin(BAUD_RATE);
  set_microros_serial_transports(Serial);

  // set up the indicator light
  pinMode(LED_PIN, OUTPUT);

#ifdef ENABLE_IR_SENSOR
  pinMode(pinIRd, INPUT); // Set the IR sensor pin as an input
  pinMode(pinFeederIR, INPUT);
#endif //ENABLE_IR_SENSOR

#ifdef ENABLE_BT_DEBUG
  BTSerial.begin(BT_DEBUG_RATE);
#endif // ENABLE_BT_DEBUG

// #ifdef ENABLE_BATTERY
//   pinMode(CURRENT_PIN, INPUT);
//   pinMode(VOLT_PIN, INPUT);

// #ifdef ENABLE_BT_DEBUG
//   BTSerial.println("[INFO] Battery Sensor enabled");
// #endif // ENABLE_BT_DEBUG
// #endif // ENABLE_BATTERY

#ifdef ENABLE_STEPPER_1
  // Configure stepper pins as outputs
  pinMode(STEPPER1_STEP_PIN, OUTPUT);
  pinMode(STEPPER1_DIR_PIN, OUTPUT);
  #ifdef ENABLE_BT_DEBUG
    BTSerial.println("[INFO] Stepper 1 enabled");
  #endif
#endif

#ifdef ENABLE_STEPPER_2
  // Configure stepper 2 pins as outputs
  pinMode(STEPPER2_STEP_PIN, OUTPUT);
  pinMode(STEPPER2_DIR_PIN, OUTPUT);
  #ifdef ENABLE_BT_DEBUG
    BTSerial.println("[INFO] Stepper 2 enabled");
  #endif
#endif

#ifdef ENABLE_CONVEYOR
  pinMode(DC_IN3, OUTPUT);
#endif // ENABLE_CONVEYOR

#ifdef ENABLE_FEEDER
  pinMode(DC_IN1, OUTPUT);
#endif // ENABLE_FEEDER


#ifdef ENABLE_LED
  FastLED.addLeds<LED_TYPE, RGB_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);  
#endif //ENABLE_LED


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

#ifdef ENABLE_TOF_SENSORS // TODO: Add ifdefs for BTSerial below
  
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(EN4, OUTPUT);

  // Initially set all EN pins LOW
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(EN3, LOW);
  digitalWrite(EN4, LOW);
  delay(50); // Give some time for sensors to reset

  // Initialize first sensor (left)
  digitalWrite(EN1, HIGH);  // Enable left sensor
  delay(50);                // Wait for the sensor to power up

  BTSerial.print("Initializing left TMF8801 sensor...");
  while(tofLeft.begin() != 0) {
    BTSerial.println("failed.");
    delay(1000);
  }
  BTSerial.println("done.");

  BTSerial.println(tofLeft.getUniqueID(), HEX); // check unique id before resetting address

  // Set I2C address for first sensor
  bool addr1 = tofLeft.setI2CAddress(1);  // Set to address 1
  if (addr1){
    BTSerial.println("failed to set address 1");
    delay(10);
  }
  else{BTSerial.println("successfully set address 1!");}

  // Print first sensor info
  BTSerial.println("Left Sensor:");
  BTSerial.print("Software Version: ");
  BTSerial.println(tofLeft.getSoftwareVersion());
  BTSerial.print("Unique ID: ");
  BTSerial.println(tofLeft.getUniqueID(), HEX);
  BTSerial.print("Model: ");
  BTSerial.println(tofLeft.getSensorModel());

  // Initialize second sensor (right)
  digitalWrite(EN2, HIGH);  // Enable right sensor
  delay(50);                // Wait for the sensor to power up

  BTSerial.print("Initializing right TMF8801 sensor...");
  while(tofRight.begin() != 0) {
    BTSerial.println("failed.");
    delay(1000);
  }
  BTSerial.println("done.");

  // Set I2C address for second sensor
  tofRight.setI2CAddress(2);  // Set to address 2

  // Print second sensor info
  BTSerial.println("Right Sensor:");
  BTSerial.print("Software Version: ");
  BTSerial.println(tofRight.getSoftwareVersion());
  BTSerial.print("Unique ID: ");
  BTSerial.println(tofRight.getUniqueID(), HEX);
  BTSerial.print("Model: ");
  BTSerial.println(tofRight.getSensorModel());

  // Initialize third sensor (front)
  digitalWrite(EN3, HIGH);  // Enable front sensor
  delay(50);                // Wait for the sensor to power up

  BTSerial.print("Initializing front TMF8801 sensor...");
  while(tofFront.begin() != 0) {
    BTSerial.println("failed.");
    delay(1000);
  }
  BTSerial.println("done.");

  // Set I2C address for third sensor
  tofFront.setI2CAddress(3);  // Set to address 3

  // Print second sensor info
  BTSerial.println("Front Sensor:");
  BTSerial.print("Software Version: ");
  BTSerial.println(tofFront.getSoftwareVersion());
  BTSerial.print("Unique ID: ");
  BTSerial.println(tofFront.getUniqueID(), HEX);
  BTSerial.print("Model: ");
  BTSerial.println(tofFront.getSensorModel());

  // Initialize fourth sensor (back)
  digitalWrite(EN4, HIGH);  // Enable back sensor
  delay(50);                // Wait for the sensor to power up

  BTSerial.print("Initializing back TMF8801 sensor...");
  while(tofBack.begin() != 0) {
    BTSerial.println("failed.");
    delay(1000);
  }
  BTSerial.println("done.");

  // Set I2C address for fourth sensor
  tofBack.setI2CAddress(4);  // Set to address 3

  // Print Fourth sensor info
  BTSerial.println("Back Sensor:");
  BTSerial.print("Software Version: ");
  BTSerial.println(tofBack.getSoftwareVersion());
  BTSerial.print("Unique ID: ");
  BTSerial.println(tofBack.getUniqueID(), HEX);
  BTSerial.print("Model: ");
  BTSerial.println(tofBack.getSensorModel());

  // Set calibration data for both sensors
  tofLeft.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));
  tofRight.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));
  tofFront.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));
  tofBack.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));

  // Start measurements on both sensors
  tofLeft.startMeasurement(/*cailbMode =*/tofLeft.eModeCalib);
  tofRight.startMeasurement(/*cailbMode =*/tofRight.eModeCalib);
  tofFront.startMeasurement(/*cailbMode =*/tofFront.eModeCalib);
  tofBack.startMeasurement(/*cailbMode =*/tofBack.eModeCalib);

#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] TOF sensors enabled");
#endif // ENABLE_BT_DEBUG
#endif // ENABLE_TOF_SENSORS


  state = WAITING_AGENT;
}

/**
 * Reads the battery sensor data. This function reads the battery sensor
 * data (voltage and current) and publishes it to the micro-ROS agent.
 */
// void read_battery() {
//   blink_led(1, 30);
//   // we did some testing to determine the below params, but
//   // it's possible they are not completely accurate
//   float voltage = (analogRead(VOLT_PIN) * 0.03437) + 0.68;
//   float current = (analogRead(CURRENT_PIN) * 0.122) - 11.95;

//   // publish the battery data
//   // battery_pub.publish(voltage, current);
// }

#ifdef ENABLE_IR_SENSOR
void read_ir_sensor() {
  IRvalueD = digitalRead(pinIRd);

  // A LOW reading means the sensor is triggered (aligned).
  if (IRvalueD == LOW) {
    digitalWrite(LED_PIN, HIGH); // Turn on LED to indicate alignment
    egg_msg.data = true;         // Set message to true (egg detected)
  } else {
    digitalWrite(LED_PIN, LOW);  // Turn off LED
    egg_msg.data = false;        // Set message to false (no egg)
  }

  // Publish the message to the "egg_detected" topic
  RCSOFTCHECK(rcl_publish(&egg_publisher, &egg_msg, NULL));

  #ifdef ENABLE_BT_DEBUG
    BTSerial.print("Egg detected: ");
    BTSerial.println(egg_msg.data ? "true" : "false");
  #endif
}
#endif // ENABLE_IR_SENSOR

#ifdef ENABLE_STEPPER_1
void stepper_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;

  // Control disrection with angular.z
  if (twist_msg->angular.z > 0) {
    digitalWrite(STEPPER1_DIR_PIN, HIGH); // Clockwise
  } else {
    digitalWrite(STEPPER1_DIR_PIN, LOW); // Counter-clockwise
  }

  // Control speed with linear.x (absolute value)
  float speed = fabs(twist_msg->linear.x);
  if (speed > 0) {
    // Convert speed (rev/s) to delay in microseconds
    // 1 / (speed * STEPS_PER_REV) = seconds per step
    // * 1,000,000 = microseconds per step
    // / 2 because we have two delays per step (HIGH and LOW)
    int us_delay = (int)(1000000.0 / (speed * STEPS_PER_REV * 2.0));

    // Step the motor
    digitalWrite(STEPPER1_STEP_PIN, HIGH);
    delayMicroseconds(us_delay);
    digitalWrite(STEPPER1_STEP_PIN, LOW);
    delayMicroseconds(us_delay);
    
    // Update position
    float position_change = 1.0 / STEPS_PER_REV;
    if (twist_msg->angular.z < 0) {
        position_change *= -1;
    }
    motor_position += position_change;
  }
}
#endif

#ifdef ENABLE_STEPPER_1
void home_stepper_callback(const void *request, void *response) {
  motor_position = 0.0;
  // You can optionally add a response message
  // home_stepper_response.message.data = "Stepper position homed to 0";
}

void stepper_position_callback(const void *msgin) {
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
  // 0: home, 1: pos1, 2: pos2, 3: pos3
  int8_t command = msg->data;

  // Set direction
  if (command > motor_position) {
    digitalWrite(STEPPER1_DIR_PIN, HIGH); // Clockwise
  } else {
    digitalWrite(STEPPER1_DIR_PIN, LOW); // Counter-clockwise
  }

  // Calculate steps to move
  int steps_to_move = abs(command - motor_position) * STEPS_PER_REV;
  int us_delay = 1000; // Adjust for desired speed

  // Step the motor
  for (int i = 0; i < steps_to_move; i++) {
    digitalWrite(STEPPER1_STEP_PIN, HIGH);
    delayMicroseconds(us_delay);
    digitalWrite(STEPPER1_STEP_PIN, LOW);
    delayMicroseconds(us_delay);
  }

  // Update position
  motor_position = command;
}

#ifdef ENABLE_STEPPER_2
// Callback for Stepper 2: just sets the target speed and direction
void stepper2_speed_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float speed_rps = msg->data; // Speed in revolutions per second

  if (speed_rps == 0.0) {
    stepper2_is_running = false;
    digitalWrite(STEPPER2_STEP_PIN, LOW); // Ensure step pin is off when stopped
  } else {
    // Set direction based on the sign of the speed
    digitalWrite(STEPPER2_DIR_PIN, (speed_rps > 0) ? HIGH : LOW);
    
    // Calculate delay based on absolute speed
    // Total steps per second = abs(speed_rps) * STEPS_PER_REV
    // Time per step (in seconds) = 1 / total_steps_per_second
    // Delay between HIGH/LOW toggle (in microseconds) = (time_per_step / 2) * 1,000,000
    stepper2_step_delay = (long)(500000.0 / (fabs(speed_rps) * STEPS_PER_REV));
    
    stepper2_is_running = true;
  }
}
#endif

#ifdef ENABLE_HBRIDGE
void hbridge_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data) {
    digitalWrite(HBRIDGE_IN1, HIGH);
    digitalWrite(HBRIDGE_IN2, LOW);
  } else {
    digitalWrite(HBRIDGE_IN1, LOW);
    digitalWrite(HBRIDGE_IN2, LOW);
  }
}
#endif

#ifdef ENABLE_TOF_SENSORS
  void read_tof_sensor() {
  // blink_led(4, 250);
    // Update the sensors as fast as they're available
    if (tofLeft.isDataReady()) {
      left_distance = tofLeft.getDistance_mm();
    }

    if (tofRight.isDataReady()) {
      right_distance = tofRight.getDistance_mm();
    }

    if (tofFront.isDataReady()) {
      front_distance = tofFront.getDistance_mm();
    }

    if (tofBack.isDataReady()) {
      back_distance = tofBack.getDistance_mm();
    }

    // publish the TOF sensor data [ADD back_distance WHEN able to power all 4 sensors]
    tof_pub.publish(left_distance, right_distance, front_distance, back_distance);
  }
#endif // ENABLE_TOF_SENSORS

/**
 * This function is the main loop for the micro-ROS node. It manages the
 * connection and disconnection of the micro-ROS agent, actuator positions,
 * and sensor data collection.
 */
void loop() {

  // fail safe for agent disconnect
  if (millis() - last_received > 5000) {

#ifdef ENABLE_ACTUATORS
    // TODO: Add actuator stop code here
#endif // ENABLE_ACTUATORS

#ifdef ENABLE_HBRIDGE
  digitalWrite(HBRIDGE_IN3, LOW);
  digitalWrite(HBRIDGE_IN4, LOW);
#endif // ENABLE_HBRIDGE

#ifdef ENABLE_BT_DEBUG
    // BTSerial.println("[INFO] No command received in timeout, stopping actuators");
#endif // ENABLE_BT_DEBUG
  }

  // state machine to manage connecting and disconnecting the micro-ROS agent
  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    };
    break;

//loop that runs when microros agent is connected
  case AGENT_CONNECTED:
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
      state = AGENT_DISCONNECTED;
    }
    else {
      //////////////////////////////////////////////////////////
      // EXECUTES WHEN THE AGENT IS CONNECTED
      //////////////////////////////////////////////////////////

// #ifdef ENABLE_BATTERY
//       EXECUTE_EVERY_N_MS(BATTERY_MS, read_battery());
// #endif // ENABLE_BATTERY

#ifdef ENABLE_IR_SENSOR
      // Read the IR sensor data at the specified interval
      EXECUTE_EVERY_N_MS(IR_MS, read_ir_sensor());
#endif // ENABLE_IR_SENSOR

#ifdef ENABLE_STEPPER_1
    // Publish stepper position every 100 ms
    EXECUTE_EVERY_N_MS(100, {
        position_msg.data = motor_position;
        RCSOFTCHECK(rcl_publish(&stepper_publisher, &position_msg, NULL));
    });
#endif

#ifdef ENABLE_STEPPER_2
  // Non-blocking control for Stepper 2
  if (stepper2_is_running) {
    unsigned long current_micros = micros();
    if (current_micros - stepper2_last_step_time >= stepper2_step_delay) {
      stepper2_last_step_time = current_micros;
      digitalWrite(STEPPER2_STEP_PIN, !digitalRead(STEPPER2_STEP_PIN)); // Toggle step pin
    }
  }
#endif

#ifdef ENABLE_HBRIDGE
      // Continuously spin the second H-bridge motor slowly
      digitalWrite(HBRIDGE_IN3, HIGH);
      digitalWrite(HBRIDGE_IN4, LOW);
#endif // ENABLE_HBRIDGE


// #ifdef ENABLE_EGGDETECT
//     EXECUTE_EVERY_N_MS(100, {
//           // egg_msg.data = true;
//           RCSOFTCHECK(rcl_publish(&eggdetect_pub, &egg_msg, NULL));
//       });
// #endif //ENABLE_EGGDETECT

#ifdef ENABLE_TOF_SENSORS
      EXECUTE_EVERY_N_MS(TOF_MS, read_tof_sensor());
#endif // ENABLE_TOF_SENSORS


      // line to process ROS messages
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));


      //////////////////////////////////////////////////////////
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }
}

