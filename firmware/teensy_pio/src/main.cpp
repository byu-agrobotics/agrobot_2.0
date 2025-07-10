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

// #include <frost_interfaces/msg/u_command.h>

#define ENABLE_ACTUATORS
#define ENABLE_TOF_SENSORS
#define ENABLE_LEDS
#define ENABLE_BATTERY
#define ENABLE_BT_DEBUG

// #define ENABLE_HBRIDGE
#define ENABLE_IR_SENSOR
// #define ENABLE_SERVOS
// #define ENABLE_LED_MATRIX

#define ENABLE_STEPPER_1
#define pinIRd 25 // IR sensor pin
// TOF enable pins
#define EN1       2                   // EN pin for left TMF8801
#define EN2       3                   // EN pin for right TMF8801
#define EN3       4                   // EN pin for front TMF8801
#define EN4       5                   // EN pin for back TMF8801
#define INT       -1                  // INT pin is floating, not used in this demo
// Stepper motor
#define STEPPER_STEP_PIN 18
#define STEPPER_DIR_PIN 19
#define STEPS_PER_REV 200
// L298N1 H-Bridge pins
#define HBRIDGE_IN1 22
#define HBRIDGE_IN2 23
#define HBRIDGE_IN3 24
#define HBRIDGE_IN4 25
#define HBRIDGE_ENA 16
#define HBRIDGE_ENB 17
// Servo pins
#define SERVO_1_PIN 11 // Pin for servo 1
#define SERVO_2_PIN 12 // Pin for servo 2
#define SERVO_3_PIN 14 // Pin for servo 3
// Big Servo
#define BIG_SERVO_PIN 29 // Pin for the big servo
// hardware pin values
#define BT_MC_RX 34
#define BT_MC_TX 35
// #define VOLT_PIN 18
// #define CURRENT_PIN 17
#define LED_PIN 13 // Built-in Teensy LED

// This is the corrected macro. The stray backslash on the empty line was removed.
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
#define CALLBACK_TOTAL 6
#define SYNC_TIMEOUT 1000




// sensor baud rates
#define BT_DEBUG_RATE 9600

// sensor update rates
#define BATTERY_MS 1000 // arbitrary
#define TOF_MS 5     // arbitrary
#define IR_MS 500    // Read the IR sensor every 500ms

// time of last received command (used as a fail safe)
unsigned long last_received = 0;

// TOF Calibration data
uint8_t caliDataBuf[14] = {0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor; // Added executor declaration

// Egg detection publisher
rcl_publisher_t egg_publisher;
std_msgs__msg__Bool egg_msg;

// Stepper motor ROS objects
#ifdef ENABLE_STEPPER_1
  rcl_subscription_t stepper_subscriber;
  rcl_publisher_t stepper_publisher;
  geometry_msgs__msg__Twist twist_msg;
  std_msgs__msg__Float32 position_msg;

  volatile float motor_position = 0.0; // In revolutions
#endif

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
      "egg_detected")); // This is the topic name
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

  // Add subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &stepper_subscriber, &twist_msg, &stepper_callback, ON_NEW_DATA));
#endif

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
  pinMode(pinIRd, INPUT); // Set the IR sensor pin as an input

#ifdef ENABLE_BT_DEBUG
  BTSerial.begin(BT_DEBUG_RATE);
#endif // ENABLE_BT_DEBUG

#ifdef ENABLE_BATTERY
  pinMode(CURRENT_PIN, INPUT);
  pinMode(VOLT_PIN, INPUT);

#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Battery Sensor enabled");
#endif // ENABLE_BT_DEBUG
#endif // ENABLE_BATTERY

#ifdef ENABLE_STEPPER_1
  // Configure stepper pins as outputs
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  #ifdef ENABLE_BT_DEBUG
    BTSerial.println("[INFO] Stepper 1 enabled");
  #endif
#endif

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
void read_battery() {
  blink_led(1, 30);
  // we did some testing to determine the below params, but
  // it's possible they are not completely accurate
  float voltage = (analogRead(VOLT_PIN) * 0.03437) + 0.68;
  float current = (analogRead(CURRENT_PIN) * 0.122) - 11.95;

  // publish the battery data
  // battery_pub.publish(voltage, current);
}

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

  // Control direction with angular.z
  if (twist_msg->angular.z > 0) {
    digitalWrite(STEPPER_DIR_PIN, HIGH); // Clockwise
  } else {
    digitalWrite(STEPPER_DIR_PIN, LOW); // Counter-clockwise
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
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(us_delay);
    digitalWrite(STEPPER_STEP_PIN, LOW);
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
