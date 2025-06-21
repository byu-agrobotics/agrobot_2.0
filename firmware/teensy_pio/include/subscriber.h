#ifndef SUBSCRIBER
#define SUBSCRIBER

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define NS_S 1000000000
#define NS_TO_S(ns) (ns / NS_S)
#define NS_REMAINDER(ns) (ns % NS_S)

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }

#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

/**
 * @author ChatGPT
 * @date June 2025
 *
 * Base class for micro-ROS subscribers.
 */
class Subscriber {

public:
  /**
   * Sets up the subscriber. Must be implemented by the derived class.
   *
   * @param node     the micro-ROS node
   * @param executor the executor used t*
