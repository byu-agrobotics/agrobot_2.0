#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Select and start a task

# TODO: Add a menu to select which task to launch
ros2 action send_goal /exec_collect_fsm agrobot_interfaces/action/GenericTask "{}" --feedback