#!/bin/bash
gnome-terminal -x bash -c " sh ./robot/robot_contact.sh; exec bash;"

gnome-terminal -x bash -c " sh ./robot/robot_device.sh; exec bash;"

gnome-terminal -x bash -c " sh ./robot/robot_ros.sh; exec bash;"

gnome-terminal -x bash -c " sh ./robot/robot_start_controller.sh; exec bash;"