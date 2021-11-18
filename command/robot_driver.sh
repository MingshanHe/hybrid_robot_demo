#!/bin/bash
gnome-terminal -- bash -c "  ~/hybrid_ws/src/command/robot/robot_contact.expect; exec bash;"

gnome-terminal -- bash -c " ~/hybrid_ws/src/command/robot/robot_device.expect; exec bash;"

gnome-terminal -- bash -c " ~/hybrid_ws/src/command/robot/robot_ros.expect; exec bash;"

gnome-terminal -- bash -c " ~/hybrid_ws/src/command/robot/robot_start_controller.expect; exec bash;"
