#!/bin/bash
gnome-terminal -- bash  -c "roslaunch four_wheels device.launch"

gnome-terminal -- bash  -c "roslaunch four_wheels scan_mode.launch"

gnome-terminal -- bash  -c "rosrun four_wheels keyboard_teleop"