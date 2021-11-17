/*
 * move_demo.cpp
 *
 *  Created on: Jan 22, 2021
 *      Author: hanbing
 */

#include "ros/ros.h"
#include "move_control/move_control.h"
#include "ca_msgs/robot_ca.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_demo");
	ros::NodeHandle n;
	hyy_move_control::MoveControl robot(n);
	std::vector<double> joint_target;
	//init robot
	if (!robot.init())
	{
		ROS_ERROR("robot init error!!!");
		return -1;
	}
	std::vector<double> joint_current;
    joint_current = robot.get_joint_current();
    ROS_INFO("Joint: %f %f %f %f %f %f %f",
		joint_current.at(0),
		joint_current.at(1),
		joint_current.at(2),
		joint_current.at(3),
		joint_current.at(4),
		joint_current.at(5),
		joint_current.at(6));

	joint_target.push_back(1.57);
	joint_target.push_back(0.5);
	joint_target.push_back(-0.02);
	joint_target.push_back(0.8);
	joint_target.push_back(0.02);
	joint_target.push_back(1.8);
	joint_target.push_back(1.57);
	robot.moveA(joint_target);

	return 0;
}