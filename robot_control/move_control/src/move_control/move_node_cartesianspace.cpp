/*
 * move_demo.cpp
 *
 *  Created on: Jan 22, 2021
 *      Author: hanbing
 */

#include "ros/ros.h"
#include "move_control/move_control.h"

int main(int argc, char ** argv)
{
		ros::init(argc, argv, "move_demo");

        ros::NodeHandle nh;
        hyy_move_control::MoveControl robot1(nh);
        if (!robot1.init())
        {
            ROS_ERROR("robot init error!!!");
        }
        ROS_INFO("Get A Position Command.");

        std::vector<double> cartesian_current;
        cartesian_current = robot1.get_cartesian_current();

        ROS_INFO("Cartesian xyz: [ %f, %f, %f ]", cartesian_current.at(0), cartesian_current.at(1), cartesian_current.at(2));

        std::vector<double> cartesian_target;
        cartesian_target.push_back(0);
        cartesian_target.push_back(0.0);
        cartesian_target.push_back(-0.2);
        cartesian_target.push_back(0);
        cartesian_target.push_back(0);
        cartesian_target.push_back(0);
        std::vector<double> cartesian_joint;

        cartesian_joint = robot1.offs(cartesian_target, cartesian_current[0], cartesian_current[1], cartesian_current[2],
        cartesian_current[3], cartesian_current[4], cartesian_current[5]);
        ROS_INFO("%d", cartesian_joint.size());
        ROS_INFO("Cartesian xyz: [ %f, %f, %f ]", cartesian_joint.at(0), cartesian_joint.at(1), cartesian_joint.at(2));
        std::string vel = "v10";
        robot1.moveL(cartesian_joint, vel);
}
