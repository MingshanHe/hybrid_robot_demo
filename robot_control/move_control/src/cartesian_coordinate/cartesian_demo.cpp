/*
 * move_demo.cpp
 *
 *  Created on: Jan 22, 2021
 *      Author: hanbing
 */

#include "ros/ros.h"
#include "move_control/move_control.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"


class cartesian
{
public:
cartesian(ros::NodeHandle n){
flag = false;
     joint_cmd = n.subscribe("/command_cart_pos", 1, &cartesian::joint_cmd_cb, this);
     robot_flag_cmd = n.subscribe("/robot_flag", 1, &cartesian::robot_flag_cb, this);
     gripper_cmd = n.advertise<std_msgs::Float64>("/gripper/command", 1);
    };
~cartesian(){};
void joint_cmd_cb(const geometry_msgs::TwistConstPtr &msg);
void robot_flag_cb(const std_msgs::Bool msg);

private:
ros::NodeHandle nh;
ros::Subscriber joint_cmd;
ros::Subscriber robot_flag_cmd;
ros::Publisher gripper_cmd;
bool flag;

};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "cartesian_demo");
    ros::NodeHandle n;
    cartesian cartesian_demo(n);



    


	while(ros::ok())
	{
                ros::spinOnce();
	}

	return 0;
}
void cartesian::robot_flag_cb(const std_msgs::Bool msg)
{
  if(msg.data)
  {
      flag = true;
  }
}
void cartesian::joint_cmd_cb(const geometry_msgs::TwistConstPtr &msg)
{
    if(flag)
    {
        float x, y, z;
        x = msg->linear.x;
        y = msg->linear.y;
        z = msg->linear.z;
        ROS_INFO("xyz: %f %f %f",x, y, z);
        ros::NodeHandle nh;
        hyy_move_control::MoveControl robot1(nh);
        if (!robot1.init())
        {
            ROS_ERROR("robot init error!!!");
        }
        ROS_INFO("Get A Position Command.");

        std::vector<double> cartesian_current;
        std::vector<double> joint_current;
        cartesian_current = robot1.get_cartesian_current();
        joint_current = robot1.get_joint_current();
        ROS_INFO("Cartesian xyz: [ %f, %f, %f ]", cartesian_current.at(0), cartesian_current.at(1), cartesian_current.at(2));
        ROS_INFO("Joint: %f %f %f %f %f %f %f", joint_current.at(0), joint_current.at(1), joint_current.at(2), joint_current.at(3), joint_current.at(4), joint_current.at(5), joint_current.at(6));
        std::vector<double> cartesian_target;
        cartesian_target.push_back(x);
        cartesian_target.push_back(-y);
        cartesian_target.push_back(-z);
        cartesian_target.push_back(0);
        cartesian_target.push_back(0);
        cartesian_target.push_back(0);
        std::vector<double> cartesian_joint;

        cartesian_joint = robot1.offs(cartesian_target, cartesian_current[0], cartesian_current[1], cartesian_current[2],
        cartesian_current[3], cartesian_current[4], cartesian_current[5]);
        ROS_INFO("%d", cartesian_joint.size());
        ROS_INFO("Cartesian xyz: [ %f, %f, %f ]", cartesian_joint.at(0), cartesian_joint.at(1), cartesian_joint.at(2));
        std::string vel = "v50";
        robot1.moveL(cartesian_joint, vel);
        flag = false;
        std_msgs::Float64 gripper_msg;
        gripper_msg.data = 700;
        gripper_cmd.publish(gripper_msg);
        ros::Duration(1).sleep();
        std::vector<double> joint_target;
        joint_target.push_back(1.57);
	joint_target.push_back(0.5);
	joint_target.push_back(-0.02);
	joint_target.push_back(0.8);
	joint_target.push_back(0.02);
	joint_target.push_back(1.8);
	joint_target.push_back(1.57);
	robot1.moveA(joint_target);
        
    }
}
