#ifndef KDL_BASE_NODE_H
#define KDL_BASE_NODE_H
#include "ros/ros.h"

#include "KDL_Base.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <cartesian_state_msgs/Joint.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "kdl_conversions/kdl_msg.h"
#include <realtime_tools/realtime_publisher.h>

#include "hyy_message/robot_move.h"
#include "hyy_message/robot_io.h"
#include "hyy_message/robot_move_data.h"
#include "move_control/move_control.h"
#include "ca_msgs/robot_ca.h"

class KDL_Base_node
{
protected:
    KDL_BASE kdl_base;

    boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos> ik_pos_solver_;
    boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
    boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;

    ros::Subscriber command_cart_vel_sub;
    ros::Subscriber command_cart_pos_sub;
    ros::Subscriber joint_states_sub;
    ros::Subscriber gripper_set;

    ros::Publisher  ee_state_publisher;
    ros::Publisher  joint_cmd_publisher;

    KDL::Frame x_des_;
    KDL::Twist x_dt_des_;         // Desired end-effector velocity
    KDL::JntArray q_dt_cmd_;      // Desired joint velocity
    KDL::JntArray q_cmd_;         // Desired joint position
    KDL::FrameVel x_dot_;
    KDL::Frame x_;
    cartesian_state_msgs::PoseTwist msg_state_;
    bool cartesian_velocity_control;

private:
    cartesian_state_msgs::PoseTwist cartesian_msg;
    cartesian_state_msgs::Joint joint_cmd_msg;
    sensor_msgs::JointState joint_msg;
public:
    ros::NodeHandle nh_;
    KDL::JntArray q_;
public:
    KDL_Base_node(int argc, char **argv, ros::NodeHandle& nh);
    ~KDL_Base_node(){};
    void command_cart_vel(const geometry_msgs::TwistConstPtr &msg);
    void command_cart_pos(const geometry_msgs::PoseConstPtr &msg);
    void joint_states_cb(const sensor_msgs::JointStateConstPtr &msg);

};

KDL_Base_node::KDL_Base_node(int argc, char **argv, ros::NodeHandle& nh)
{
    nh_ = nh;
    kdl_base.init(nh_);
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(kdl_base.kdl_chain_));
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_base.kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_base.kdl_chain_));

    command_cart_vel_sub = nh_.subscribe("/xmate/cartesian_velocity_controller/command_cart_vel", 5, &KDL_Base_node::command_cart_vel, this);
    command_cart_pos_sub = nh_.subscribe("/xmate/cartesian_velocity_controller/command_cart_pos", 5, &KDL_Base_node::command_cart_pos, this);
    joint_states_sub = nh_.subscribe("/joint_states", 1, &KDL_Base_node::joint_states_cb, this);

    ee_state_publisher = nh_.advertise<cartesian_state_msgs::PoseTwist>("/xmate/cartesian_velocity_controller/ee_state", 1);
    joint_cmd_publisher = nh_.advertise<cartesian_state_msgs::Joint>("/xmate/cartesian_velocity_controller/command", 1);
    // gripper_set = nh_.subscribe("/ur5/joint_states",5,&KDL_Base_node::gripper, this);
    // Variable init
    kdl_base.joint_msr_.q.resize(7);
    q_dt_cmd_.resize(7);
    q_cmd_.resize(7);
    x_dt_des_ = KDL::Twist::Zero();
    x_.p.Zero();
    x_.M.Identity();
    x_dot_.p.Zero();
    x_dot_.M.Identity();
    ROS_INFO("Finished Cartesian Velocity Controller Base init");
}

void KDL_Base_node::command_cart_vel( const geometry_msgs::TwistConstPtr &msg )
{
    x_dt_des_.vel(0) = msg->linear.x;
    x_dt_des_.vel(1) = msg->linear.y;
    x_dt_des_.vel(2) = msg->linear.z;
    x_dt_des_.rot(0) = msg->angular.x;
    x_dt_des_.rot(1) = msg->angular.y;
    x_dt_des_.rot(2) = msg->angular.z;

    fk_vel_solver_->JntToCart(kdl_base.joint_msr_, x_dot_);
    fk_pos_solver_->JntToCart(kdl_base.joint_msr_.q, x_);

    // try to publish
    tf::poseKDLToMsg(x_, cartesian_msg.pose);
    tf::twistKDLToMsg(x_dot_.GetTwist(), cartesian_msg.twist);
    ee_state_publisher.publish(cartesian_msg);
    ROS_INFO("In vel cb %f", joint_msg.position[1]);  
    for (unsigned int i = 0; i < 7; i++)
    {
        kdl_base.joint_msr_.q(i) = joint_msg.position.at(i);
    }
    float dt = 0.95;
    ik_vel_solver_->CartToJnt(kdl_base.joint_msr_.q, x_dt_des_, q_dt_cmd_);
    joint_cmd_msg.Joint1 = joint_msg.position[0]+q_dt_cmd_(0)*dt;
    joint_cmd_msg.Joint2 = joint_msg.position[1]+q_dt_cmd_(1)*dt;
    joint_cmd_msg.Joint3 = joint_msg.position[2]+q_dt_cmd_(2)*dt;
    joint_cmd_msg.Joint4 = joint_msg.position[3]+q_dt_cmd_(3)*dt;
    joint_cmd_msg.Joint5 = joint_msg.position[4]+q_dt_cmd_(4)*dt;
    joint_cmd_msg.Joint6 = joint_msg.position[5]+q_dt_cmd_(5)*dt;
    joint_cmd_msg.Joint7 = joint_msg.position[6]+q_dt_cmd_(6)*dt;

    joint_cmd_publisher.publish(joint_cmd_msg);

}

void KDL_Base_node::command_cart_pos( const geometry_msgs::PoseConstPtr &msg )
{
ROS_INFO("test");
    fk_vel_solver_->JntToCart(kdl_base.joint_msr_, x_dot_);
    fk_pos_solver_->JntToCart(kdl_base.joint_msr_.q, x_);
  ROS_INFO("test");
    // try to publish
    tf::poseKDLToMsg(x_, cartesian_msg.pose);
    ROS_INFO("test");
    tf::twistKDLToMsg(x_dot_.GetTwist(), cartesian_msg.twist);
    ee_state_publisher.publish(cartesian_msg);
    ROS_INFO("In vel cb %f", joint_msg.position[1]);  
    for (unsigned int i = 0; i < 7; i++)
    {
        kdl_base.joint_msr_.q(i) = joint_msg.position.at(i);
    }
    ROS_INFO("test");
    x_des_.p(0) = 0;
    x_des_.p(1) = 0;
    x_des_.p(2) = 0;
    x_des_.M = KDL::Rotation(1,0,0,0,1,0,0,0,1);
    ik_pos_solver_->CartToJnt(kdl_base.joint_msr_.q, x_des_, q_cmd_);
    ROS_INFO("q: %f %f %f %f %f %f %f", q_cmd_(0), q_cmd_(1), q_cmd_(2), q_cmd_(3), q_cmd_(4), q_cmd_(5), q_cmd_(6));
    //q_ = q_cmd_;
}

void KDL_Base_node::joint_states_cb(const sensor_msgs::JointStateConstPtr &msg)
{
    joint_msg.position = msg->position;
}
#endif
