#ifndef KDL_BASE_H
#define KDL_BASE_H

#include <urdf/model.h>

#include <ros/node_handle.h>
#include <ros/ros.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <vector>

class KDL_BASE
{
public:
  KDL::Chain kdl_chain_;
  KDL_BASE() {}
  ~KDL_BASE() {}

  bool init(ros::NodeHandle &n);

public:
  ros::NodeHandle KDL_BASE_nh;
  KDL::JntArrayVel joint_msr_;

  struct limits_
  {
    KDL::JntArray min;
    KDL::JntArray max;
    KDL::JntArray center;
  } joint_limits_;

private:
  std::string robot_description, root_name, tip_name, name_space;
  std::string xml_string;
  urdf::Model model;
  KDL::Tree kdl_tree;

  urdf::LinkConstSharedPtr link_;
  urdf::JointConstSharedPtr joint_;
};
#endif
