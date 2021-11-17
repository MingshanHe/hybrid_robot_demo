#include "cartesian_coordinate/KDL_Base.h"

bool KDL_BASE::init(ros::NodeHandle &n)
{
    KDL_BASE_nh = n;

    // get URDF and name of root and tip from the parameter server
    name_space = KDL_BASE_nh.getNamespace();
    if (!ros::param::search(name_space,"robot_description", robot_description))
    {
        ROS_ERROR_STREAM("KDL_BASE: No robot description (URDF) found on parameter server (" << n.getNamespace() <<"/robot_description)");
        return false;
    }
    root_name = "base_link";
    tip_name = "link7";
    /*
    if (!KDL_BASE_nh.getParam("/base_link", root_name))
    {
        ROS_ERROR_STREAM("KDL_BASE: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
        return false;
    }
    if (!KDL_BASE_nh.getParam("/link7", tip_name))
    {
        ROS_ERROR_STREAM("KDL_BASE: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
        return false;
    }
    */
    // Construct an URDF model from the xml string
    if (KDL_BASE_nh.hasParam(robot_description))
        KDL_BASE_nh.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
        KDL_BASE_nh.shutdown();
        return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s", robot_description.c_str());
        KDL_BASE_nh.shutdown();
        return false;
    }

    // Get urdf model out of robot_description
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        n.shutdown();
        return false;
    }
    ROS_INFO("Successfully parsed urdf file");

    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        n.shutdown();
        return false;
    }

    // Populate the KDL chain
    if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
        ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }

    // KDL chain information
    ROS_INFO("tip_name:  %s",tip_name.c_str());
    ROS_INFO("root_name: %s",root_name.c_str());
    ROS_INFO("Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_INFO("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
    for(std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++){
        ROS_INFO_STREAM("segment("<<i<<"): " << kdl_chain_.getSegment(i).getName());
    }

    // Parsing joint limits from urdf model along kdl chain
    // Param Initialize
    link_ = model.getLink(tip_name);
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
    int index;

    for (std::size_t i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
    {
        joint_ = model.getJoint(link_->parent_joint->name);
        ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());
        index = kdl_chain_.getNrOfJoints() - i - 1;

        if(joint_->limits){
        joint_limits_.min(index) = joint_->limits->lower;
        joint_limits_.max(index) = joint_->limits->upper;
        joint_limits_.center(index) = (joint_limits_.min(index) +
                                        joint_limits_.max(index))/2;
        }else{
        joint_limits_.min(index) = 0;
        joint_limits_.max(index) = 0;
        joint_limits_.center(index) = 0;
        ROS_INFO("joint_->limits is NULL %s",joint_->name.c_str());
        }

        link_ = model.getLink(link_->getParent()->name);
    }

    ROS_INFO_STREAM("kdl_chain.getNrOfJoints: " << kdl_chain_.getNrOfJoints());

    ROS_INFO("Finished Kinematic Base init");

    return true;
}
