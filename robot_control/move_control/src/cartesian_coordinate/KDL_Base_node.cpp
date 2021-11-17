#include "cartesian_coordinate/KDL_Base_node.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kdl_base_node");
    ros::NodeHandle nh;
    KDL_Base_node kdl_base_node (argc, argv, nh);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}
