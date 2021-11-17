#include "gripper/Gripper_node.h"
#include "stdio.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Gripper_node");
    ros::NodeHandle nh;

    Gripper_node gripper_node(nh);
    gripper_node.main_();

	return 0;
}
