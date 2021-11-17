//ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include "sensor_msgs/Range.h"

//C++ headers
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <cstring>

//custom headers: Exactly the Message Header
#include "four_wheels/DeviceState.h"

class device_driver
{
private:
    // ROS Parameters:
    ros::NodeHandle* node_handle_ptr;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher	device_state_pub;
    ros::Publisher	odom_pub;
    ros::Publisher  device_raw_pub;

    geometry_msgs::Twist cmdVel,targetCmdVel;
    nav_msgs::Odometry odom_msg;
    std::vector<ros::Publisher> sonar_pubs;

    //ROS Message:
    four_wheels::DeviceState deviceState;
    four_wheels::DeviceState last_deviceState;

    //Communication:
    int sock_cli;
    std::string sendBuffer;
    std::string recvBuffer;
    bool odom_received;


    //Parameter server
    std::string p_controller_ip_;
    int p_controller_port_;
    double p_wheel_ratio_;
    bool p_provide_odom_frame_;
    bool p_provide_odom_topic_;

    //Others:
    fd_set rfds;
    int retval,maxfd;
    struct timeval tv;
    bool shutdown_flag;
public:
    device_driver(int argc, char **argv);
    ~device_driver(){};

    bool decode();
    void readSocket();
    void onCmdVel(const geometry_msgs::Twist::ConstPtr& msgs);
    void pubControl();
    bool socketInit();
    void initMsg();

    void main();
};

