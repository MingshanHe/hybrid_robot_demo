#include "Modbus.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

class Gripper_node
{
private:
    int currpos;
    int g_state;
    float command;
    ros::NodeHandle nh_;

    ros::Subscriber gripper_sub;

    DH_Modbus_Gripper m_gripper;
public:
    Gripper_node(ros::NodeHandle& nh);
    ~Gripper_node();
    void gripper_cb(const std_msgs::Float64ConstPtr& msg);
    void main_();
};

Gripper_node::Gripper_node(ros::NodeHandle& nh):
m_gripper(1, "/dev/ttyUSB0", 115200)
{
    nh_ = nh;
    gripper_sub = nh_.subscribe<std_msgs::Float64>("/gripper/command",1,&Gripper_node::gripper_cb, this);

    DH_Modbus_Gripper m_gripper(1, "/dev/ttyUSB0", 115200);
    if(m_gripper.open()<0)
    {
        return ;
    }
    //initialize the gripper
    int initstate = 0;
    m_gripper.GetInitState(initstate);
    if(initstate != DH_Modbus_Gripper::S_INIT_FINISHED)
    {
        m_gripper.Initialization();
        std::cout<< " Send grip init " << std::endl;

        //wait for gripper initialization
        initstate = 0;
        std::cout<< " Send grip GetInitState " << std::endl;
        while(initstate != DH_Modbus_Gripper::S_INIT_FINISHED )
            m_gripper.GetInitState(initstate); 
        std::cout<< " Send grip GetInitState "<< initstate << std::endl;
    }

    currpos= 0;
    g_state = 0;
}

Gripper_node::~Gripper_node()
{
}

void Gripper_node::gripper_cb(const std_msgs::Float64ConstPtr& msg)
{
    if(m_gripper.open()<0)
    {
        return ;
    }
    //initialize the gripper
    int initstate = 0;
    m_gripper.GetInitState(initstate);
    if(initstate != DH_Modbus_Gripper::S_INIT_FINISHED)
    {
        m_gripper.Initialization();
        std::cout<< " Send grip init " << std::endl;

        //wait for gripper initialization
        initstate = 0;
        std::cout<< " Send grip GetInitState " << std::endl;
        while(initstate != DH_Modbus_Gripper::S_INIT_FINISHED )
            m_gripper.GetInitState(initstate); 
        std::cout<< " Send grip GetInitState "<< initstate << std::endl;
    }

    currpos= 0;
    g_state = 0;
    ROS_INFO("In SUB");
    command = msg->data;
    //set gripper target position 1000
    m_gripper.SetTargetPosition(command);
    //wait gripper arrived the target postion
    g_state = 0;
    while(g_state == DH_Modbus_Gripper::S_GRIP_MOVING)
        m_gripper.GetGripState(g_state);
    std::cout<< "1 current grip state " << g_state << std::endl;
}

void Gripper_node::main_()
{
    // ros::Rate loop_rate(10);
    while(ros::ok())
    {
        // loop_rate.sleep();
        ros::spinOnce();
    }
}
