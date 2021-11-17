//---------------------- C++ headers ----------------------
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>   
#include <sstream>
#include <termios.h> 

//---------------------- ROS headers ----------------------
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"


//---------------------- Other headers ----------------------

//---------------------- Joystick Button Marco ----------------------
#define MAX_VEL (0.7)
#define MAX_RAD (0.4) 

#define	L_x 	(-joy.axes[0])
#define	L_y 	(joy.axes[1])
#define L_T 	(joy.axes[2]<0)
#define R_x 	(-joy.axes[3])
#define R_y 	(joy.axes[4])
#define R_T 	(joy.axes[5]<0)
#define P_x	(joy.buttons[2])
#define P_y	(joy.buttons[3])

#define A   (joy.buttons[0])
#define B   (joy.buttons[1])
#define X	(joy.buttons[2])
#define Y	(joy.buttons[3])
#define L_B	(joy.buttons[4])
#define R_B	(joy.buttons[5])
#define BACK	(joy.buttons[6])
#define START	(joy.buttons[7])
#define HOME	(joy.buttons[8])
#define	L_down	(joy.buttons[9])
#define R_down	(joy.buttons[10])
#define P_l	(joy.axes[6]>0.5)
#define P_r	(joy.axes[6]<-0.5)
#define P_u	(joy.axes[7]>0.5)
#define P_d	(joy.axes[7]<-0.5)

#define	last_L_x 	(-last_joy.axes[0])
#define	last_L_y 	(last_joy.axes[1])
#define last_L_T 	(last_joy.axes[2]<0)
#define last_R_x 	(-last_joy.axes[3])
#define last_R_y 	(last_joy.axes[4])
#define last_R_T 	(last_joy.axes[5]<0)
#define last_P_x	(last_joy.buttons[2])
#define last_P_y	(last_joy.buttons[3])

#define last_A   	(last_joy.buttons[0])
#define last_B   	(last_joy.buttons[1])
#define last_X		(last_joy.buttons[2])
#define last_Y		(last_joy.buttons[3])
#define last_L_B	(last_joy.buttons[4])
#define last_R_B	(last_joy.buttons[5])
#define last_BACK	(last_joy.buttons[6])
#define last_START	(last_joy.buttons[7])
#define last_HOME	(last_joy.buttons[8])
#define	last_L_down	(last_joy.buttons[9])
#define last_R_down	(last_joy.buttons[10])
#define last_P_l	(last_joy.axes[6]>0.5)
#define last_P_r	(last_joy.axes[6]<-0.5)
#define last_P_u	(last_joy.axes[7]>0.5)
#define last_P_d	(last_joy.axes[7]<-0.5)

#define vel_Acc	(0.1)
#define rad_Acc	(0.3)
#define Zero_point (0.2)

//---------------------- ROS Publishers ----------------------
ros::Publisher joy_cmd_vel_pub;	//robot vel and rad publisher
ros::Publisher joy_event_pub;	//system message, currently used for switching from auto mode to joystick mode

double vel=0,rad=0,vy=0;		//linear and angular speed (really sent)
double target_vel,target_rad;	//target linear and angular speed (used for smooth acceleration)
double target_vy;


std_msgs::String sys_msgs;			//global system message
sensor_msgs::Joy joy;				//joystick reader message
sensor_msgs::Joy last_joy;

//parameters server
double p_max_robot_vel=1.0;
double p_max_robot_rad=0.3;

bool is_first_joy_msg = true;

int joy_watch_dog = 0;
//================= ROS Callback =================
void	onJoy(const sensor_msgs::Joy::ConstPtr &msgs)
{
	if (msgs->axes.size()!=8 || msgs->buttons.size()!=11) return;
	if (joy_watch_dog>=20)
	{
		is_first_joy_msg = true;
		ROS_INFO("Joystick Connected!");
	}
	joy_watch_dog=0;
	joy = *msgs;
}
void pubSys(const char msg[])
{
	char buf[20];
	if (R_T)//组合触发
	{
		sprintf(buf,"%s*",msg);
		sys_msgs.data.assign(buf);
	}
	else
	if (L_T)//组合触发2
	{
		sprintf(buf,"%s#",msg);
		sys_msgs.data.assign(buf);
	}
	else
		sys_msgs.data.assign(msg);
	joy_event_pub.publish(sys_msgs);
}
void loop()
{
	if (is_first_joy_msg)
	{
		last_joy.axes.assign(joy.axes.begin(),joy.axes.end());
		last_joy.buttons.assign(joy.buttons.begin(),joy.buttons.end());
		is_first_joy_msg = false;
	}
	
	//按键单次触发
	if (A && !last_A) pubSys("A");
	if (!A && last_A) pubSys("~A");
	if (B && !last_B) pubSys("B");
	if (!B && last_B) pubSys("~B");
	if (X && !last_X) pubSys("X");
	if (!X && last_X) pubSys("~X");
	if (Y && !last_Y) pubSys("Y");
	if (!Y && last_Y) pubSys("~Y");
	if (START && !last_START) pubSys("START");
	if (!START && last_START) pubSys("~START");
	if (HOME && !last_HOME) pubSys("HOME");	
	if (!HOME && last_HOME) pubSys("~HOME");	
	if (BACK && !last_BACK) pubSys("BACK");
	if (!BACK && last_BACK) pubSys("~BACK");
	if (L_B && !last_L_B) pubSys("L_B");
	if (!L_B && last_L_B) pubSys("~L_B");
	if (R_B && !last_R_B) pubSys("R_B");
	if (!R_B && last_R_B) pubSys("~R_B");
	if (L_down && !last_L_down) pubSys("L_down");
	if (!L_down && last_L_down) pubSys("~L_down");
	if (R_down && !last_R_down) pubSys("R_down");
	if (!R_down && last_R_down) pubSys("~R_down");
	if (P_l && !last_P_l) pubSys("P_l");
	if (!P_l && last_P_l) pubSys("~P_l");
	if (P_r && !last_P_r) pubSys("P_r");
	if (!P_r && last_P_r) pubSys("~P_r");
	if (P_u && !last_P_u) pubSys("P_u");
	if (!P_u && last_P_u) pubSys("~P_u");
	if (P_d && !last_P_d) pubSys("P_d");
	if (!P_d && last_P_d) pubSys("~P_d");

	//速度角速度
	target_rad = -R_x;
	target_vel = L_y;
	target_vy = -L_x;
	
	if (fabs(target_vel)<0.1) target_vel = 0;
	if (fabs(target_rad)<0.1) target_rad = 0;
	if (fabs(target_vy)<0.1) target_vy = 0;
	
	
	vel = target_vel;
	rad = target_rad;
	vy = target_vy;

	geometry_msgs::Twist msgs;
	msgs.linear.x  = vel*p_max_robot_vel;
	msgs.linear.y  = vy*p_max_robot_vel;
	msgs.angular.z = rad*p_max_robot_rad;
	
	static int zero_speed_cnt = 10;
	if (fabs(msgs.linear.x)<=0.01 && fabs(msgs.linear.y)<=0.01 && fabs(msgs.angular.z)<=0.01)
	{
	        msgs.linear.x = 0;
	        msgs.linear.y = 0;
	        msgs.angular.z = 0;
	        if (zero_speed_cnt==2)
	                ROS_INFO_THROTTLE(5,"zero joy speed");
	        if (zero_speed_cnt>0)
	                zero_speed_cnt--;
	}
	else
	{
	        ROS_INFO("%0.4f %0.4f %0.4f ",msgs.linear.x,msgs.linear.y, msgs.angular.z);
	        zero_speed_cnt = 10;
	}
	        
	if (zero_speed_cnt>0)
	        joy_cmd_vel_pub.publish(msgs);
	
	//record next
	last_joy.axes.assign(joy.axes.begin(),joy.axes.end());
	last_joy.buttons.assign(joy.buttons.begin(),joy.buttons.end());
}
void clearJoy()
{
	std::vector<int>::iterator button_iter;
	std::vector<float>::iterator axes_iter;
	for (button_iter = joy.buttons.begin();button_iter != joy.buttons.end();button_iter++)
	{
		*button_iter = 0;
	}
	for (axes_iter = joy.axes.begin();axes_iter != joy.axes.end();axes_iter++)
	{
		*axes_iter = 0;
	}
}
int     main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_control");
	ros::NodeHandle n;
	
	joy_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	joy_event_pub = n.advertise<std_msgs::String>("/joy/event",10);
	
	ros::Subscriber joystick_sub = n.subscribe<sensor_msgs::Joy>("/joy",3,onJoy);
	
	for (int i =0;i<8;i++) joy.axes.push_back(0);
	for (int i =0;i<15;i++)joy.buttons.push_back(0);
	
	ros::Rate rate(20);	
	while (ros::ok())
	{
		ros::spinOnce();
		joy_watch_dog++;
		if (joy_watch_dog==10)
		{
			clearJoy();
			ROS_ERROR("Joystick disconnected!");
		}
		loop();
		rate.sleep();
	}

	return 0;
}
