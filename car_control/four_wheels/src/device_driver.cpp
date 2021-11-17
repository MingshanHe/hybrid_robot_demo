#include "device_driver.h"

device_driver::device_driver(int argc, char **argv)
{
    //param init:
    odom_received = false;

    //ROS init:
    ros::init(argc, argv, "device_driver");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	node_handle_ptr = new ros::NodeHandle();
    //ROS parameter server
	private_nh.param("ip",p_controller_ip_,std::string("192.168.188.5"));
	private_nh.param("port",p_controller_port_,2048);
	private_nh.param("wheel_ratio",p_wheel_ratio_,10000.0);
	private_nh.param("provide_odom_frame",p_provide_odom_frame_,true);
	private_nh.param("provide_odom_topic",p_provide_odom_topic_,true);
	//subscribers
	cmd_vel_sub = n.subscribe("cmd_vel", 2, &device_driver::onCmdVel, this);
    //publishers
	device_state_pub = n.advertise<four_wheels::DeviceState>("device_state",10,true);
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	device_raw_pub = n.advertise<std_msgs::String>("device_raw",2); //底层传过来的原始数据

    initMsg();

	ROS_INFO("Connecting TCP-STM32");
	socketInit();
	ROS_INFO("Connected TCP-STM32");
}


bool device_driver::decode()
{
	if (recvBuffer.find('~',0)==std::string::npos) return false;
	std_msgs::String raw_str;
	raw_str.data = recvBuffer;
	device_raw_pub.publish(raw_str);

	while (recvBuffer.find('~',0)!=std::string::npos)
	{
		char command[100][100];
		int  cmd_index=0,tmp_index=0;
		for (int i =0;i<recvBuffer.length();i++)
		{
			if (recvBuffer[i] == '~')
			{
				recvBuffer = recvBuffer.substr(i+1);
				break;
			}
			else
			if (recvBuffer[i] == ',')
			{
				command[cmd_index++][tmp_index]=0;
				tmp_index = 0;
			}
			else
			if (recvBuffer[i] == '|')//process command
			{
				//ROS_INFO("processing request %s",command[0]);
				bool illegal = false;
				command[cmd_index][tmp_index] = 0;
				if (strcmp(command[0],"CONTROLMODE")==0)
				{
					if (strcmp(command[1],"MANUAL")==0)
					{
					}
					else
					if (strcmp(command[1],"AUTO")==0)
					{
						char str[50];
						sprintf(str,"SET_CONTROLMODE,MANUAL|");
						sendBuffer.append(str);
					}
					else
					illegal = true;
				}
				else
				if (strcmp(command[0],"BUTTON")==0)
				{
					if (strcmp(command[1],"EMERSTOPF")==0)
					{
						if (strcmp(command[2],"T")==0)
						{
							deviceState.button_emergency = true;
						}
						if (strcmp(command[2],"F")==0)
						{
							deviceState.button_emergency = false;
						}
					}
					else
					if (strcmp(command[1],"POWER")==0)
					{
						if (strcmp(command[2],"T")==0)
						{
							deviceState.button_power = true;
						}
						if (strcmp(command[2],"F")==0)
						{
							deviceState.button_power = false;
						}
					}
				}
				else
				if (strcmp(command[0],"CHARGE")==0)
				{
					if (strcmp(command[1],"CONTACK")==0)
					{
						if (strcmp(command[2],"T")==0)
						{
							deviceState.sen_charger = true;
						}
						else
						{
							deviceState.sen_charger = false;
						}
					}
					else
					illegal = true;
				}
				else
				if (strcmp(command[0],"STATE")==0)
				{
					deviceState.state.assign(command[1]);
					if(strcmp(command[1],"PAUSE")==0)
					{
						sendBuffer.append(std::string("SET_STATE,NORMAL|"));//急停后切换到正常状态
						std_msgs::String msg;
					}
				}
				else
				if (strcmp(command[0],"BATT")==0)
				{
					if (strcmp(command[1],"N")==0)
					{
						deviceState.batt_n = std::atoi(command[2]);
						if (deviceState.batt_n>100) deviceState.batt_n/=10;
					}
					else
					if (strcmp(command[1],"C")==0)
					{
						deviceState.batt_c = std::atoi(command[2]);
					}
					else
					if (strcmp(command[1],"V")==0)
					{
						deviceState.batt_v = std::atoi(command[2]);

					}
				}
				else if(strcmp(command[0],"SONAR")==0)
				{
					ros::Time time_now = ros::Time::now();
					int sonar_size = std::atoi(command[1]);
					for(int i = 0;i<sonar_size;i++)
					{
						char sonar_frame_name[30];
						sprintf(sonar_frame_name,"sonar_%d",i);
						sensor_msgs::Range sonar_msgs;
						sonar_msgs.header.frame_id.assign(sonar_frame_name);
						sonar_msgs.header.stamp = time_now;
						sonar_msgs.range = std::atoi(command[i+2])/1000.0;
						sonar_msgs.min_range = 0.01;
						sonar_msgs.max_range = 1.5;
						sonar_msgs.radiation_type = 0;
						if (sonar_pubs.size()<=i){
							sonar_pubs.push_back(node_handle_ptr->advertise<sensor_msgs::Range>(sonar_frame_name,5,false));
						}
						sonar_pubs[i].publish(sonar_msgs);
					}
				}
				else if (strcmp(command[0],"ODOM2D")==0 || strcmp(command[0],"LOCATION")==0)
				{
					odom_msg.pose.pose.position.x = std::atoi(command[1])/10000.0;
					odom_msg.pose.pose.position.y = std::atoi(command[2])/10000.0;
					double theta = std::atoi(command[3])/10000.0;
					odom_msg.pose.pose.orientation.w = cos(theta/2);
					odom_msg.pose.pose.orientation.z = sin(theta/2);
					//if (odom_msg.pose.pose.position.x!=0 || odom_msg.pose.pose.position.y!=0 || theta!=0)
					odom_received = true;
				}
				else
				if (strcmp(command[0],"FAULT_CODE")==0)
				{
					deviceState.fault_code.assign(command[1]);
				}
				else
				if (strcmp(command[0],"FORCEDSHUTDOWN")==0)  //已准备强制关机
				{
				}
				else
				{
					illegal = true;
				}
				if (illegal)
				{
					//ROS_ERROR("[cmdvel2stm32] Illegal Command %s,%s,%s from STM32!",command[0],command[1],command[2]);
				}
				cmd_index = 0;
				tmp_index = 0;
			}
			else
			command[cmd_index][tmp_index++] = recvBuffer[i];
		}
	}
return true;
}

void device_driver::readSocket()
{
	const int LEN = 1000;
	unsigned char str[LEN+1];
	int num;
	int read_len=0;

	FD_ZERO(&rfds);
	FD_SET(sock_cli,&rfds);
	maxfd = 0;
	if (maxfd< sock_cli)
		maxfd =  sock_cli;
	static int cnt = 0;
	retval = select(maxfd+1,&rfds,NULL,NULL,&tv);
	if (retval ==-1)
	{
		ROS_ERROR("Socket select error!\n");
		exit(0);
	}
	else
	if (retval ==0)
	{
		cnt = cnt+1;
		if (cnt ==200)
		{
			ROS_ERROR("Socket no receive\n");
		}
		if (cnt==400)
		{
			ROS_ERROR("Closing stm32\n");
			shutdown_flag = true;
		}
	}
	else
	{
		cnt = 0;
		if (FD_ISSET(sock_cli,&rfds))
		{
			memset(str,0,sizeof(str));
			int len = recv(sock_cli,str,LEN,0);
			str[len] = '\0';

			if (len <=0)
			{
				ROS_ERROR("Socket connection lost!\n");
				close(sock_cli);
				exit(0);
			}
			recvBuffer.append(reinterpret_cast<char*>(str));
		}
	}
}
//ROS callback
void device_driver::onCmdVel(const geometry_msgs::Twist::ConstPtr& msgs)
{
	targetCmdVel=*msgs;
}

void device_driver::pubControl()
{
    char str[200];   
    sprintf(str,"SET_VA,%d,%d,%d|",int(cmdVel.linear.x*10000),int(cmdVel.linear.y*10000),int(cmdVel.angular.z*10000));
    sendBuffer.append(std::string(str));
}

bool device_driver::socketInit()
{
    tv.tv_sec = 0;
	tv.tv_usec = 500000;
	sock_cli	= socket(AF_INET,SOCK_STREAM,0);
	struct sockaddr_in servaddr;
	memset(&servaddr,0,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(p_controller_port_);
	servaddr.sin_addr.s_addr = inet_addr(p_controller_ip_.c_str());

	if (connect(sock_cli,(struct sockaddr*)&servaddr,sizeof(servaddr))<0)
	{
		ROS_ERROR("Socket connection error");
		exit(0);
	}
	return true;
}

void device_driver::initMsg()
{
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_link";
	const double cov[36] = {
					0.5,0,0,0,0,0,
					0,0.5,0,0,0,0,
					0,0,0.5,0,0,0,
					0,0,0,0.5,0,0,
					0,0,0,0,0.5,0,
					0,0,0,0,0,0.5,
					};
	for (int i=0;i<36;i++)
	{
		odom_msg.pose.covariance[i]=cov[i];
		odom_msg.twist.covariance[i]=cov[i];
	}

}

void device_driver::main()
{
    ros::Rate read_rate(100);
	ros::Rate loop_rate(20);
	ros::Rate try_connection_rate(0.5);
    ros::Time last_time = ros::Time::now();
    while (ros::ok() && !shutdown_flag)
    {
        ros::spinOnce();
	    //发送一次速度
        cmdVel.linear.x = (cmdVel.linear.x+targetCmdVel.linear.x)/2;
        cmdVel.linear.y = (cmdVel.linear.y+targetCmdVel.linear.y)/2;
        cmdVel.angular.z = (cmdVel.angular.z+targetCmdVel.angular.z)/2;
        pubControl();
		sendBuffer.append("~");
		char buf[1000];
		strcpy(buf, sendBuffer.c_str());
		if (send(sock_cli,buf,sendBuffer.length(),0)<0)
            ROS_ERROR("Socket send fail!\n");
		else
		{
			ROS_INFO("Socket sent data:%s",sendBuffer.c_str());
        }
        sendBuffer.clear();
		last_time = ros::Time::now();
		do
        {
            readSocket();
            read_rate.sleep();
		}
		while(!decode() && !shutdown_flag && ros::ok());

		//odometry
		geometry_msgs::TransformStamped transformStamped;
		static tf::TransformBroadcaster broadcaster;
		if (odom_received)
		{
			if (p_provide_odom_topic_)
			{
				odom_msg.header.stamp = ros::Time::now();
				odom_msg.header.seq++;
				odom_pub.publish(odom_msg);
			}
            if (p_provide_odom_frame_)
			{
				transformStamped.header.stamp = ros::Time::now();
				transformStamped.header.frame_id = "odom";
				transformStamped.child_frame_id = "base_link";
				transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
				transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
				transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;
				transformStamped.transform.rotation.x = odom_msg.pose.pose.orientation.x;
				transformStamped.transform.rotation.y = odom_msg.pose.pose.orientation.y;
				transformStamped.transform.rotation.z = odom_msg.pose.pose.orientation.z;
				transformStamped.transform.rotation.w = odom_msg.pose.pose.orientation.w;
				broadcaster.sendTransform(transformStamped);
			}
		}

        last_deviceState=deviceState;
		device_state_pub.publish(deviceState);	//向ROS中发送STM32信息

        loop_rate.sleep();
    }

    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.angular.z = 0;
    pubControl();

	sendBuffer.append("~");
	char buf[1000];
	strcpy(buf, sendBuffer.c_str());
	send(sock_cli,buf,sendBuffer.length(),0);
	try_connection_rate.sleep();
	ROS_INFO("stm32 closed");
}