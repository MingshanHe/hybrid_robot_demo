/*
 * move_control.cpp
 *
 *  Created on: Jan 24, 2021
 *      Author: hanbing
 */
#include "move_control/move_control.h"
#include <cmath>
#define R_PI 3.1415926535898
namespace hyy_move_control
{

bool MoveControl::init()
{
	move_name_="/hyy_trajectory_controller/hyy_move_command";
	io_name_="/hyy_trajectory_controller/hyy_io_command";
	data_name_="/hyy_trajectory_controller/hyy_data_command";
	grip_name_="/hyy_trajectory_controller/hyy_grip_command";
	move_client_ = n_.serviceClient<hyy_message::robot_move>(move_name_);
	if (!move_client_)
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_);
		return false;
	}
	io_client_ = n_.serviceClient<hyy_message::robot_io>(io_name_);
	if (!io_client_)
	{
		ROS_ERROR_STREAM("Failed to call service "<<io_name_);
		return false;
	}

	data_client_ = n_.serviceClient<hyy_message::robot_move_data>(data_name_);
	if (!data_client_)
	{
		ROS_ERROR_STREAM("Failed to call service "<<data_name_);
		return false;
	}

	grip_client_ = n_.serviceClient<hyy_message::robot_grip>(grip_name_);
	if (!grip_client_)
	{
		ROS_ERROR_STREAM("Failed to call service "<<grip_name_);
		return false;
	}

	init_flag=true;
	return true;
}

int MoveControl::moveA(const std::string& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		return -1;
	}
	move_srv_.request.type="moveA";
	move_srv_.request.targetstr=target;
	move_srv_.request.target.clear();
	move_srv_.request.velocity=velocity;
	move_srv_.request.zone=zone;
	move_srv_.request.tool=tool;
	move_srv_.request.wobj=wobj;
	if (!move_client_.call(move_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_<<",err="<<move_srv_.response.result);
		return -1;
	}

	return wait_move_finish();
}
int MoveControl::moveA(std::vector<double>& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		return -1;
	}
	move_srv_.request.type="moveA";
	move_srv_.request.target=target;
	move_srv_.request.targetstr.clear();
	move_srv_.request.velocity=velocity;
	move_srv_.request.zone=zone;
	move_srv_.request.tool=tool;
	move_srv_.request.wobj=wobj;
	if (!move_client_.call(move_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_<<",err="<<move_srv_.response.result);
		return -1;
	}

	return wait_move_finish();
}
int MoveControl::moveJ(const std::string& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		return -1;
	}
	move_srv_.request.type="moveJ";
	move_srv_.request.targetstr=target;
	move_srv_.request.target.clear();
	move_srv_.request.velocity=velocity;
	move_srv_.request.zone=zone;
	move_srv_.request.tool=tool;
	move_srv_.request.wobj=wobj;
	if (!move_client_.call(move_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_<<",err="<<move_srv_.response.result);
		return -1;
	}

	return wait_move_finish();
}
int MoveControl::moveJ(std::vector<double>& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		return -1;
	}
	move_srv_.request.type="moveJ";
	move_srv_.request.target=target;
	move_srv_.request.targetstr.clear();
	move_srv_.request.velocity=velocity;
	move_srv_.request.zone=zone;
	move_srv_.request.tool=tool;
	move_srv_.request.wobj=wobj;
	if (!move_client_.call(move_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_<<",err="<<move_srv_.response.result);
		return -1;
	}

	return wait_move_finish();
}
int MoveControl::moveL(const std::string& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		return -1;
	}
	move_srv_.request.type="moveL";
	move_srv_.request.targetstr=target;
	move_srv_.request.target.clear();
	move_srv_.request.velocity=velocity;
	move_srv_.request.zone=zone;
	move_srv_.request.tool=tool;
	move_srv_.request.wobj=wobj;
	if (!move_client_.call(move_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_<<",err="<<move_srv_.response.result);
		return -1;
	}

	return wait_move_finish();
}
int MoveControl::moveL(std::vector<double>& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		return -1;
	}
	move_srv_.request.type="moveL";
	move_srv_.request.target=target;
	move_srv_.request.targetstr.clear();
	move_srv_.request.velocity=velocity;
	move_srv_.request.zone=zone;
	move_srv_.request.tool=tool;
	move_srv_.request.wobj=wobj;
	if (!move_client_.call(move_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_<<",err="<<move_srv_.response.result);
		return -1;
	}

	return wait_move_finish();
}
int MoveControl::moveC(const std::string& target,const std::string& target_mid,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		return -1;
	}
	move_srv_.request.type="moveC";
	move_srv_.request.targetstr=target+","+target_mid;
	move_srv_.request.target.clear();
	move_srv_.request.velocity=velocity;
	move_srv_.request.zone=zone;
	move_srv_.request.tool=tool;
	move_srv_.request.wobj=wobj;
	if (!move_client_.call(move_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_<<",err="<<move_srv_.response.result);
		return -1;
	}

	return wait_move_finish();
}
int MoveControl::moveC(std::vector<double>& target,std::vector<double>& target_mid,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		return -1;
	}
	move_srv_.request.type="moveA";
	move_srv_.request.target=target;
	int num=move_srv_.request.target.size();
	for (int i=0;i<num;i++)
	{
		move_srv_.request.target.push_back(target_mid[i]);
	}
	move_srv_.request.targetstr.clear();
	move_srv_.request.velocity=velocity;
	move_srv_.request.zone=zone;
	move_srv_.request.tool=tool;
	move_srv_.request.wobj=wobj;
	if (!move_client_.call(move_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<move_name_<<",err="<<move_srv_.response.result);
		return -1;
	}

	return wait_move_finish();
}

int MoveControl::wait_move_finish()
{
	if (block_flag)
	{
		//get move state
		move_srv_.request.type="move_state";
		int state=0;
		while (move_client_.call(move_srv_))
		{
			state=move_srv_.response.result;
			if (0==state)
			{
				//ROS_INFO("robot move finish");
				break;
			}
			else if (state>0)
			{
				//ROS_INFO("robot moving!!!");
				usleep(100000);
			}
			else
			{
				ROS_ERROR("robot move error!!!");
				break;
			}
		}
		return state;
	}
	else
	{
		return move_srv_.response.result;
	}

}

void MoveControl::isblock(bool value)
{
	block_flag=value;
}


bool MoveControl::getDI(int index)
{
	io_srv_.request.type="DI";
	io_srv_.request.index=index;
	if (!io_client_.call(io_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<io_name_<<",err="<<io_srv_.response.result);
		return -1;
	}
	return io_srv_.response.result;
}
bool MoveControl::setDO(int index, bool value)
{

	io_srv_.request.type="DO";
	io_srv_.request.index=index;
	io_srv_.request.value=value;
	if (!io_client_.call(io_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<io_name_<<",err="<<io_srv_.response.result);
		return -1;
	}
	return io_srv_.response.result;
}


std::vector<double> MoveControl::get_joint_data(const std::string& name)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		move_data_srv_.response.data.clear();
		return move_data_srv_.response.data;
	}
	move_data_srv_.request.type="joint";
	move_data_srv_.request.datastr=name;
	move_data_srv_.request.tool.clear();
	move_data_srv_.request.wobj.clear();
	if (!data_client_.call(move_data_srv_))
	{
		move_data_srv_.response.data.clear();
		ROS_ERROR_STREAM("Failed to call service "<<data_name_);
	}

	return move_data_srv_.response.data;
}

std::vector<double> MoveControl::get_cartesian_data(const std::string& name)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		move_data_srv_.response.data.clear();
		return move_data_srv_.response.data;
	}
	move_data_srv_.request.type="cartesian";
	move_data_srv_.request.datastr=name;
	move_data_srv_.request.tool.clear();
	move_data_srv_.request.wobj.clear();
	if (!data_client_.call(move_data_srv_))
	{
		move_data_srv_.response.data.clear();
		ROS_ERROR_STREAM("Failed to call service "<<data_name_);
	}

	return move_data_srv_.response.data;
}

std::vector<double> MoveControl::get_joint_current(void)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		move_data_srv_.response.data.clear();
		return move_data_srv_.response.data;
	}
	move_data_srv_.request.type="joint_current";
	move_data_srv_.request.datastr.clear();
	move_data_srv_.request.tool.clear();
	move_data_srv_.request.wobj.clear();
	if (!data_client_.call(move_data_srv_))
	{
		move_data_srv_.response.data.clear();
		ROS_ERROR_STREAM("Failed to call service "<<data_name_);
	}

	return move_data_srv_.response.data;
}

std::vector<double> MoveControl::get_cartesian_current(const std::string& tool,const std::string& wobj)
{
	if (!init_flag)
	{
		ROS_ERROR_STREAM("please init()");
		move_data_srv_.response.data.clear();
		return move_data_srv_.response.data;
	}
	move_data_srv_.request.type="cartesian_current";
	move_data_srv_.request.datastr.clear();
	move_data_srv_.request.tool=tool;
	move_data_srv_.request.wobj=wobj;
	if (!data_client_.call(move_data_srv_))
	{
		move_data_srv_.response.data.clear();
		ROS_ERROR_STREAM("Failed to call service "<<data_name_);
	}
	return move_data_srv_.response.data;
}

std::vector<double> MoveControl::offs(std::vector<double>& target, double x, double y, double z, double k, double p, double s)
{
	std::vector<double> of;
	of.clear();
	if (target.empty())
	{
		ROS_ERROR_STREAM("offs: data wrong");
		return of;
	}
	of.push_back(target[0]+x);
	of.push_back(target[1]+y);
	of.push_back(target[2]+z);

	double kps[3]={target[3],target[4],target[5]};
	double R0[3][3];
	double R1[3][3];
	double Rres[3][3];
	double rpy[3]={k,p,s};
	rpy2tr(kps,  R0, 2);
	rpy2tr(rpy,  R1, 2);
	Rmulti(R1,R0,Rres);
	tr2rpy(Rres, kps, 2);
	of.push_back(kps[0]);
	of.push_back(kps[1]);
	of.push_back(kps[2]);

	return of;
}

std::vector<double> MoveControl::offsrel(std::vector<double>& target, double x, double y, double z, double k, double p, double s)
{
	std::vector<double> of;
	of.clear();
	if (target.empty())
	{
		ROS_ERROR_STREAM("offsrel: data wrong");
		return of;
	}
	double R0[3][3];
	double R1[3][3];
	double Rres[3][3];
	double rpy[3]={k,p,s};
	double kps[3]={target[3],target[4],target[5]};
	rpy2tr(kps,  R0, 2);
	rpy2tr(rpy,  R1, 2);
	Rmulti(R0,R1,Rres);
	tr2rpy(Rres, kps, 2);

	double _pos[3];
	double pos[3]={x,y,z};
	RMultVec(R0, pos, _pos);

	of.push_back(target[0]+_pos[0]);
	of.push_back(target[1]+_pos[1]);
	of.push_back(target[2]+_pos[2]);
	of.push_back(kps[0]);
	of.push_back(kps[1]);
	of.push_back(kps[2]);

	return of;
}

int MoveControl::grip_creat(const std::string& name)
{
	grip_srv_.request.type=name;
	grip_srv_.request.value=2;
	if (!grip_client_.call(grip_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<grip_name_<<",err="<<grip_srv_.response.result);
		return -1;
	}
	return grip_srv_.response.result;
}

int MoveControl::grip_control(const std::string& name,double position)
{
	grip_srv_.request.type=name;
	grip_srv_.request.value=position;
	if (!grip_client_.call(grip_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<grip_name_<<",err="<<grip_srv_.response.result);
		return -1;
	}
	return grip_srv_.response.result;
}

int MoveControl::grip_destroy(const std::string& name)
{
	grip_srv_.request.type=name;
	grip_srv_.request.value=-1;
	if (!grip_client_.call(grip_srv_))
	{
		ROS_ERROR_STREAM("Failed to call service "<<grip_name_<<",err="<<grip_srv_.response.result);
		return -1;
	}
	return grip_srv_.response.result;
}

void MoveControl::tr2rpy(double R[3][3], double* rpy, int flag)
{
	double eps = 0.000001;
	double s = 0.0;
	double c = 0.0;
	double cp = 0.0;
	//double Q = 0.0;
	double m=0.0;

	//matlab 四元数
	double qs=0;
	double kx=0;
	double ky=0;
	double kz=0;
	double kx1=0;
	double ky1=0;
	double kz1=0;
	int add=0;
	double nm=0;
	switch (flag)
	{
	case 0://XYZ order
		if (fabs(R[2][2])<eps&&fabs(R[1][2])<eps)
		{
			//singularity
			rpy[0] = 0;//roll is zero
			rpy[1] =R_PI/2;// atan2(R[0][2], R[2][2]);//pitch
			rpy[2] = atan2(R[1][0], R[1][1]);//yaw is sum of roll+yaw

		}
		else
		{
			rpy[0] = atan2(-R[1][2], R[2][2]);//roll
			s = sin(rpy[0]);
			c = cos(rpy[0]);
			rpy[1] = atan2(R[0][2], c*R[2][2] - s*R[1][2]);//pitch
			rpy[2] = atan2(-R[0][1], R[0][0]);//yaw

		}
		break;
	case 1://ZYX order
		if (fabs(R[0][0])<eps&&fabs(R[1][0])<eps)
		{
			//singularity
			rpy[0] = 0;//roll is zero
			rpy[1] = atan2(-R[2][0], R[0][0]);//pitch
			rpy[2] = atan2(-R[1][2], R[1][1]);//yaw is sum of roll+yaw
		}
		else
		{
			rpy[0] = atan2(R[1][0], R[0][0]);//roll
			s = sin(rpy[0]);
			c = cos(rpy[0]);
			rpy[1] = atan2(-R[2][0], c*R[0][0] + s*R[1][0]);//pitch
			rpy[2] = atan2(s*R[0][2] - c*R[1][2], c*R[1][1] - s*R[0][1]);//yaw

		}
		break;
	case 2://XYZ HLHN J.CRAIG method
		rpy[1] = atan2(-R[2][0],sqrt(R[0][0]*R[0][0]+R[1][0]*R[1][0])); //pitch
		if (fabs(fabs(rpy[1]) - R_PI / 2.0) < eps)
		{
			if (rpy[1] > 0) //pi/2
			{
				rpy[1] = R_PI / 2.0;
				rpy[2] = 0.0;
				rpy[0] = atan2(R[0][1],R[1][1]);
			}
			else//-pi/2
			{
				rpy[1] = -R_PI / 2.0;
				rpy[2] = 0.0;
				rpy[0] = -atan2(R[0][1], R[1][1]);
			}
		}
		else
		{
			cp = cos(rpy[1]);
			rpy[2] = atan2(R[1][0]/cp,R[0][0]/cp);
			rpy[0] = atan2(R[2][1]/cp, R[2][2]/cp);
		}
		break;
	case 3://��Чת�ᷨ  //orocos
		cp=(R[0][0]+R[1][1]+R[2][2]-1)/2.0;
		if (cp>1-(eps*(1e-6)))
		{
        	rpy[0] = 0;
        	rpy[1] = 0;
        	rpy[2] = 0;
		}else if (cp<-1+(eps*(1e-6)))
		{
			rpy[0] = sqrt( (R[0][0]+1.0)/2);
			rpy[1] = sqrt( (R[1][1]+1.0)/2);
			rpy[2] = sqrt( (R[2][2]+1.0)/2);
			if ( R[0][2]< 0) rpy[0] =-rpy[0] ;
			if ( R[2][1]< 0) rpy[1] =-rpy[1] ;
			if ( rpy[0]*rpy[1]*R[0][1] < 0) rpy[0]=-rpy[0];  // this last line can be necessary when z is 0
			// z always >= 0
			// if z equal to zero
			rpy[0] = rpy[0]*R_PI;
			rpy[1] = rpy[1]*R_PI;
			rpy[2] = rpy[2]*R_PI;
		}
		else
		{
//			Q=acos(cp);
//
//	    	rpy[0] = Q*0.5*(R[2][1]-R[1][2])/sin(Q);
//	    	rpy[1] = Q*0.5*(R[0][2]-R[2][0])/sin(Q);
//	    	rpy[2] = Q*0.5*(R[1][0]-R[0][1])/sin(Q);


			 double angle;
			double mod_axis;
			double axisx, axisy, axisz;
			axisx = R[2][1]-R[1][2];
			axisy = R[0][2]-R[2][0];
			axisz = R[1][0]-R[0][1];
			mod_axis = sqrt(axisx*axisx+axisy*axisy+axisz*axisz);
			angle = atan2(mod_axis/2,cp);
			rpy[0] = angle*(R[2][1]-R[1][2])/mod_axis;
			rpy[1] = angle*(R[0][2]-R[2][0])/mod_axis;
			rpy[2] = angle*(R[1][0]-R[0][1])/mod_axis;
		}


		break;
	case 4://单位四元数
			m=0.5*sqrt(1+R[0][0]+R[1][1]+R[2][2]);
			if (fabs(m)<eps)
			{
				ROS_ERROR_STREAM("quaternion no solution!\n");
				break;
			}
			rpy[3]=m;
			rpy[0]=(R[2][1]-R[1][2])/(4*rpy[3]);
			rpy[1]=(R[0][2]-R[2][0])/(4*rpy[3]);
			rpy[2]=(R[1][0]-R[0][1])/(4*rpy[3]);
			break;
	case 5: //matlab 机器人工具箱
		 qs = sqrt((R[0][0]+R[1][1]+R[2][2])+1)/2.0;
		 kx = R[2][1] - R[1][2];   // Oz - Ay
		 ky =R[0][2] - R[2][0];   //Ax - Nz
		 kz = R[1][0] - R[0][1];   // Ny - Ox

		 if ((R[0][0]>=R[1][1]) && (R[0][0]>= R[2][2]) )
		 {
				kx1 = R[0][0]- R[1][1] - R[2][2] + 1; // Nx - Oy - Az + 1
				ky1 = R[1][0] + R[0][1];          // Ny + Ox
				kz1 = R[2][0]+ R[0][2];          // Nz + Ax
				if (kx >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		 }
		 else if  (R[1][1]>= R[2][2])
		 {
		        kx1 = R[1][0]+ R[0][1];          //Ny + Ox
		        ky1 = R[1][1]- R[0][0]- R[2][2] + 1; // Oy - Nx - Az + 1
		        kz1 = R[2][1] + R[1][2];          // Oz + Ay
				if (ky >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		 }
		else
		{
				kx1 = R[2][0] + R[0][2];          // Nz + Ax
				ky1 = R[2][1]+ R[1][2];          //Oz + Ay
				kz1 = R[2][2] - R[0][0] - R[1][1]+ 1; // Az - Nx - Oy + 1
				if (kz >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		}
		 if( add)
		 {
				 kx = kx + kx1;
				 ky = ky + ky1;
				 kz = kz + kz1;
		 }
		 else
		 {
				 kx = kx - kx1;
				 ky = ky - ky1;
				 kz = kz - kz1;
		 }
		 nm=sqrt(kx*kx+ky*ky+kz*kz);
		 if (nm<0.000001)
		 {
			 	 rpy[3]=1;rpy[0]=0;rpy[1]=0;rpy[2]=0;

		 }
		 else
		 {
			 	 s=sqrt(1-qs*qs)/nm;
			 	rpy[3]=qs;rpy[0]=s*kx;rpy[1]=s*ky;rpy[2]=s*kz;

		 }
		break;
	default:
		break;
	}

}

/*//rpyת��̬����
*rpy     ��ת�ǣ������ǣ�ƫת��(����)
*R       ĩ����̬������
*flag    0:XYZ order 1:ZYX order 2:XYZ order
*/

void MoveControl::rpy2tr(double* rpy, double R[3][3], int flag)
{
	double sx = 0;
	double cx = 0;
	double sy = 0;
	double cy = 0;
	double sz = 0;
	double cz = 0;

	double Q = 0;
	double k[3];
	double v=0;

	/*
	Rx = [
	1   0    0
	0   cx  -sx
	0   sx   cx
	];
	Ry = [
	cy  0   sy
	0   1   0
	-sy  0   cy
	];
	Rz = [
	cz  -sz  0
	sz   cz  0
	0    0   1
	];
	*/
	switch (flag)
	{
	case 0://XYZ order
		sx = sin(rpy[0]);
		cx = cos(rpy[0]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[2]);
		cz = cos(rpy[2]);
		//R=Rx*Ry*Rz
		R[0][0] = cy*cz; R[0][1] = -cy*sz; R[0][2] = sy;
		R[1][0] = cx*sz + cz*sx*sy; R[1][1] = cx*cz - sx*sy*sz; R[1][2] = -cy*sx;
		R[2][0] = sx*sz - cx*cz*sy; R[2][1] = cz*sx + cx*sy*sz; R[2][2] = cx*cy;
		break;
	case 1://ZYX order
		sx = sin(rpy[2]);
		cx = cos(rpy[2]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[0]);
		cz = cos(rpy[0]);
		//R=Rz*Ry*Rx
		R[0][0] = cy*cz; R[0][1] = cz*sx*sy - cx*sz; R[0][2] = sx*sz + cx*cz*sy;
		R[1][0] = cy*sz; R[1][1] = cx*cz + sx*sy*sz; R[1][2] = cx*sy*sz - cz*sx;
		R[2][0] = -sy; R[2][1] = cy*sx; R[2][2] = cx*cy;
		break;
	case 2://XYZ HLHN J.CRAIG method
		sx = sin(rpy[0]);
		cx = cos(rpy[0]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[2]);
		cz = cos(rpy[2]);
		//R=Rz*Ry*Rx
		R[0][0] = cy*cz; R[0][1] = cz*sx*sy - cx*sz; R[0][2] = sx*sz + cx*cz*sy;
		R[1][0] = cy*sz; R[1][1] = cx*cz + sx*sy*sz; R[1][2] = cx*sy*sz - cz*sx;
		R[2][0] = -sy; R[2][1] = cy*sx; R[2][2] = cx*cy;
		break;
	case 3://��Чת�ᷨ
		Q=sqrt(rpy[0]*rpy[0]+rpy[1]*rpy[1]+rpy[2]*rpy[2]);
		if (Q<1e-10)
		{
	        R[0][0] = 1;	   R[0][1] = 0;	 R[0][2] = 0;
	        R[1][0] = 0;    R[1][1] = 1;   R[1][2] = 0;
	        R[2][0] = 0;	   R[2][1] = 0;   R[2][2] = 1;
        	//Rdebug("rpy2tr no solution\n");
        	break;
		}
        k[0]=rpy[0]/Q;
        k[1]=rpy[1]/Q;
        k[2]=rpy[2]/Q;
        sx=sin(Q);cx=cos(Q);v=1-cx;
        R[0][0] = k[0]*k[0]*v + cx;			R[0][1] = k[0]*k[1]*v - k[2]*sx;	R[0][2] = k[0]*k[2]*v + k[1]*sx;
        R[1][0] = k[0]*k[1]*v + k[2]*sx;	R[1][1] = k[1]*k[1]*v + cx;			R[1][2] = k[1]*k[2]*v - k[0]*sx;
        R[2][0] = k[0]*k[2]*v - k[1]*sx;	R[2][1] = k[1]*k[2]*v + k[0]*sx;	R[2][2] = k[2]*k[2]*v + cx;
		break;
	case 4://单位四元数
			R[0][0] = 1-2*rpy[1]*rpy[1]-2*rpy[2]*rpy[2]; R[0][1] = 2*(rpy[0]*rpy[1]-rpy[2]*rpy[3]); R[0][2] = 2*(rpy[0]*rpy[2]+rpy[1]*rpy[3]);
			R[1][0] = 2*(rpy[0]*rpy[1]+rpy[2]*rpy[3]); R[1][1] = 1-2*rpy[0]*rpy[0]-2*rpy[2]*rpy[2]; R[1][2] = 2*(rpy[1]*rpy[2]-rpy[0]*rpy[3]);
			R[2][0] = 2*(rpy[0]*rpy[2]-rpy[1]*rpy[3]); R[2][1] = 2*(rpy[1]*rpy[2]+rpy[0]*rpy[3]); R[2][2] = 1-2*rpy[0]*rpy[0]-2*rpy[1]*rpy[1];
			break;
	case 5: //matlab 机器人工具箱
		    R[0][0] = 1-2*rpy[1]*rpy[1]-2*rpy[2]*rpy[2]; R[0][1] = 2*(rpy[0]*rpy[1]-rpy[2]*rpy[3]); R[0][2] = 2*(rpy[0]*rpy[2]+rpy[1]*rpy[3]);
			R[1][0] = 2*(rpy[0]*rpy[1]+rpy[2]*rpy[3]); R[1][1] = 1-2*rpy[0]*rpy[0]-2*rpy[2]*rpy[2]; R[1][2] = 2*(rpy[1]*rpy[2]-rpy[0]*rpy[3]);
			R[2][0] = 2*(rpy[0]*rpy[2]-rpy[1]*rpy[3]); R[2][1] = 2*(rpy[1]*rpy[2]+rpy[0]*rpy[3]); R[2][2] = 1-2*rpy[0]*rpy[0]-2*rpy[1]*rpy[1];
		break;
	default:
		break;
	}

}

void MoveControl::Rmulti(double R0[3][3],double R1[3][3],double Rres[3][3])
{
	int i=0;
	int j=0;
	int k=0;
	for (i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			Rres[i][j]=0;
			for (k=0;k<3;k++)
			{
				Rres[i][j]=Rres[i][j]+R0[i][k]*R1[k][j];
			}
		}
	}
}

void MoveControl::RMultVec(double(*R)[3], double* v, double * vres)
{
	vres[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
	vres[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
	vres[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

}

