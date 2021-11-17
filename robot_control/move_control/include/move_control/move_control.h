/*
 * move_control.h
 *
 *  Created on: Jan 24, 2021
 *      Author: hanbing
 */

#ifndef MOVE_CONTROL_INCLUDE_MOVE_CONTROL_MOVE_CONTROL_H_
#define MOVE_CONTROL_INCLUDE_MOVE_CONTROL_MOVE_CONTROL_H_

#include "ros/ros.h"
#include "hyy_message/robot_move.h"
#include "hyy_message/robot_io.h"
#include "hyy_message/robot_move_data.h"
#include "hyy_message/robot_grip.h"
namespace hyy_move_control
{
class MoveControl
{
public:
	MoveControl(ros::NodeHandle &n) {n_=n;init_flag=false;block_flag=true;}
	~MoveControl() {}

	bool init();
	int moveA(const std::string& target,const std::string& velocity="v100",const std::string& zone="z0",const std::string& tool="tool0",const std::string& wobj="wobj0");
	int moveA(std::vector<double>& target,const std::string& velocity="v100",const std::string& zone="z0",const std::string& tool="tool0",const std::string& wobj="wobj0");
	int moveJ(const std::string& target,const std::string& velocity="v100",const std::string& zone="z0",const std::string& tool="tool0",const std::string& wobj="wobj0");
	int moveJ(std::vector<double>& target,const std::string& velocity="v100",const std::string& zone="z0",const std::string& tool="tool0",const std::string& wobj="wobj0");
	int moveL(const std::string& target,const std::string& velocity="v100",const std::string& zone="z0",const std::string& tool="tool0",const std::string& wobj="wobj0");
	int moveL(std::vector<double>& target,const std::string& velocity="v100",const std::string& zone="z0",const std::string& tool="tool0",const std::string& wobj="wobj0");
	int moveC(const std::string& target,const std::string& target_mid,const std::string& velocity="v100",const std::string& zone="z0",const std::string& tool="tool0",const std::string& wobj="wobj0");
	int moveC(std::vector<double>& target,std::vector<double>& target_mid,const std::string& velocity="v100",const std::string& zone="z0",const std::string& tool="tool0",const std::string& wobj="wobj0");
	bool getDI(int index);
	bool setDO(int index, bool value);

	std::vector<double> get_joint_data(const std::string& name);

	std::vector<double> get_cartesian_data(const std::string& name);

	std::vector<double> get_joint_current(void);

	std::vector<double> get_cartesian_current(const std::string& tool="tool0",const std::string& wobj="wobj0");

	void isblock(bool value);

	std::vector<double> offs(std::vector<double>& target, double x, double y, double z, double k, double p, double s);
	std::vector<double> offsrel(std::vector<double>& target, double x, double y, double z, double k, double p, double s);

	int grip_creat(const std::string& name);
	int grip_control(const std::string& name,double position);
	int grip_destroy(const std::string& name);
private:
	ros::NodeHandle n_;
	ros::ServiceClient move_client_;
	ros::ServiceClient io_client_;
	ros::ServiceClient data_client_;
	ros::ServiceClient grip_client_;
	hyy_message::robot_move move_srv_;
	hyy_message::robot_io io_srv_;
	hyy_message::robot_move_data move_data_srv_;
	hyy_message::robot_grip grip_srv_;
	std::string move_name_;
	std::string io_name_;
	std::string data_name_;
	std::string grip_name_;
	bool init_flag;
	bool block_flag;

	int wait_move_finish();
	void rpy2tr(double* rpy, double R[3][3], int flag);
	void tr2rpy(double R[3][3], double* rpy, int flag);
	void Rmulti(double R0[3][3],double R1[3][3],double Rres[3][3]);
	void RMultVec(double(*R)[3], double* v, double * vres);
};


}

#endif /* MOVE_CONTROL_INCLUDE_MOVE_CONTROL_MOVE_CONTROL_H_ */
