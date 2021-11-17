/**
 * @file InitPose.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 设置初始的机器人位姿, 使 move_base 导航包获取的机器人初始位姿和 gazebo 中一致
 * @version 0.1
 * @date 2020-06-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/* ========================================== 头文件 ===========================================
   实验中用到的库在 CMakeLists.txt 和 package.xml 中已进行了相关设置, 同学们直接用就行
   ============================================================================================ */

// C++ STL
#include <string>
#include <sstream>
#include <iostream>
// ROS
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
// 2.2.2 补全头文件
#include <std_srvs/Empty.h>
// 2.2.3 补全头文件
#include <gazebo_msgs/GetModelState.h>


// ROS节点基类实现
#include "ExperNodeBase.h"

/* ========================================== 宏定义 ===========================================
   用到的 topic 和 service 名称均以宏定义的方式写在程序中
   ============================================================================================ */
#define MACRO_INIT_POSE_TOPIC           "/initialpose"                      // 发送初始位姿的 topic
#define MACRO_CLEAR_COST_MAP_SRV        "/move_base/clear_costmaps"         // 清除 Costmap 的 service

const double CONST_PI =                      3.141592654;                       // 圆周率 pi

/* ========================================== 程序正文 =========================================== */

/** @brief 初始化机器人位姿的节点类型 */
class InitPoseNode : public ExperNodeBase
{
public:
    /**
     * @brief 构造函数
     * @param[in] nArgc         命令行参数个数
     * @param[in] ppcArgv       命令行参数表列
     * @param[in] pcNodeName    当前节点运行时的名字
     */
    InitPoseNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {
        if(!nh.getParam ("init_x", init_x))
        {
            ROS_ERROR_STREAM("ros con't find paramber \"init_x\"");
        }
        if(!nh.getParam ("init_y", init_y))
        {
            ROS_ERROR_STREAM("ros con't find paramber \"init_y\"");
        }
        if(!nh.getParam ("init_z", init_z))
        {
            ROS_ERROR_STREAM("ros con't find paramber \"init_z\"");
        }
        if(!nh.getParam ("init_theta", init_theta))
        {
            ROS_ERROR_STREAM("ros con't find paramber \"init_theta\"");
        }
        // 2.2.1 初始化发布器
        mPubInitPose  = mupNodeHandle->advertise<geometry_msgs::PoseWithCovarianceStamped>(MACRO_INIT_POSE_TOPIC, 1);
        // 确保初始化完成
        ros::Duration(0.1).sleep();
    }

    /** @brief 析构函数 */
    ~InitPoseNode(){};

    /** @brief 主循环, override 表示当前函数强制覆写基类的同名虚函数 */
    void Run(void) override
    {
        // 设置机器人的初始位姿
        {
            // Step 2.1 设置消息
            SetInitPoseMsg();

            // Step 2.2 等待 topic 具有订阅者
            while(mPubInitPose.getNumSubscribers()==0)
            {
                ros::Duration(1,0).sleep();//1s检测一次订阅者
            }
            ros::Duration(1,0).sleep();

            // 2.2.1 发布机器人初始位姿
            mPubInitPose.publish(mMsgInitPos);
            ROS_INFO("Initilize pose OK.");
        }
    }

private:

    void SetInitPoseMsg()
    {

        // Step 1 设置消息头
        mMsgInitPos.header.stamp         = ros::Time::now();
        // 以 map 为世界坐标系, 这里的机器人位姿也是在 map 坐标系下表示的
        mMsgInitPos.header.frame_id      = "map";

        // Step 2 设置机器人初始位姿
        mMsgInitPos.pose.pose.position.x = init_x;
        mMsgInitPos.pose.pose.position.y = init_y;
        mMsgInitPos.pose.pose.position.z = init_z;

        Heading2Quat(init_theta, mMsgInitPos.pose.pose.orientation);

        // Step 3 协方差矩阵
        // 这个协方差矩阵决定了初始时刻粒子的分布. 可以设置成全0矩阵, 也可以参考 rviz 中捕获的数据设置
        // 表示位姿的不确定度
        // pt.x            pt.y               pt.z               axis.x             axis.y              axis.z
        mMsgInitPos.pose.covariance[ 0] = 0.25f; mMsgInitPos.pose.covariance[ 6] = 0.00f; mMsgInitPos.pose.covariance[12] = 0.00f; mMsgInitPos.pose.covariance[18] = 0.00f; mMsgInitPos.pose.covariance[24] = 0.00f; mMsgInitPos.pose.covariance[30] = 0.00f;   // pt.x
        mMsgInitPos.pose.covariance[ 1] = 0.00f; mMsgInitPos.pose.covariance[ 7] = 0.25f; mMsgInitPos.pose.covariance[13] = 0.00f; mMsgInitPos.pose.covariance[19] = 0.00f; mMsgInitPos.pose.covariance[25] = 0.00f; mMsgInitPos.pose.covariance[31] = 0.00f;   // pt.y
        mMsgInitPos.pose.covariance[ 2] = 0.00f; mMsgInitPos.pose.covariance[ 8] = 0.00f; mMsgInitPos.pose.covariance[14] = 0.00f; mMsgInitPos.pose.covariance[20] = 0.00f; mMsgInitPos.pose.covariance[26] = 0.00f; mMsgInitPos.pose.covariance[32] = 0.00f;   // pt.z
        mMsgInitPos.pose.covariance[ 3] = 0.00f; mMsgInitPos.pose.covariance[ 9] = 0.00f; mMsgInitPos.pose.covariance[15] = 0.00f; mMsgInitPos.pose.covariance[21] = 0.00f; mMsgInitPos.pose.covariance[27] = 0.00f; mMsgInitPos.pose.covariance[33] = 0.00f;   // axis.x
        mMsgInitPos.pose.covariance[ 4] = 0.00f; mMsgInitPos.pose.covariance[10] = 0.00f; mMsgInitPos.pose.covariance[16] = 0.00f; mMsgInitPos.pose.covariance[22] = 0.00f; mMsgInitPos.pose.covariance[28] = 0.00f; mMsgInitPos.pose.covariance[34] = 0.00f;   // axis.y
        mMsgInitPos.pose.covariance[ 5] = 0.00f; mMsgInitPos.pose.covariance[11] = 0.00f; mMsgInitPos.pose.covariance[17] = 0.00f; mMsgInitPos.pose.covariance[23] = 0.00f; mMsgInitPos.pose.covariance[29] = 0.00f; mMsgInitPos.pose.covariance[35] = 0.068f;   // axis.z
    }

    /**
     * @brief 计算偏航到四元数的转换
     * @param[in]  dYawDeg  偏航角, 角度表示
     * @param[out] quat     转换后的四元数
     */
    void Heading2Quat(double dYawDeg, geometry_msgs::Quaternion& quat)
    {
        // 实现偏航向四元数的换算
        double dFaiDiv2_rad = 0.5f * dYawDeg / CONST_PI;
        quat.x = 0.0f;
        quat.y = 0.0f;
        quat.z = sin(dFaiDiv2_rad);
        quat.w = cos(dFaiDiv2_rad);
    }

private:

    ros::Publisher                            mPubInitPose;             ///< 发布器, 负责发布机器人初始位姿

    ros::ServiceClient                        mClientClrMap;            ///< 服务客户端, 调用清除 costmap 的服务
    ros::ServiceClient                        mClientGzbPose;           ///< 服务客户端, 获取 gazebo 中机器人的初始位姿

    geometry_msgs::PoseWithCovarianceStamped  mMsgInitPos;              ///< 机器人初始位姿消息

    std_srvs::Empty                           mSrvClrMap;               ///< 清除 Costmap 使用的服务类型

    float                                     init_x;
    float                                     init_y;
    float                                     init_z;
    float                                     init_theta;

    ros::NodeHandle                           nh;
};

/**
 * @brief 主函数
 * @param[in] argc 命令行参数个数
 * @param[in] argv 命令行参数表列
 * @return int     执行返回码
 */
int main(int argc, char **argv)
{
    // 生成 ROS 节点对象
    InitPoseNode node(argc, argv, "init_pose");
    // 运行
    node.Run();
    return 0;
}