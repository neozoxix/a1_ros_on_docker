/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <string>

class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);
ros::NodeHandle _nm;
ros::Subscriber _servo_sub[12], _imu_sub;
ros::Publisher _servo_pub[12];
unitree_legged_msgs::LowCmd _lowCmd;
unitree_legged_msgs::LowState _lowState;
std::string _robot_name;

double roll = 0;
double pitch = 0;
double yaw = 0;
double FR_x = 0;
double FR_y = 0;
double FR_z = 0;
double FL_x = 0;
double FL_y = 0;
double FL_z = 0;
double RR_x = 0;
double RR_y = 0;
double RR_z = 0;
double RL_x = 0;
double RL_y = 0;
double RL_z = 0;
double FRb_x = 0;
double FRb_y = 0;
double FRb_z = 0;
double FLb_x = 0;
double FLb_y = 0;
double FLb_z = 0;
double RRb_x = 0;
double RRb_y = 0;
double RRb_z = 0;
double RLb_x = 0;
double RLb_y = 0;
double RLb_z = 0;
double L[5] = {0.0838, 0.2, 0.2, 0, 0};
double TL[3] = {0.1805, 0.047, 0};

//repeated functions for multi-thread
void initRecv();
void initSend();

//Callback functions for ROS
void imuCallback(const sensor_msgs::Imu & msg);

void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif  // IOROS_H

#endif  // COMPILE_WITH_ROS