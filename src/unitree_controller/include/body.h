/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#include "Eigen/Dense"

#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern unitree_legged_msgs::LowCmd lowCmd;
extern unitree_legged_msgs::LowState lowState;

void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);
Eigen::Vector3d quat_rotate_inverse(const std::vector<float>& q, const std::vector<float>& v);
Eigen::Vector3d euler_from_quaternion(Eigen::Vector4d quaternion);
}

#endif
