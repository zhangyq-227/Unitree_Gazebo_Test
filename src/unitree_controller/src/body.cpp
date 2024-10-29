/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"

namespace unitree_model {

ros::Publisher servo_pub[12];
unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 50;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 1;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 50;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 1;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 50;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 1;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = lowState.motorState[i].q;
    }
}

void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 2*1000);
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    // usleep(1000);
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
    }
}
Eigen::Vector3d quat_rotate_inverse(const std::vector<float>& q, const std::vector<float>& v)
{
    float q_w = q[0];
    std::vector<float> q_vec(q.begin() + 1, q.end());
    std::vector<float> a = v;
    std::vector<float> b(3), c(3);
    Eigen::Vector3d result(3);
    for (size_t j = 0; j < 3; ++j)
    {
        a[j] *= 2.0 * std::pow(q_w, 2) - 1.0;
        b[j] =
            (q_vec[(j + 1) % 3] * v[(j + 2) % 3] - q_vec[(j + 2) % 3] * v[(j + 1) % 3]) * q_w * 2.0;
        for (size_t k = 0; k < 3; ++k)
        {
            c[j] += q_vec[k] * v[k] * q_vec[j];
        }
        c[j] *= 2.0;
    }

    for (size_t j = 0; j < 3; ++j)
    {
        result[j] = a[j] - b[j] + c[j];
    }
    return result;
}

/**
 * convert quaternion to euler angle.
 * quaternion order: w, x,y,z in Unitree SDK.
*/
Eigen:: Vector3d euler_from_quaternion(Eigen::Vector4d quaternion)
{
    double w = quaternion(0); double x = quaternion(1); double y = quaternion(2); double z = quaternion(3);
    double t0 = 2.0f*(w*x + y*z);
    double t1 = 1.0f-2.0*(x*x+y*y);

    double roll_x = atan2(t0,t1);

    double t2 = 2.0f*(w*y - z*x);
    if(t2 < -1.0f)  t2 = -1.0f;
    else if (t2 > 1.0f) t2 = 1.0f;

    double pitch_y = asin(t2);

    double t3 = 2.0f*(w*z + x*y);
    double t4 = 1.0f-2.0*(y*y+z*z);
    double yaw_z = atan2(t3,t4);

    Eigen::Vector3d ans;
    ans << roll_x, pitch_y, yaw_z;
    return ans;
}


}
