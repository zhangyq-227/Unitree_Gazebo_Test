/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Float64MultiArray.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "body.h"
#include "torch/script.h"
#include <geometry_msgs/Twist.h>
#include "Eigen/Eigen"

using namespace std;
using namespace unitree_model;

bool start_up = true;

typedef Eigen::Matrix<double, 1, 1> Vector1d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 32, 1> Vector32d;
typedef Eigen::Matrix<double, 45, 1> VectorObservation;
typedef Eigen::Matrix<double, 45*6, 1> VectorObservationBuffer;

#define DEFAULT_POS  {-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5}

Vector3d commands;
Vector12d last_actions; 
Vector12d default_position;
Vector32d vision_latent;
bool init_flag = false;

#define hip_min -0.863
#define hip_max 0.863
#define thigh_min -0.686
#define thigh_max 4.501
#define calf_min -2.818
#define calf_max -0.888

class multiThread
{
public:
    multiThread(string rname){
        robot_name = rname;
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
        latent_sub = nm.subscribe("/vision_latent", 1, &multiThread::LatentCallback, this);
        cmd_vel = nm.subscribe("/cmd_vel",1,&multiThread::CmdCallback,this);
    }

    void LatentCallback(const std_msgs::Float32MultiArray msg)
    {
        for(int i = 0;i < 32;i++)  vision_latent[i] = static_cast<double>(msg.data[i]);

        init_flag = true;
    }

    void CmdCallback(const geometry_msgs::Twist & msg)
    {
        commands(0) = msg.linear.x*2.0;
        commands(1) = msg.linear.y*2.0;
        commands(2) = msg.angular.z*0.25;
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;
        
        lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
        
    }

    void FRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].q = msg.q;
        lowState.motorState[0].dq = msg.dq;
        lowState.motorState[0].tauEst = msg.tauEst;
    }

    void FRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].q = msg.q;
        lowState.motorState[1].dq = msg.dq;
        lowState.motorState[1].tauEst = msg.tauEst;
    }

    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].q = msg.q;
        lowState.motorState[2].dq = msg.dq;
        lowState.motorState[2].tauEst = msg.tauEst;
    }

    void FLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].q = msg.q;
        lowState.motorState[3].dq = msg.dq;
        lowState.motorState[3].tauEst = msg.tauEst;
    }

    void FLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].q = msg.q;
        lowState.motorState[4].dq = msg.dq;
        lowState.motorState[4].tauEst = msg.tauEst;
    }

    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].q = msg.q;
        lowState.motorState[5].dq = msg.dq;
        lowState.motorState[5].tauEst = msg.tauEst;
    }

    void RRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].q = msg.q;
        lowState.motorState[6].dq = msg.dq;
        lowState.motorState[6].tauEst = msg.tauEst;
    }

    void RRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].q = msg.q;
        lowState.motorState[7].dq = msg.dq;
        lowState.motorState[7].tauEst = msg.tauEst;
    }

    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].q = msg.q;
        lowState.motorState[8].dq = msg.dq;
        lowState.motorState[8].tauEst = msg.tauEst;
    }

    void RLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].q = msg.q;
        lowState.motorState[9].dq = msg.dq;
        lowState.motorState[9].tauEst = msg.tauEst;
    }

    void RLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].q = msg.q;
        lowState.motorState[10].dq = msg.dq;
        lowState.motorState[10].tauEst = msg.tauEst;
    }

    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].q = msg.q;
        lowState.motorState[11].dq = msg.dq;
        lowState.motorState[11].tauEst = msg.tauEst;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }
     
    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }
     
    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }
     
private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub;
    ros::Subscriber cmd_vel;
    ros::Subscriber latent_sub;
    string robot_name;
};

VectorObservation get_observation()
{
    // construct quaternion data
    Vector4d quaternion;
    std::vector<float> downvec{0.0f,0.0f,-1.0f};
    std::vector<float> Imu{1.0f,0.0f,0.0f,0.0f};
    for(int i = 0;i < 4;i++) Imu[i] = lowState.imu.quaternion[i];
    Eigen::Vector3d projected_gravity = quat_rotate_inverse(Imu, downvec);

    // // w,x,y,z in Unitree Real Machine, but xy,y,z,w in isaac gym. Need to be converted!
    quaternion << Imu[0], Imu[1], Imu[2], Imu[3];

    Eigen::Vector3d Euler = euler_from_quaternion(quaternion);

    // shut down the program if Robot is turn over.
    // if(abs(Euler(0)) > 1.3)
    // {
    //     std::cout << "Too Large Roll Angles!"<<std::endl;
    //     exit(-1);
    // }

    // if(abs(Euler(1)) > 1.3)
    // {
    //     std::cout << "Too Large Pitch Angles!"<<std::endl;
    //     exit(-1);
    // }

    // construct position data
    Vector12d joint_position;
     
    for(int i = 0;i < 12;i++)
        joint_position[i] = lowState.motorState[i].q - default_position[i];

    // construct velocity data
    Vector12d joint_velocity;

    for(int i = 0;i < 12;i++)
        joint_velocity[i] = lowState.motorState[i].dq;
    joint_velocity *= 0.05f;

    // ang velocity
    Vector3d base_ang_vel;
    for(int i = 0;i < 3;i++)
        base_ang_vel[i] = lowState.imu.gyroscope[i];
    base_ang_vel *= 0.25f;

    // construct the final results, 2+3+12+12+12=41
    VectorObservation obs;
    obs << base_ang_vel, projected_gravity, commands,joint_position,joint_velocity, last_actions;
    return obs;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_gazebo_servo");

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;

    multiThread listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    ros::Publisher prop_pub;
    // ros::Rate loop_rate(1000);
    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);
    prop_pub = n.advertise<std_msgs::Float64MultiArray>("/prop_obs", 1);
    commands.setZero();
    last_actions.setZero();
    default_position <<-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5;
    torch::jit::script::Module network_;
    torch::jit::script::Module walk_network_;
    try{
        network_ = torch::jit::load("/home/zhangyq227/code/gazebo_sim2sim/models/Actor_Deploy.pt");
        walk_network_ = torch::jit::load("/home/zhangyq227/code/gazebo_sim2sim/models/walk.pt");
    }
    catch(const c10::Error& e)
    {
        std::cerr << "Erro when loading neral network!"<<e.what()<<std::endl;
    }

    motion_init();
    ros::Rate loop_rate(50);
    VectorObservationBuffer obs_buffer;
    obs_buffer.setZero();

    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 28;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 0.7;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 28;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 0.7;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 28;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 0.7;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }

    while (ros::ok()){
        /*
        control logic
        */
        lowState_pub.publish(lowState);
        VectorObservation obs = get_observation();
        for(int i = 0;i < 45;i++){
            if(obs[i] > 100) obs[i] = 100.0f;
            if(obs[i] < -100) obs[i] = -100.0f;
        }

        obs_buffer.tail(5*45) << obs_buffer.head(5*45).eval();
        obs_buffer.head(45) << obs;
         
        bool danger = false;
        int cnt = 0;

        for(int i = 0;i < 4;i++)
        {
            if(lowState.motorState[3*i].q < hip_min*0.9 || lowState.motorState[3*i].q > hip_max*0.9
               || lowState.motorState[3*i+1].q < thigh_min*0.9 || lowState.motorState[3*i+1].q > 0.9*thigh_max)
               cnt++;
        }

        if(cnt >= 1) danger = true;
         
        std_msgs::Float64MultiArray array;
        array.data.resize(45);
        for(int i = 0;i < 45;i++)
            array.data[i] = obs[i];

        // publish array msgs to be used in vision embedding code. 50Hz publishment
        prop_pub.publish(array);
        
        // if(init_flag){ 
        //     // if(commands(0) >= 0){
        //         torch::Tensor input_tensor =
        //             torch::from_blob(obs_buffer.data(), {1, obs_buffer.rows()}, at::kDouble)
        //             .clone();

        //         torch::Tensor depth_encoder =
        //             torch::from_blob(vision_latent.data(), {1, vision_latent.rows()}, at::kDouble)
        //             .clone();

        //         // convert torch double tensor to torchscript float IValue
        //         std::vector<torch::jit::IValue> input_ivalue;
        //         input_ivalue.push_back(depth_encoder.to(torch::kFloat));
        //         input_ivalue.push_back(input_tensor.to(torch::kFloat));

        //         torch::Tensor output_tensor;
        //         output_tensor = network_(input_ivalue).toTensor();

        //         // output actions
        //         Vector12d output(output_tensor.to(torch::kDouble).data_ptr<double>());
        //         for(int i = 0;i < 12;i++){
        //             if(output[i] > 4.8) output[i] = 4.8f;
        //             if(output[i] < -4.8) output[i] = -4.8f;
        //         }
        //         last_actions = output;

        //         output *= 0.25;

        //         output = output + default_position;
        //         for(int i = 0;i < 12;i++)
        //             lowCmd.motorCmd[i].q = output[i];
        //     // }
        //     // else{ 
                torch::Tensor input_tensor =
                    torch::from_blob(obs_buffer.data(), {1, obs_buffer.rows()}, at::kDouble)
                    .clone();

                // convert torch double tensor to torchscript float IValue
                std::vector<torch::jit::IValue> input_ivalue;
                input_ivalue.push_back(input_tensor.to(torch::kFloat));

                torch::Tensor output_tensor;
                output_tensor = walk_network_(input_ivalue).toTensor();

                // output actions
                Vector12d output(output_tensor.to(torch::kDouble).data_ptr<double>());

                last_actions = output;
                output *= 0.25;
                 
                output = output + default_position;
                for(int i = 0;i < 12;i++)
                    lowCmd.motorCmd[i].q = output[i];

        //     // }
        // }
        // else{
        //     Vector12d output = default_position;
        //     for(int i = 0;i < 12;i++)
        //         lowCmd.motorCmd[i].q = output[i];
        // }

        // if(!danger)
        sendServoCmd();
        loop_rate.sleep();
    }
    return 0;
}
