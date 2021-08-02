#ifndef SWARM_GROUND_STATION_H
#define SWARM_GROUND_STATION_H

#include <Eigen/Eigen>
#include <math.h>

#include <drone_msg/Message.h>
#include <drone_msg/ControlCommand.h>
#include <drone_msg/SwarmCommand.h>
#include <drone_msg/DroneState.h>
#include <drone_msg/PositionReference.h>
#include <drone_msg/AttitudeReference.h>
#include <drone_msg/ControlOutput.h>
#include <drone_msg/LogMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "message_utils.h"
// #include "math_utils.h"
#include "formation_utils.h"

using namespace std;
#define NUM_POINT 2

//参数声明
int swarm_num;
const int max_swarm_num = 10; // indicate max num
string uav_name[max_swarm_num+1];
int uav_id[max_swarm_num+1];
bool flag_ros2groundstation;
drone_msg::DroneState State_uav[max_swarm_num+1];
drone_msg::SwarmCommand Command_uav[max_swarm_num+1];
geometry_msgs::PoseStamped ref_pose_uav[max_swarm_num+1];
ros::Subscriber command_sub[max_swarm_num+1];
ros::Subscriber drone_state_sub[max_swarm_num+1];
ros::Subscriber message_sub[max_swarm_num+1];
char* servInetAddr = (char *)"127.0.0.1"; //sever ips
string data;
char sendline[1024];
int socketfd;
struct sockaddr_in sockaddr;

//函数声明
void swarm_command_cb_0(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[0] = *msg; }
void swarm_command_cb_1(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[1] = *msg; }
void swarm_command_cb_2(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[2] = *msg; }
void swarm_command_cb_3(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[3] = *msg; }
void swarm_command_cb_4(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[4] = *msg; }
void swarm_command_cb_5(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[5] = *msg; }
void swarm_command_cb_6(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[6] = *msg; }
void swarm_command_cb_7(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[7] = *msg; }
void swarm_command_cb_8(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[8] = *msg; }
void swarm_command_cb_9(const drone_msg::SwarmCommand::ConstPtr& msg) { Command_uav[9] = *msg; }


void (*swarm_command_cb[max_swarm_num+1])(const drone_msg::SwarmCommand::ConstPtr&)={
    swarm_command_cb_0,swarm_command_cb_1,swarm_command_cb_2,swarm_command_cb_3,swarm_command_cb_4,
    swarm_command_cb_5,swarm_command_cb_6,swarm_command_cb_7,swarm_command_cb_8,swarm_command_cb_9
};

void drone_state_cb0(const drone_msg::DroneState::ConstPtr& msg) { State_uav[0] = *msg; }
void drone_state_cb1(const drone_msg::DroneState::ConstPtr& msg) { State_uav[1] = *msg; }
void drone_state_cb2(const drone_msg::DroneState::ConstPtr& msg) { State_uav[2] = *msg; }
void drone_state_cb3(const drone_msg::DroneState::ConstPtr& msg) { State_uav[3] = *msg; }
void drone_state_cb4(const drone_msg::DroneState::ConstPtr& msg) { State_uav[4] = *msg; }
void drone_state_cb5(const drone_msg::DroneState::ConstPtr& msg) { State_uav[5] = *msg; }
void drone_state_cb6(const drone_msg::DroneState::ConstPtr& msg) { State_uav[6] = *msg; }
void drone_state_cb7(const drone_msg::DroneState::ConstPtr& msg) { State_uav[7] = *msg; }
void drone_state_cb8(const drone_msg::DroneState::ConstPtr& msg) { State_uav[8] = *msg; }
void drone_state_cb9(const drone_msg::DroneState::ConstPtr& msg) { State_uav[9] = *msg; }
void drone_state_cb10(const drone_msg::DroneState::ConstPtr& msg) { State_uav[10] = *msg; }

void (*drone_state_cb[max_swarm_num+1])(const drone_msg::DroneState::ConstPtr&)={
    drone_state_cb0,drone_state_cb1,drone_state_cb2,drone_state_cb3,drone_state_cb4,
    drone_state_cb5, drone_state_cb6,drone_state_cb7,drone_state_cb8,drone_state_cb9,
};

void printf_swarm_state(int swarm_num, int uav_id, string uav_name, const drone_msg::DroneState& _Drone_state, const drone_msg::SwarmCommand& SwarmCommand)
{
    Eigen::MatrixXf formation;
    float x,y,z,yaw;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> "<< uav_name << " State <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << "uav_id: ["<< uav_id <<"] ";
   //是否和飞控建立起连接
    if (_Drone_state.connected == true)
    {
        cout << "[ Connected ] ";
    }
    else
    {
        cout << "[ Unconnected ] ";
    }
    //是否上锁
    if (_Drone_state.armed == true)
    {
        cout << "[ Armed ] ";
    }
    else
    {
        cout << "[ DisArmed ] ";
    }
    //是否在地面
    if (_Drone_state.landed == true)
    {
        cout << "[ Ground ] ";
    }
    else
    {
        cout << "[ Air ] ";
    }

    cout << "[ " << _Drone_state.mode<<" ] " <<endl;

    cout << "Position [X Y Z] : " << _Drone_state.position[0] << " [ m ] "<< _Drone_state.position[1]<<" [ m ] "<<_Drone_state.position[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << _Drone_state.velocity[0] << " [m/s] "<< _Drone_state.velocity[1]<<" [m/s] "<<_Drone_state.velocity[2]<<" [m/s] "<<endl;
    cout << "Attitude [R P Y] : " << _Drone_state.attitude[0] * 180/M_PI <<" [deg] "<<_Drone_state.attitude[1] * 180/M_PI << " [deg] "<< _Drone_state.attitude[2] * 180/M_PI<<" [deg] "<<endl;

    switch(SwarmCommand.Mode)
    {
        case drone_msg::SwarmCommand::Idle:
            if(SwarmCommand.yaw_ref == 999)
            {
                cout << "Command: [ Idle + Arming + Switching to OFFBOARD mode ] " <<endl;
            }else
            {
                cout << "Command: [ Idle ] " <<endl;
            }
            break;

        case drone_msg::SwarmCommand::Takeoff:
            cout << "Command: [ Takeoff ] " <<endl;
            break;

        case drone_msg::SwarmCommand::Hold:
            cout << "Command: [ Hold ] " <<endl;
            break;

        case drone_msg::SwarmCommand::Land:
            cout << "Command: [ Land ] " <<endl;
            break;

        case drone_msg::SwarmCommand::Disarm:
            cout << "Command: [ Disarm ] " <<endl;
            break;

        case drone_msg::SwarmCommand::Position_Control:
            if(SwarmCommand.swarm_shape == drone_msg::SwarmCommand::One_column)
            {
                cout << "Command: [ Position_Control ] [One_column] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == drone_msg::SwarmCommand::Triangle)
            {
                cout << "Command: [ Position_Control ] [Triangle] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == drone_msg::SwarmCommand::Square)
            {
                cout << "Command: [ Position_Control ] [Square] size: " << SwarmCommand.swarm_size <<endl;
            }else if(SwarmCommand.swarm_shape == drone_msg::SwarmCommand::Circular)
            {
                cout << "Command: [ Position_Control ] [Circular] size: " << SwarmCommand.swarm_size <<endl;
            }
            
            formation = formation_utils::get_formation_separation(SwarmCommand.swarm_shape, SwarmCommand.swarm_size, swarm_num);

            x = SwarmCommand.position_ref[0] + formation(uav_id,0);
            y = SwarmCommand.position_ref[1] + formation(uav_id,1);
            z = SwarmCommand.position_ref[2] + formation(uav_id,2);
            yaw = SwarmCommand.yaw_ref + formation(uav_id,3);

            cout << "Position [X Y Z] : " << x  << " [ m ] "<< y <<" [ m ] "<< z <<" [ m ] "<<endl;
            cout << "Yaw : "  << yaw * 180/M_PI << " [deg] " <<endl;
            break;

        case drone_msg::SwarmCommand::Velocity_Control:
            cout << "Command: [ Velocity_Control ] " <<endl;

            break;

        case drone_msg::SwarmCommand::Accel_Control:
            cout << "Command: [ Accel_Control ] " <<endl;

            break;

        case drone_msg::SwarmCommand::Move:

            if(SwarmCommand.Move_mode == drone_msg::SwarmCommand::XYZ_POS)
            {
                cout << "Command: [ Move in XYZ_POS] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.position_ref[0] << " [ m ] "<< SwarmCommand.position_ref[1]<<" [ m ] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == drone_msg::SwarmCommand::XY_VEL_Z_POS)
            {
                cout << "Command: [ Move in XY_VEL_Z_POS] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.velocity_ref[0] << " [m/s] "<< SwarmCommand.velocity_ref[1]<<" [m/s] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else if(SwarmCommand.Move_mode == drone_msg::SwarmCommand::TRAJECTORY)
            {
                cout << "Command: [ Move in TRAJECTORY] " <<endl;
                cout << "Position [X Y Z] : " << SwarmCommand.position_ref[0] << " [ m ] "<< SwarmCommand.position_ref[1]<<" [ m ] "<< SwarmCommand.position_ref[2]<<" [ m ] "<<endl;
                cout << "Velocity [X Y Z] : " << SwarmCommand.velocity_ref[0] << " [m/s] "<< SwarmCommand.velocity_ref[1]<<" [m/s] "<< SwarmCommand.velocity_ref[2]<<" [m/s] "<<endl;
                cout << "Yaw : "  << SwarmCommand.yaw_ref* 180/M_PI << " [deg] " <<endl;
            }else
            {
                cout << " wrong sub move mode. " <<endl;
            }
            break;
            
        case drone_msg::SwarmCommand::User_Mode1:
            cout << "Command: [ User_Mode1 ] " <<endl;
            break;
    }
}

#endif
