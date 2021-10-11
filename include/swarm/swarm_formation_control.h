#ifndef Formation_control_H
#define Formation_control_H

//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

#include "Topology.h"
//topic 头文件
#include <drone_msg/SwarmCommand.h>
#include <drone_msg/DroneState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
using namespace std;

#define NODE_NAME "swarm_formation_control"
#define MAX_NUM 8
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int swarm_num;
string uav_name[MAX_NUM+1];
int uav_id[MAX_NUM+1];
drone_msg::SwarmCommand swarm_command[MAX_NUM+1];
Topology topo;
int controller_num;
float repel_range;
bool sim_mode;
int connect_flag, agent1, agent2;

Eigen::Vector3f direction_ref;
Eigen::Vector3f virtual_leader_pos;
Eigen::Vector3f virtual_leader_vel;
float virtual_leader_yaw;

int topology_num = 1;


ros::Publisher command_pub[MAX_NUM+1];



void pub_formation_command()
{
    if(topology_num == 1)
    {
        cout << "Formation shape: [ Fully_Connected ]"<<endl;
        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].swarm_shape = drone_msg::SwarmCommand::Fully_Connected;
        }
    }else if(topology_num == 2){
        cout << "Formation shape: [ Ring_Connected ]"<<endl;
        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].swarm_shape = drone_msg::SwarmCommand::Ring_Connected;
        }
    }else if(topology_num == 3){
        cout << "Formation shape: [ Sensor_Connected ]"<<endl;
        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].swarm_shape = drone_msg::SwarmCommand::Sensor_Connected;
        }
    }else if(topology_num == 4){
        cout << "Formation shape: [ Change_Connected ]"<<endl;
        cout << "0 for connect , 1 for disconnect" << endl;
        cin >>  connect_flag;
        cout << "input  agent1" << endl;
        cin >> agent1;
        cout << "input  agent2" << endl;
        cin >> agent2;
        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].swarm_shape = drone_msg::SwarmCommand::Change_Connected;
            swarm_command[i].connect_flag = connect_flag;
            swarm_command[i].agent1 = agent1;
            swarm_command[i].agent2 = agent2;
        }
    }
    else
    {
        cout << "Wrong formation shape!"<<endl;
    }

    if(controller_num == 0)
    {
        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].Mode = drone_msg::SwarmCommand::Position_Control;
        }
    }
    else if(controller_num == 1)
    {
        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].Mode = drone_msg::SwarmCommand::Velocity_Control;
        }
    }
    else if(controller_num == 2)
    {
        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].Mode = drone_msg::SwarmCommand::Accel_Control;
        }
    }
    // cout << "controller_num: " << controller_num << endl;

    for(int i = 0; i < swarm_num; i++) 
    {
        swarm_command[i].range = repel_range;
        swarm_command[i].direction[0] = direction_ref[0] ; 
        swarm_command[i].direction[1] = direction_ref[1] ;
        swarm_command[i].direction[2] = direction_ref[2] ;  
        swarm_command[i].velocity_ref[0] = virtual_leader_vel[0] ; 
        swarm_command[i].velocity_ref[1] = virtual_leader_vel[1] ; 
        swarm_command[i].velocity_ref[2] = virtual_leader_vel[2] ; 
        swarm_command[i].yaw_ref = virtual_leader_yaw;
        command_pub[i].publish(swarm_command[i]);
    }
    cout << "direction_ref: " << direction_ref[0] << "m "<<  direction_ref[1] << "m "<<  direction_ref[2] << "m "<< endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Formation Flight Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "swarm_num   : "<< swarm_num <<endl;
    cout << "controller_num   : "<< controller_num <<endl;
    cout << "range : "<< repel_range << " [m] "<< endl;

    for(int i = 0; i < swarm_num; i++) 
    {
        cout << "uav_name["<< i << "]" << uav_name[i] <<endl;
        cout << "uav_id["<< i << "]" << uav_id[i] <<endl;
    }  
}

#endif