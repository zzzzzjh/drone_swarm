#ifndef Topo_visualization_H
#define Topo_visualization_H

//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

#include "Topology.h"
//topic 头文件
#include <drone_msg/SwarmCommand.h>
#include <drone_msg/DroneState.h>
#include <drone_msg/Topology.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "visualization_msgs/Marker.h"
using namespace std;

#define NODE_NAME "Topo_visualization"
#define MAX_NUM 8
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int swarm_num;
string uav_name[MAX_NUM+1];
int uav_id[MAX_NUM+1];
Topology topo;
int controller_num;
bool sim_mode;
int connect_flag, agent1, agent2;
drone_msg::DroneState _DroneState[MAX_NUM];        // 无人机状态
Eigen::Vector3d pos_drone[MAX_NUM];                      // 无人机位置
Eigen::Vector3d vel_drone[MAX_NUM];                      // 无人机速度

drone_msg::SwarmCommand Command_uav;
drone_msg::Topology topo_pub;
int topology_num = 1;

ros::Subscriber drone_state_sub[MAX_NUM+1];
ros::Subscriber drone_command_sub;

ros::Publisher meshPub[MAX_NUM][MAX_NUM];
ros::Publisher TopoPub;

vector<geometry_msgs::Point>  Spring_link(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1 ){
    vector<geometry_msgs::Point> v;
    Eigen::Vector3d ps = p1 - p0,pv;
    pv[0] = -ps[1];pv[1] = ps[0]; pv[2] = 0;
    pv.normalize();
    double i, t,s,dis,x;
    x = 1.0;
    dis = ps.squaredNorm();
    geometry_msgs::Point point;
    point.x = p0[0];point.y = p0[1]; point.z = p0[2];
    v.push_back(point);
    t = 1.0/6; s=0;
    point.x = p0[0]+t*ps[0];point.y = p0[1]+t*ps[1]; point.z = p0[2]+t*ps[2];

    for(i=0;i<8;i++){
        t += 2.0/24; s = (0.5 - (2.0/24*dis)*(2.0/24*dis))>0?sqrt((0.5 - (2.0/24*dis)*(2.0/24*dis))):0;
        point.x = p0[0] + t*ps[0] + x*s*pv[0];
        point.y = p0[1]+ t*ps[1] + x*s*pv[1];
        point.z = p0[2]+ t*ps[2] + x*s*pv[2];
        v.push_back(point);
        x = -x;
    }
    // t = 5.0/6;
    // point.x = p0[0]+t*ps[0];point.y = p0[1]+t*ps[1]; point.z = p0[2]+t*ps[2];
    // v.push_back(point);
    point.x = p1[0];point.y = p1[1]; point.z = p1[2];
    v.push_back(point);

    return v;
}

vector<geometry_msgs::Point>  Direct_link(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1 ){
    vector<geometry_msgs::Point> v;
    Eigen::Vector3d ps = p1 - p0;
    double i, t,dis;
    dis = ps.squaredNorm();
    t = 0;
    geometry_msgs::Point point;
    for(i=0;i<24;i++){
        t += 1.0/24; 
        point.x = p0[0] + t*ps[0]; 
        point.y = p0[1]+ t*ps[1];
        point.z = p0[2]+ t*ps[2]; 
        v.push_back(point);
    }

    return v;
}

void timercb_topo(const ros::TimerEvent &e)
{
    visualization_msgs::Marker meshROS[MAX_NUM][MAX_NUM];
    for(int i = 0; i < swarm_num; i++){
        for(int j = i+1; j < swarm_num; j++){
                meshROS[i][j].header.frame_id  = "world";
                meshROS[i][j].header.stamp = ros::Time::now();
                meshROS[i][j].ns = "mesh";
                 meshROS[i][j].id = i*10+j;

                meshROS[i][j].type = visualization_msgs::Marker::LINE_LIST;
                meshROS[i][j].action = visualization_msgs::Marker::ADD;
                if(topo.get(i,j)){
                    meshROS[i][j].points = Direct_link(pos_drone[i],pos_drone[j]) ;
                }else{
                    meshROS[i][j].points.clear();
                }
                meshROS[i][j].pose.orientation.w = 1;
                meshROS[i][j].scale.x = 0.02;
                meshROS[i][j].scale.y = 0.02;
                meshROS[i][j].scale.z = 0.02;
                if((pos_drone[i]-pos_drone[j]).squaredNorm()>Command_uav.range){
                    meshROS[i][j].color.a = 1.0;
                    meshROS[i][j].color.r = 0.0;
                    meshROS[i][j].color.g = 1.0;
                    meshROS[i][j].color.b = 0.0;
                }else{
                    meshROS[i][j].color.a = 1.0;
                    meshROS[i][j].color.r = 0.0;
                    meshROS[i][j].color.g = 0.0;
                    meshROS[i][j].color.b = 1.0;
                }
                // meshROS[i][j].color.a = 1.0;
                // meshROS[i][j].color.r = 0.0;
                // meshROS[i][j].color.g = 1.0;
                // meshROS[i][j].color.b = 0.0;

                meshPub[i][j].publish(meshROS[i][j]);
        }
    }
}





void swarm_command_cb(const drone_msg::SwarmCommand::ConstPtr& msg) {
     Command_uav = *msg;
     connect_flag = msg->connect_flag;
     agent1 = msg->agent1; 
     agent2 = msg->agent2;
}

void drone_state_cb0(const drone_msg::DroneState::ConstPtr& msg) {
    _DroneState[0] = *msg;
    pos_drone[0]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
     vel_drone[0]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
void drone_state_cb1(const drone_msg::DroneState::ConstPtr& msg) { 
    _DroneState[1] = *msg;
    pos_drone[1]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
     vel_drone[1]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
void drone_state_cb2(const drone_msg::DroneState::ConstPtr& msg) { 
    _DroneState[2] = *msg;
    pos_drone[2]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
     vel_drone[2]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
 }
void drone_state_cb3(const drone_msg::DroneState::ConstPtr& msg) { 
    _DroneState[3] = *msg; 
    pos_drone[3]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
     vel_drone[3]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
void drone_state_cb4(const drone_msg::DroneState::ConstPtr& msg) {
     _DroneState[4] = *msg; 
     pos_drone[4]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
     vel_drone[4]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
void drone_state_cb5(const drone_msg::DroneState::ConstPtr& msg) {
     _DroneState[5] = *msg; 
     pos_drone[5]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
     vel_drone[5]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
void drone_state_cb6(const drone_msg::DroneState::ConstPtr& msg) {
     _DroneState[6] = *msg; 
     pos_drone[6]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
     vel_drone[6]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
void drone_state_cb7(const drone_msg::DroneState::ConstPtr& msg) { 
    _DroneState[7] = *msg; 
    pos_drone[7]  = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    vel_drone[7]  = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
void (*drone_state_cb[MAX_NUM+1])(const drone_msg::DroneState::ConstPtr&)={
    drone_state_cb0,drone_state_cb1,drone_state_cb2,drone_state_cb3,drone_state_cb4,
    drone_state_cb5, drone_state_cb6,drone_state_cb7,
};


#endif