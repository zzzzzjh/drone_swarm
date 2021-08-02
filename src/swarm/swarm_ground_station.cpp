//头文件
#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "swarm_ground_station.h"

using namespace std;

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_ground_station");
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num", swarm_num, 1);
    nh.param<bool>("flag_ros2groundstation", flag_ros2groundstation, false);

    for(int i = 0; i <swarm_num; i++) 
    {

        string name = "uav"+to_string(i)+"_name";
        nh.param<string>(name, uav_name[i], "/none");
        cout<<uav_name[i];
        string id = "uav"+to_string(i)+"_id";
        nh.param<int>(id, uav_id[i], 0);
        // 订阅
        command_sub[i] = nh.subscribe<drone_msg::SwarmCommand>(uav_name[i] + "/drone_swarm/swarm_command", 10, swarm_command_cb[i]);
        drone_state_sub[i] = nh.subscribe<drone_msg::DroneState>(uav_name[i] + "/drone_swarm/drone_state", 10, drone_state_cb[i]);
    }
    
    while(ros::ok())
    {
        ros::spinOnce();
        cout << ">>>>>>>>>>>>>>>>>>>> Swarm Ground Station <<<<<<<<<<<<<<<<<<< "<< endl;
        for(int i = 0; i < swarm_num; i++)
        {

                printf_swarm_state(swarm_num, uav_id[i], uav_name[i], State_uav[i], Command_uav[i]);

        }
        sleep(2.0); // frequence
    }
    return 0;
}
