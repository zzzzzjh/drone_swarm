#include <swarm_formation_control.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    
    nh.param<int>("swarm_num", swarm_num, 1);
    // 0代表位置追踪模式，１代表速度追踪模式，２代表加速度追踪模式 
    nh.param<int>("controller_num", controller_num, 0);
    nh.param<float>("repel_range", repel_range, 1.0);
    nh.param<bool>("sim_mode",sim_mode,true);

    direction_ref << 0.0,0.0,1.0;
    virtual_leader_yaw = 0.0;

    for(int i = 0; i < swarm_num; i++) 
    {
        string name_uav = "/uav"+to_string(i);
        command_pub[i] = nh.advertise<drone_msg::SwarmCommand>(name_uav + "/drone_swarm/swarm_command", 10); //【发布】阵型
    }

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    // Waiting for input
    int start_flag = 0;

    printf_param();

    while(sim_mode && (start_flag == 0))
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to disarm all the UAVs."<<endl;
        cin >> start_flag;

        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].Mode = drone_msg::SwarmCommand::Idle;
            swarm_command[i].yaw_ref = 999;
            command_pub[i].publish(swarm_command[i]); //【发布】阵型
        }
    }

    start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff all the UAVs."<<endl;
        cin >> start_flag;

        for(int i = 0; i < swarm_num; i++) 
        {
            swarm_command[i].Mode = drone_msg::SwarmCommand::Takeoff;
            swarm_command[i].yaw_ref = 0.0;
            command_pub[i].publish(swarm_command[i]); //【发布】阵型
        }
    }
    
    float trajectory_total_time;
    while (ros::ok()) // todo: only check start_flag=0, other function need tested
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the action: 0 for Mission, 1 for Repel Vector, 2 for Hold, 3 for Land, 5 for Disarm..."<<endl;
        cin >> start_flag;
        if (start_flag == 0)
        {
            cout << "Please choose the topology: 1 for Fully Connected, 2 for Ring Connected, 3 Sensor Connected, 4 for Change Connected ..."<<endl;
            cin >> topology_num;
            pub_formation_command();
        }
        else if (start_flag == 1)
        {
            cout << "Please enter the Repel direction:"<<endl;
            cout << "range  "<<endl;
            cin >> repel_range;
            cout << "Direction -x "<<endl;
            cin >> direction_ref[0];
            cout << "Direction: -y"<<endl;
            cin >> direction_ref[1];
            cout << "Direction: -z"<<endl;
            cin >> direction_ref[2];
            cout << "virtual_leader_yaw [deg]:"<<endl;
            cin >> virtual_leader_yaw;
            virtual_leader_yaw = virtual_leader_yaw/180.0*M_PI;

            pub_formation_command();
        }
        else if (start_flag == 2)
        {
            for(int i = 0; i < swarm_num; i++) 
            {
                swarm_command[i].Mode = drone_msg::SwarmCommand::Hold;
                command_pub[i].publish(swarm_command[i]); //【发布】阵型
            }
        }
        else if (start_flag == 3)
        {
            for(int i = 0; i < swarm_num; i++) 
            {
                swarm_command[i].Mode = drone_msg::SwarmCommand::Land;
                command_pub[i].publish(swarm_command[i]); //【发布】阵型
            }
        }
        else if (start_flag == 5)
        {
            for(int i = 0; i < swarm_num; i++) 
            {
                swarm_command[i].Mode = drone_msg::SwarmCommand::Disarm;
                command_pub[i].publish(swarm_command[i]); //【发布】阵型
            }
        }
        else
        {
            cout << "Wrong input."<<endl;
        }
        
        cout << "direction_ref [X Y] : " << direction_ref[0] << " [ m ] "<< direction_ref[1] <<" [ m ] "<< direction_ref[2] <<" [ m ] "<< endl;
        cout << "virtual_leader_yaw: " << virtual_leader_yaw/M_PI*180.0 <<" [ deg ] "<< endl;
     
        ros::Duration(1.0).sleep();
    }
    return 0;
}


