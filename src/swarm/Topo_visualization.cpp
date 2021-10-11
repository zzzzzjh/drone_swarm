#include <Topo_visualization.h>


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    nh.param<int>("swarm_num", swarm_num, 1);


    for(int i = 0; i < swarm_num; i++) 
    {
        string name_uav = "/uav"+to_string(i);
        drone_state_sub[i] = nh.subscribe<drone_msg::DroneState>(name_uav + "/drone_swarm/drone_state", 10, drone_state_cb[i]); //【发布】阵型
    }
    for(int i = 0; i < swarm_num; i++){
        for(int j = i+1; j < swarm_num; j++){
            meshPub[i][j]   = nh.advertise<visualization_msgs::Marker>( "/topology_visual"+to_string(i)+to_string(j), 10);  
        }
    }
    ros::Timer timer_rviz_pub = nh.createTimer(ros::Duration(0.5), timercb_topo);
    drone_command_sub = nh.subscribe<drone_msg::SwarmCommand>("/uav0/drone_swarm/swarm_command", 10, swarm_command_cb);
    TopoPub = nh.advertise<drone_msg::Topology>("/topology",10);
    while (ros::ok()) {
        ros::spinOnce();
         switch(Command_uav.swarm_shape){
            case drone_msg::SwarmCommand::Fully_Connected:
                
                topo.Fully_connected();
                
                break;

            case drone_msg::SwarmCommand::Ring_Connected:

                topo.Ring_connected();
                
                break;
            case drone_msg::SwarmCommand::Sensor_Connected:

                for(int i = 0; i < swarm_num; i++){
                    for(int j = i+1; j < swarm_num; j++){
                        if((pos_drone[i] - pos_drone[j]).norm()<Command_uav.range*1.5){
                            topo.connect(i,j);
                            topo.connect(j,i);
                        }else{
                            topo.disconnect(i,j);
                            topo.disconnect(j,i);
                        }
                    }
                }
                
                break;
            case drone_msg::SwarmCommand::Change_Connected:

                if( agent1 <= 7 && agent2 <= 7 && agent2 >= 0 && agent2 >= 0){
                    if(connect_flag){
                        topo.disconnect(agent1, agent2);
                    }else{
                        topo.connect(agent1, agent2);
                    }
                }else{
                    cout << "Wrong Input" <<endl;
                }
                



                break;
        }
        topo_pub.topology = topo.get_topo_int();
        TopoPub.publish(topo_pub);
        rate.sleep();
    }
    return 0;
}


