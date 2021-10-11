#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "gazebo_ground_truth");
    ros::NodeHandle nh("~");

    ros::Publisher drone_pos_uav0, drone_pos_uav1, drone_pos_uav2, drone_pos_uav3, 
        drone_pos_uav4, drone_pos_uav5, drone_pos_uav6, drone_pos_uav7;

    ros::ServiceClient gazebo_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    drone_pos_uav0 = nh.advertise<geometry_msgs::PoseStamped>("/uav0/drone_swarm/ground_truth", 100);
    drone_pos_uav1 = nh.advertise<geometry_msgs::PoseStamped>("/uav1/drone_swarm/ground_truth", 100);
    drone_pos_uav2 = nh.advertise<geometry_msgs::PoseStamped>("/uav2/drone_swarm/ground_truth", 100);
    drone_pos_uav3 = nh.advertise<geometry_msgs::PoseStamped>("/uav3/drone_swarm/ground_truth", 100);
    drone_pos_uav4 = nh.advertise<geometry_msgs::PoseStamped>("/uav4/drone_swarm/ground_truth", 100);
    drone_pos_uav5 = nh.advertise<geometry_msgs::PoseStamped>("/uav5/drone_swarm/ground_truth", 100);
    drone_pos_uav6 = nh.advertise<geometry_msgs::PoseStamped>("/uav6/drone_swarm/ground_truth", 100);
    drone_pos_uav7 = nh.advertise<geometry_msgs::PoseStamped>("/uav7/drone_swarm/ground_truth", 100);

    geometry_msgs::PoseStamped drone_pos_0, drone_pos_1, drone_pos_2, drone_pos_3,
        drone_pos_4, drone_pos_5, drone_pos_6, drone_pos_7;
    gazebo_msgs::GetModelState drone_pos_get_0, drone_pos_get_1, drone_pos_get_2, drone_pos_get_3,
        drone_pos_get_4, drone_pos_get_5, drone_pos_get_6, drone_pos_get_7;
    drone_pos_get_0.request.model_name = "iris_0"; 
    drone_pos_get_1.request.model_name = "iris_1"; 
    drone_pos_get_2.request.model_name = "iris_2";
    drone_pos_get_3.request.model_name = "iris_3";
    drone_pos_get_4.request.model_name = "iris_4";
    drone_pos_get_5.request.model_name = "iris_5";
    drone_pos_get_6.request.model_name = "iris_6";
    drone_pos_get_7.request.model_name = "iris_7";

    ros::Rate rate(100);

    while(ros::ok()){
        if (gazebo_client.call(drone_pos_get_0)){
            drone_pos_0.pose = drone_pos_get_0.response.pose;
            drone_pos_0.header.stamp = ros::Time::now();
            drone_pos_uav0.publish(drone_pos_0);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }

       if (gazebo_client.call(drone_pos_get_1)){
            drone_pos_1.pose = drone_pos_get_1.response.pose;
            drone_pos_1.header.stamp = ros::Time::now();
            drone_pos_uav1.publish(drone_pos_1);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }

       if (gazebo_client.call(drone_pos_get_2)){
            drone_pos_2.pose = drone_pos_get_2.response.pose;
            drone_pos_2.header.stamp = ros::Time::now();
            drone_pos_uav2.publish(drone_pos_2);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }

       if (gazebo_client.call(drone_pos_get_3)){
            drone_pos_3.pose = drone_pos_get_3.response.pose;
            drone_pos_3.header.stamp = ros::Time::now();
            drone_pos_uav3.publish(drone_pos_3);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }

       if (gazebo_client.call(drone_pos_get_4)){
            drone_pos_4.pose = drone_pos_get_4.response.pose;
            drone_pos_4.header.stamp = ros::Time::now();
            drone_pos_uav4.publish(drone_pos_4);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }

       if (gazebo_client.call(drone_pos_get_5)){
            drone_pos_5.pose = drone_pos_get_5.response.pose;
            drone_pos_5.header.stamp = ros::Time::now();
            drone_pos_uav5.publish(drone_pos_5);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }

       if (gazebo_client.call(drone_pos_get_6)){
            drone_pos_6.pose = drone_pos_get_6.response.pose;
            drone_pos_6.header.stamp = ros::Time::now();
            drone_pos_uav6.publish(drone_pos_6);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }

       if (gazebo_client.call(drone_pos_get_7)){
            drone_pos_7.pose = drone_pos_get_7.response.pose;
            drone_pos_7.header.stamp = ros::Time::now();
            drone_pos_uav7.publish(drone_pos_7);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }



       rate.sleep();

    }
    

    return 0;
}