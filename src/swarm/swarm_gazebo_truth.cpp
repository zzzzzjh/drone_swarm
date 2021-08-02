#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "gazebo_ground_truth");
    ros::NodeHandle nh("~");

    ros::Publisher drone_pos_uav0, drone_pos_uav1, drone_pos_uav2;

    ros::ServiceClient gazebo_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    drone_pos_uav0 = nh.advertise<geometry_msgs::PoseStamped>("/uav0/drone_swarm/ground_truth", 100);
    drone_pos_uav1 = nh.advertise<geometry_msgs::PoseStamped>("/uav1/drone_swarm/ground_truth", 100);
    drone_pos_uav2 = nh.advertise<geometry_msgs::PoseStamped>("/uav2/drone_swarm/ground_truth", 100);


    geometry_msgs::PoseStamped drone_pos_0, drone_pos_1, drone_pos_2;
    gazebo_msgs::GetModelState drone_pos_get_0, drone_pos_get_1, drone_pos_get_2;
    drone_pos_get_0.request.model_name = "iris_0"; 
    drone_pos_get_1.request.model_name = "iris_1"; 
    drone_pos_get_2.request.model_name = "iris_2";

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

       rate.sleep();

    }
    

    return 0;
}