#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo_msgs/GetModelState.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "gazebo_ground_truth");
    ros::NodeHandle nh("~");

    ros::Publisher drone_pos_pub;

    ros::ServiceClient gazebo_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    drone_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/gazebo_ground_truth/fly_pos", 100);

    geometry_msgs::PoseStamped drone_pos;
    gazebo_msgs::GetModelState drone_pos_get;
    drone_pos_get.request.model_name = "p450"; 

    ros::Rate rate(100);

    while(ros::ok()){
        if (gazebo_client.call(drone_pos_get)){
            drone_pos.pose = drone_pos_get.response.pose;
            drone_pos.header.stamp = ros::Time::now();
            drone_pos_pub.publish(drone_pos);
        }
        else
       {
           ROS_ERROR("Failed to call service /gazebo/get_model_state");
       }

       rate.sleep();

    }
    

    return 0;
}