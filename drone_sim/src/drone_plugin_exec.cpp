#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_plugin_exec");
    ros::NodeHandle nh;

    ROS_INFO("Drone plugin executable running...");

    ros::spin();  // Keeps the node alive for interaction

    return 0;
}