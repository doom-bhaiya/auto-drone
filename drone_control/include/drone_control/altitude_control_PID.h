#ifndef ALTITUDE_CONTROL_PID_H
#define ALTITUDE_CONTROL_PID_H


#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_gz_interfaces/msg/float32_array.hpp"

#include <fstream>
#include <Eigen/Dense>
#include <cmath>

#include "drone_control/PID_controller.h"

class DronePoseSubscriber : public rclcpp::Node
{
public:
    DronePoseSubscriber();
    void loadParams();

private:
    PIDController pidController;

    double kp_, ki_, kd_;
    double kf_, km_, mass_, I_, propLength_, MomentOfInertia_;

    double goalPos = 5;
    double currentPos = 0;

    rclcpp::Clock clock;
    rclcpp::Time prevTime;
    rclcpp::Time curTime;

    double dt;

    void topic_callback(std::shared_ptr<const nav_msgs::msg::Odometry> msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<ros_gz_interfaces::msg::Float32Array>::SharedPtr publisher_;
};

#endif // ALTITUDE_CONTROL_PID_H
