#ifndef FULL_CONTROL_PID_H
#define FULL_CONTROL_PID_H

#include "spdlog/spdlog.h"
#include <spdlog/sinks/basic_file_sink.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_gz_interfaces/msg/float32_array.hpp"

#include <fstream>
#include <Eigen/Dense>
#include <cmath>

#include "drone_control/PID_controller.h"
#include "drone_control/Drone.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/point.hpp"

#define GRAVITY 9.8

class DronePoseSubscriber : public rclcpp::Node
{
public:
    DronePoseSubscriber();
    void loadParams();

private:

    std::unique_ptr<Drone> drone_;
    std::unique_ptr<PIDController> AltitudePidController_;

    std::unique_ptr<PIDController> LateralXPidController_;
    std::unique_ptr<PIDController> InnerXPidController_;

    std::unique_ptr<PIDController> LateralYPidController_;
    std::unique_ptr<PIDController> InnerYPidController_;

    std::unique_ptr<PIDController> yawPidController_;

    Eigen::Vector3d goalPos;
    Eigen::Vector3d currentPos;

    std::shared_ptr<spdlog::logger> logger_;


    rclcpp::Clock clock;
    rclcpp::Time prevTime;
    rclcpp::Time curTime;

    int lateralCounter_;

    double dt;

    double errLateralX;
    double errVelX;

    double errLateralY;
    double errVelY;

    double errYaw;

    double prevVelocity;

    void topic_callback(std::shared_ptr<const nav_msgs::msg::Odometry> msg);
    void goalTopicCallback(std::shared_ptr<const geometry_msgs::msg::Point> msg);

    std::vector<Eigen::Vector3d> goalPoints;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goalSubscription_;
    rclcpp::Publisher<ros_gz_interfaces::msg::Float32Array>::SharedPtr publisher_;
};

#endif // ALTITUDE_CONTROL_PID_H
