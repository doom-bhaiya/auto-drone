#include "drone_control/altitude_control_PID.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#define ODOM_TOPIC_NAME "DronePose"
#define CMD_TOPIC "/cmd_vel"



DronePoseSubscriber::DronePoseSubscriber() : Node("drone_pose_subscriber"){
    
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC_NAME, 10,
        std::bind(&DronePoseSubscriber::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to odom topic : " << ODOM_TOPIC_NAME);

    publisher_ = this->create_publisher<ros_gz_interfaces::msg::Float32Array>(CMD_TOPIC, 10);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing to cmdVel topic : " << CMD_TOPIC);

    prevTime = clock.now();
    curTime = clock.now();

    loadParams();

}

void DronePoseSubscriber::loadParams() {

    RCLCPP_INFO_STREAM(this->get_logger(), "Loading Params ");

    std::string sim_pkg_path = ament_index_cpp::get_package_share_directory("drone_sim");
    std::string control_pkg_path = ament_index_cpp::get_package_share_directory("drone_control");
    
    std::string drone_config_file_path = sim_pkg_path + "/config/drone_params.json";
    std::string pid_config_file_path = control_pkg_path + "/config/PID_params.json";

    drone_ = std::make_unique<Drone>(drone_config_file_path);
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded Drone Params from " << drone_config_file_path);

    pidController_ = std::make_unique<PIDController>(pid_config_file_path, "altitude");
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded Altitude PID Params from " << pid_config_file_path);

}

void DronePoseSubscriber::topic_callback(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
{
    curTime = clock.now();
    dt = std::max((curTime - prevTime).seconds(), 1e-6);
    prevTime = curTime;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "dt : " << dt);

    currentPos = msg->pose.pose.position.z;
    double errTotal = pidController_->compute(goalPos, currentPos, dt);
    std::cout << errTotal << std::endl;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Total Error from Altitude PID : " << errTotal);

    ros_gz_interfaces::msg::Float32Array propSpeeds;

    double u1 = drone_->calculateThrust(errTotal);
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;


    Eigen::Vector4f propForce = drone_->calculatePropSpeeds(u1, u2, u3, u4);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing propSpeeds : " << propForce(0) << ", " << propForce(1) << ", " << 
                                                                        propForce(2) << ", " << propForce(3));

    propSpeeds.data = {propForce(0), propForce(1), propForce(2), propForce(3)};
    publisher_->publish(propSpeeds);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DronePoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
