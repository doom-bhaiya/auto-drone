#include "drone_control/altitude_control_PID.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#define ODOM_TOPIC_NAME "DronePose"
#define CMD_TOPIC "/cmd_vel"
#define GRAVITY 9.8


DronePoseSubscriber::DronePoseSubscriber() : Node("drone_pose_subscriber"){
    
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC_NAME, 10,
        std::bind(&DronePoseSubscriber::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<ros_gz_interfaces::msg::Float32Array>(CMD_TOPIC, 10);

    prevTime = clock.now();
    curTime = clock.now();

    loadParams();
    pidController.setParams(kp_, ki_, kd_);

}

void DronePoseSubscriber::loadParams() {

    std::string pkg_path = ament_index_cpp::get_package_share_directory("drone_sim");
    std::string config_file_path = pkg_path + "/config/drone_params.json";

    std::ifstream file(config_file_path);
    if (!file)
    {
        std::cerr << "Could not open config.json\n";
    }
    else
    {
        json config;
        file >> config;

        kf_ = config["k1"];
        km_ = config["k2"];
        mass_ = config["m"];
        MomentOfInertia_ = config["Ip"];
        propLength_ = config["length_of_prop"];
    }

    pkg_path = ament_index_cpp::get_package_share_directory("drone_control");
    config_file_path = pkg_path + "/config/PID_params.json";

    std::ifstream paramsFile(config_file_path);
    if (!paramsFile)
    {
        std::cerr << "Could not open config.json\n";
    }
    else
    {
        json config;
        paramsFile >> config;

        kp_ = config["altitude"]["kp"];
        kd_ = config["altitude"]["kd"];
        ki_ = config["altitude"]["ki"];
    }
}

void DronePoseSubscriber::topic_callback(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
{
    curTime = clock.now();
    dt = std::max((curTime - prevTime).seconds(), 1e-6);
    prevTime = curTime;

    currentPos = msg->pose.pose.position.z;
    double errTotal = pidController.compute(goalPos, currentPos, dt);

    ros_gz_interfaces::msg::Float32Array propSpeeds;

    float thrust = (errTotal + GRAVITY) * mass_;
    float thrustPerProp = thrust / 4;

    float propForce = sqrt(std::max(thrustPerProp / kf_, 0.0));

    propSpeeds.data = {propForce, propForce, propForce, propForce};
    publisher_->publish(propSpeeds);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DronePoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
