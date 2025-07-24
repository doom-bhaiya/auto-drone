#include "drone_control/full_control_PID.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#define ODOM_TOPIC_NAME "DronePose"
#define GOAL_TOPIC_NAME "Goal"
#define CMD_TOPIC "/cmd_vel"

#define INNER_LOOP_FREQUENCY 100
#define OUTER_LOOP_FREQUENCY 10
#define SKIPS_FOR_OUTER_LOOP 10


DronePoseSubscriber::DronePoseSubscriber() : Node("drone_pose_subscriber"){
    
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC_NAME, 10,
        std::bind(&DronePoseSubscriber::topic_callback, this, std::placeholders::_1));

    goalSubscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        GOAL_TOPIC_NAME, 10,
        std::bind(&DronePoseSubscriber::goalTopicCallback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to odom topic : " << ODOM_TOPIC_NAME);

    publisher_ = this->create_publisher<ros_gz_interfaces::msg::Float32Array>(CMD_TOPIC, 10);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing to cmdVel topic : " << CMD_TOPIC);

    prevTime = clock.now();
    curTime = clock.now();

    loadParams();

    prevVelocity = 0;

    logger_ = spdlog::get("DronePoseSubscriber_logger");

}

void DronePoseSubscriber::loadParams() {

    RCLCPP_INFO_STREAM(this->get_logger(), "Loading Params ");

    std::string sim_pkg_path = ament_index_cpp::get_package_share_directory("drone_sim");
    std::string control_pkg_path = ament_index_cpp::get_package_share_directory("drone_control");
    
    std::string drone_config_file_path = sim_pkg_path + "/config/drone_params.json";
    std::string pid_config_file_path = control_pkg_path + "/config/PID_params.json";

    drone_ = std::make_unique<Drone>(drone_config_file_path);

    AltitudePidController_ = std::make_unique<PIDController>(pid_config_file_path, "altitude");

    LateralXPidController_ = std::make_unique<PIDController>(pid_config_file_path, "lateralX");
    InnerXPidController_ = std::make_unique<PIDController>(pid_config_file_path, "XInner");

    LateralYPidController_ = std::make_unique<PIDController>(pid_config_file_path, "lateralY");
    InnerYPidController_ = std::make_unique<PIDController>(pid_config_file_path, "YInner");

    yawPidController_ = std::make_unique<PIDController>(pid_config_file_path, "yaw");

}

void DronePoseSubscriber::goalTopicCallback(std::shared_ptr<const geometry_msgs::msg::Point> msg){

    double x = msg->x;
    double y = msg->y;
    double z = msg->z;

    Eigen::Vector3d goal = {x, y, z};
    goalPoints.push_back(goal);
    logger_->info("Received Goal: x = {}, y = {}, z = {}", x, y, z);

}

void DronePoseSubscriber::topic_callback(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
{  
    currentPos(0) = msg->pose.pose.position.x;
    currentPos(1) = msg->pose.pose.position.y;
    currentPos(2) = msg->pose.pose.position.z;

    if (goalPoints.size() == 0) goalPos = currentPos;
    if (goalPos(0) - currentPos(0) < 0.1 &&
        goalPos(1) - currentPos(1) < 0.1 &&
        goalPos(2) - currentPos(2) < 0.1 &&
        goalPoints.size() > 1) {
        goalPos = goalPoints.front();
        goalPoints.erase(goalPoints.begin());
    }

    lateralCounter_++;



    geometry_msgs::msg::Quaternion quat_msg = msg->pose.pose.orientation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(quat_msg, tf_q);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Eigen::Vector3d euler = {roll, pitch, yaw};

    drone_->setOrientation(euler);


    curTime = clock.now();
    dt = std::max((curTime - prevTime).seconds(), 1e-6);

    double curVelocity = msg->twist.twist.linear.x;
    double curAcceleration = (curVelocity - prevVelocity) / dt;
    prevVelocity = curVelocity;
    prevTime = curTime;

    if (lateralCounter_ == SKIPS_FOR_OUTER_LOOP){
        errLateralX = LateralXPidController_->compute(goalPos(0), currentPos(0), dt * SKIPS_FOR_OUTER_LOOP);
        errLateralY = LateralYPidController_->compute(goalPos(1), currentPos(1), dt * SKIPS_FOR_OUTER_LOOP);
        lateralCounter_ = 0;
    }
    double errAngularX = InnerXPidController_->compute(- errLateralX, euler(1), dt);
    double errAngularY = InnerYPidController_->compute(- errLateralY, -euler(0), dt);

    double errAltitude = AltitudePidController_->compute(goalPos(2), currentPos(2), dt);

    double errYaw = yawPidController_->compute(0, euler(2), dt);

    logger_->debug("Goal \tX : {:.3f},\tY : {:.3f},\tZ : {:.3f}", goalPos(0), goalPos(1), goalPos(2));
    logger_->debug("Cur \tX : {:.3f},\tY : {:.3f},\tZ : {:.3f}", currentPos(0), currentPos(1), currentPos(2));
    logger_->debug("Error \tX : {:.3f},\tY : {:.3f},\tZ : {:.3f}", errLateralX, errLateralY, errAltitude);

    logger_->debug("Cur Orientation \tX : {:.3f},\tY : {:.3f},\tZ : {:.3f}", euler(0), euler(1), euler(2));
    logger_->debug("Error Orientation \tX : {:.3f},\tY : {:.3f},\tZ : {:.3f}", errAngularY, errAngularX, errYaw);

    ros_gz_interfaces::msg::Float32Array propSpeeds;

    double u1 = drone_->calculateThrust(errAltitude);
    double u2 = drone_->calculateAngulaMoment(errAngularX);
    double u3 = drone_->calculateAngulaMoment(errAngularY);
    double u4 = drone_->calculateAngulaMoment(errYaw);

    


    Eigen::Vector4f propForce = drone_->calculatePropSpeeds(u1, u2, u3, u4);

    logger_->info("U : {:.3f}, {:.3f}, {:.3f}, {:.3f} ", u1, u2, u3, u4);
    logger_->info("PropSpeeds : {:.0f}, {:.0f}, {:.0f}, {:.0f} ", propForce(0), propForce(1), propForce(2), propForce(3));

    propSpeeds.data = {propForce(0), propForce(1), propForce(2), propForce(3)};
    publisher_->publish(propSpeeds);
}


int main(int argc, char *argv[])
{
    auto file_logger = spdlog::basic_logger_mt("DronePoseSubscriber_logger", "my_log_file.txt");
    file_logger->set_level(spdlog::level::debug); 
    file_logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] %v");
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DronePoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
