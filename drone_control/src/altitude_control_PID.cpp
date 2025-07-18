#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_gz_interfaces/msg/float32_array.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include <fstream>
#include <Eigen/Dense>
#include <cmath>

#define ODOM_TOPIC_NAME "DronePose"
#define CMD_TOPIC "/cmd_vel"
#define GRAVITY 9.8


double kf;
double km;
double mass;
double MomentOfInertia;
double propLength;

class DronePoseSubscriber : public rclcpp::Node
{
public:
    DronePoseSubscriber() : Node("drone_pose_subscriber")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            ODOM_TOPIC_NAME, 10,
            std::bind(&DronePoseSubscriber::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<ros_gz_interfaces::msg::Float32Array>(CMD_TOPIC, 10);
    }

private:

    double att_kp;
    double att_ki;
    double att_kd;

    double goal = 0;
    double currentPos = 0;

    double errP = 0;
    double errI = 0;
    double errD = 0;

    double prevErr = 0;
    double IntegratedErr = 0;

    rclcpp::Clock clock;
    rclcpp::Time prevTime = clock.now();
    rclcpp::Time curTime = clock.now();

    double dt;


    void topic_callback(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
    {   
        curTime = clock.now();
        dt = (prevTime - curTime).seconds();


        currentPos = msg->pose.pose.position.z;

        errP = goal - currentPos;
        errD = (prevErr - errP) / dt;
        errI += errP * dt;

        ros_gz_interfaces::msg::Float32Array propSpeeds;


        float weight = mass * GRAVITY;
        float weightPerProp = weight / 4;

        float propForce = sqrt(weightPerProp / kf);
        // propForce = 1500;



        propSpeeds.data = {propForce, propForce, propForce, propForce};
        publisher_->publish(propSpeeds);


        std::cout << "Attitude Error in Z : " << propForce << std::endl;
        
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<ros_gz_interfaces::msg::Float32Array>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{

    std::string pkg_path = ament_index_cpp::get_package_share_directory("drone_sim");
    std::string config_file_path = pkg_path + "/config/drone_params.json";

    std::ifstream file(config_file_path);
    if (!file) {
        std::cerr << "Could not open config.json\n";
        return 1;
    }
    else{
      json config;
      file >> config;

      kf = config["k1"];
      km = config["k2"];
      mass = config["m"];
      MomentOfInertia = config["Ip"];
      propLength = config["length_of_prop"];

    }





    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DronePoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
