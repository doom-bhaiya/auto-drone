#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <drone_msgs/msg/propeller_velocity.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <fstream>
#include <memory>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>

struct ConfigParams {
    double k1=0.000001, k2=0.0000001, m=1.2004, Ip = 0.03;
};

bool loadConfig(const std::string& file_path, ConfigParams& params) {
    std::ifstream file(file_path);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string key;
        double value;
        if (std::getline(ss, key, '=') && ss >> value) {
            if (key == "k1") params.k1 = value;
            else if (key == "k2") params.k2 = value;
            else if (key == "m") params.m = value;
            else if (key == "Ip") params.Ip = value;
        }
    }

    file.close();
    return true;
}

namespace gazebo {

class DroneThrustPlugin : public ModelPlugin {
public:
    DroneThrustPlugin() : ModelPlugin() {}

    // void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {

      std::cout << "[BasicModelPlugin] Loaded successfully for model: "
                << std::endl;

        model_ = model;
        world_ = model->GetWorld();

        // Set link names
        base_link_ = model->GetLink("base_link");
        prop1_ = model->GetLink("prop1");
        prop2_ = model->GetLink("prop2");
        prop3_ = model->GetLink("prop3");
        prop4_ = model->GetLink("prop4");

        if (!base_link_ || !prop1_ || !prop2_ || !prop3_ || !prop4_) {
            gzerr << "[DroneThrustPlugin] One or more links not found!" << std::endl;
            return;
        }

        // Load config file (assume absolute path passed in SDF param)
        std::string config_file;
        if (sdf->HasElement("config_file")) {
            config_file = sdf->Get<std::string>("config_file");
            if (!loadConfig(config_file, params_)) {
                gzerr << "[DroneThrustPlugin] Failed to load config file: " << config_file << std::endl;
            }
        }

        // Initialize ROS 2 node
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        node_ = std::make_shared<rclcpp::Node>("drone_thrust_plugin");

        subscription_ = node_->create_subscription<drone_msgs::msg::PropellerVelocity>(
            "cmd_vel_three", 10,
            std::bind(&DroneThrustPlugin::velocityCallback, this, std::placeholders::_1)
        );

        // Start executor thread
        executor_.add_node(node_);
        executor_thread_ = std::thread([this]() { executor_.spin(); });

        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&DroneThrustPlugin::OnUpdate, this));
    }


    void velocityCallback(std::shared_ptr<const drone_msgs::msg::PropellerVelocity> msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        torque_z_ = params_.k2 * (msg->prop1 * msg->prop1 - msg->prop2 * msg->prop2 +
                                  msg->prop3 * msg->prop3 - msg->prop4 * msg->prop4);
        thrust_[0] = params_.k1 * msg->prop1 * msg->prop1;
        thrust_[1] = params_.k1 * msg->prop2 * msg->prop2;
        thrust_[2] = params_.k1 * msg->prop3 * msg->prop3;
        thrust_[3] = params_.k1 * msg->prop4 * msg->prop4;
    }

    void OnUpdate() {
        std::lock_guard<std::mutex> lock(mutex_);

        base_link_->AddRelativeTorque(ignition::math::Vector3d(0, 0, torque_z_));
        prop1_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust_[0]));
        prop2_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust_[1]));
        prop3_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust_[2]));
        prop4_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust_[3]));
    }

private:
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::LinkPtr base_link_, prop1_, prop2_, prop3_, prop4_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<drone_msgs::msg::PropellerVelocity>::SharedPtr subscription_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread executor_thread_;
    std::mutex mutex_;

    event::ConnectionPtr update_connection_;

    ConfigParams params_;
    double torque_z_ = 0.0;
    double thrust_[4] = {0.0};
};

GZ_REGISTER_MODEL_PLUGIN(DroneThrustPlugin)

} // namespace gazebo


// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <iostream>

// namespace gazebo
// {
//   class BasicModelPlugin : public ModelPlugin
//   {
//   public:
//     void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/) override
//     {
//       std::cout << "[BasicModelPlugin] Loaded successfully for model: "
//                 << model->GetName() << std::endl;
//     }
//   };

//   GZ_REGISTER_MODEL_PLUGIN(BasicModelPlugin)
// }