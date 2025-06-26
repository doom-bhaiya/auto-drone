#ifndef DRONE_THRUST_PLUGIN_HH_
#define DRONE_THRUST_PLUGIN_HH_

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <drone_msgs/msg/propeller_velocity.hpp>

// Helpers
#include <mutex>
#include <string>

namespace gazebo {

struct ConfigParams {
  double k1 = 0.0, k2 = 0.0, m = 0.0, Ip = 0.0;
};

class DroneThrustPlugin : public ModelPlugin {
public:
  DroneThrustPlugin();
  virtual ~DroneThrustPlugin();

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void velocityCallback(const drone_msgs::msg::PropellerVelocity::SharedPtr msg);
  void OnUpdate();

  // Gazebo elements
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr base_link_, prop1_, prop2_, prop3_, prop4_;
  event::ConnectionPtr update_connection_;

  // ROS 2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<drone_msgs::msg::PropellerVelocity>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;

  // Synchronization
  std::mutex mutex_;

  // Configuration & forces/torques
  ConfigParams params_;
  double torque_z_{0.0};
  double thrust_[4]{0.0};

  // Utility
  bool loadConfig(const std::string& file_path, ConfigParams& params);
};

}  // namespace gazebo

#endif  // DRONE_THRUST_PLUGIN_HH_
