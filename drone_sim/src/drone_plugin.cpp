#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <thread>
#include "ros/ros.h"
#include <ros/package.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include "drone_msgs/PropellerVelocity.h"

#include <geometry_msgs/Wrench.h>

struct ConfigParams {
    double k1 = 0.0, k2 = 0.0, m = 0.0, Ip = 0.0;
};

bool loadConfig(const std::string &package_name, const std::string &config_file, ConfigParams &params)
{
    // Get absolute path of the package
    std::string package_path = ros::package::getPath(package_name);
    if (package_path.empty())
    {
        ROS_ERROR_STREAM("Error: Could not find package: " << package_name);
        return false;
    }

    // Construct absolute path to config.txt
    std::string file_path = package_path + "/config/" + config_file;
    
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR_STREAM("Error: Unable to open config file: " << file_path);
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string key;
        double value;

        if (std::getline(ss, key, '=') && ss >> value)
        {
            if (key == "k1") params.k1 = value;
            else if (key == "k2") params.k2 = value;
            else if (key == "m") params.m = value;
            else if (key == "Ip") params.Ip = value;
        }
    }

    file.close();
    return true;
}
namespace gazebo
{
  class DroneThrustPlugin : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    physics::LinkPtr base_link;

    physics::WorldPtr world_;
    physics::LinkPtr link_;

    std::string robot_namespace_;


    geometry_msgs::Wrench wrench_msg_;

    ros::NodeHandle* rosnode_;

    ros::Subscriber sub_;
    ros::CallbackQueue queue_;

    boost::mutex lock_;
    boost::thread callback_queue_thread_;


    event::ConnectionPtr updateConnection;
    double propeller_speed;  
    double thrust_coefficient;

    std::string link_name; 
    std::string topic_name_;

  public:

    void UpdateObjectForce(const boost::shared_ptr<const drone_msgs::PropellerVelocity>& _msg)
    {
      ROS_INFO_STREAM("Received propeller velocities: "
                      << _msg->prop1 << ", "
                      << _msg->prop2 << ", "
                      << _msg->prop3 << ", "
                      << _msg->prop4);
      // this->wrench_msg_.force.x = _msg->force.x;
      // this->wrench_msg_.force.y = _msg->force.y;
      // this->wrench_msg_.force.z = _msg->force.z;
      // this->wrench_msg_.torque.x = _msg->torque.x;
      // this->wrench_msg_.torque.y = _msg->torque.y;
      // this->wrench_msg_.torque.z = _msg->torque.z;
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {

  ConfigParams params;
  std::string package_name = "drone_sim";
    if (loadConfig(package_name, "drone_params.txt", params))
    {
        ROS_INFO_STREAM("Loaded Configurations:");
        ROS_INFO_STREAM("k1: " << params.k1);
        ROS_INFO_STREAM("k2: " << params.k2);
        ROS_INFO_STREAM("m: " << params.m);
        ROS_INFO_STREAM("Ip: " << params.Ip);
    }

      ROS_INFO("Hello World!");

      this->world_ = _model->GetWorld();
      this->robot_namespace_ = "";

      link_name = "base_link";
      topic_name_ = "cmd_vel_three";
      this->link_ = _model->GetLink(this->link_name);

      if (!this->link_)
      {
        ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link_name.c_str());
        return;
      }

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM_NAMED("force", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

      ros::SubscribeOptions so = ros::SubscribeOptions::create<drone_msgs::PropellerVelocity>(
          this->topic_name_,1,
          boost::bind( &DroneThrustPlugin::UpdateObjectForce,this, boost::placeholders::_1),
          ros::VoidPtr(), &this->queue_);
        this->sub_ = this->rosnode_->subscribe(so);

      this->callback_queue_thread_ = boost::thread( boost::bind( &DroneThrustPlugin::QueueThread,this ) );

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DroneThrustPlugin::UpdateChild, this));


    }

  void UpdateChild()
  {
  #ifdef ENABLE_PROFILER
    IGN_PROFILE("GazeboRosForce::OnNewFrame");
    IGN_PROFILE_BEGIN("fill ROS message");
  #endif
    this->lock_.lock();
    ignition::math::Vector3d force(this->wrench_msg_.force.x,this->wrench_msg_.force.y,this->wrench_msg_.force.z);
    ignition::math::Vector3d torque(this->wrench_msg_.torque.x,this->wrench_msg_.torque.y,this->wrench_msg_.torque.z);
    this->link_->AddForce(force);
    this->link_->AddTorque(torque);
    this->lock_.unlock();
  #ifdef ENABLE_PROFILER
    IGN_PROFILE_END();
  #endif
  }

  void QueueThread()
  {
    static const double timeout = 0.01;

    while (this->rosnode_->ok())
    {
      this->queue_.callAvailable(ros::WallDuration(timeout));
    }
  }



  };



  GZ_REGISTER_MODEL_PLUGIN(DroneThrustPlugin)
}