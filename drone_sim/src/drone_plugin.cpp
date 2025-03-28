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
    physics::LinkPtr base_link_;

    physics::LinkPtr prop_link1_;
    physics::LinkPtr prop_link2_;
    physics::LinkPtr prop_link3_;
    physics::LinkPtr prop_link4_;


    std::string robot_namespace_;


    geometry_msgs::Wrench wrench_msg_;
    geometry_msgs::Wrench wrench_msg_1;
    geometry_msgs::Wrench wrench_msg_2;
    geometry_msgs::Wrench wrench_msg_3;
    geometry_msgs::Wrench wrench_msg_4;

    ros::NodeHandle* rosnode_;

    ros::Subscriber sub_;
    ros::CallbackQueue queue_;

    boost::mutex lock_;
    boost::thread callback_queue_thread_;


    event::ConnectionPtr updateConnection;
    double propeller_speed;  
    double thrust_coefficient;

    std::string link_name; 
    std::string link_name_prop1;
    std::string link_name_prop2;
    std::string link_name_prop3;
    std::string link_name_prop4;

    std::string topic_name_;

    ConfigParams params;

  public:

    void UpdateObjectForce(const boost::shared_ptr<const drone_msgs::PropellerVelocity>& _msg)
    {

      this->wrench_msg_.force.x = 0;
      this->wrench_msg_.force.y = 0;
      this->wrench_msg_.force.z = 0;
      this->wrench_msg_.torque.x = 0;
      this->wrench_msg_.torque.y = 0;
      this->wrench_msg_.torque.z = params.k2 * ((_msg->prop1 * _msg->prop1) - (_msg->prop2 * _msg->prop2) + (_msg->prop3 * _msg->prop3) - (_msg->prop4 * _msg->prop4));

      this->wrench_msg_1.force.x = 0;
      this->wrench_msg_1.force.y = 0;
      this->wrench_msg_1.force.z = params.k1 * (_msg->prop1 * _msg->prop1);
      this->wrench_msg_1.torque.x = 0;
      this->wrench_msg_1.torque.y = 0;
      this->wrench_msg_1.torque.z = 0;

      this->wrench_msg_2.force.x = 0;
      this->wrench_msg_2.force.y = 0;
      this->wrench_msg_2.force.z = params.k1 * (_msg->prop2 * _msg->prop2);
      this->wrench_msg_2.torque.x = 0;
      this->wrench_msg_2.torque.y = 0;
      this->wrench_msg_2.torque.z = 0;

      this->wrench_msg_3.force.x = 0;
      this->wrench_msg_3.force.y = 0;
      this->wrench_msg_3.force.z = params.k1 * (_msg->prop3 * _msg->prop3);
      this->wrench_msg_3.torque.x = 0;
      this->wrench_msg_3.torque.y = 0;
      this->wrench_msg_3.torque.z = 0;

      this->wrench_msg_4.force.x = 0;
      this->wrench_msg_4.force.y = 0;
      this->wrench_msg_4.force.z = params.k1 * (_msg->prop4 * _msg->prop4);
      this->wrench_msg_4.torque.x = 0;
      this->wrench_msg_4.torque.y = 0;
      this->wrench_msg_4.torque.z = 0;
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {


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
      this->base_link_ = _model->GetLink(this->link_name);

      link_name_prop1 = "prop1";
      this->prop_link1_ = _model->GetLink(this->link_name_prop1);

      link_name_prop2 = "prop2";
      this->prop_link2_ = _model->GetLink(this->link_name_prop2);

      link_name_prop3 = "prop3";
      this->prop_link3_ = _model->GetLink(this->link_name_prop3);

      link_name_prop4 = "prop4";
      this->prop_link4_ = _model->GetLink(this->link_name_prop4);

      if (!this->base_link_)
      {
        ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: base does not exist\n");
        return;
      }

      if (!this->prop_link1_)
      {
        ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: props does not exist\n");
        return;
      }





      topic_name_ = "cmd_vel_three";

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

    ignition::math::Vector3d torque(this->wrench_msg_.torque.x,this->wrench_msg_.torque.y,this->wrench_msg_.torque.z);
    this->base_link_->AddRelativeTorque(torque);

{
    ignition::math::Vector3d force(this->wrench_msg_1.force.x,this->wrench_msg_1.force.y,this->wrench_msg_1.force.z);
    this->prop_link1_->AddRelativeForce(force);
}
{
    ignition::math::Vector3d force(this->wrench_msg_2.force.x,this->wrench_msg_2.force.y,this->wrench_msg_2.force.z);
    this->prop_link2_->AddRelativeForce(force);
}
{
    ignition::math::Vector3d force(this->wrench_msg_3.force.x,this->wrench_msg_3.force.y,this->wrench_msg_3.force.z);
    this->prop_link3_->AddRelativeForce(force);
}
{
    ignition::math::Vector3d force(this->wrench_msg_4.force.x,this->wrench_msg_4.force.y,this->wrench_msg_4.force.z);
    this->prop_link4_->AddRelativeForce(force);
}


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