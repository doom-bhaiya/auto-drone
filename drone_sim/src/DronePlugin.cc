#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include "drone_sim/DronePlugin.hh"

#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include <ignition/msgs/odometry.pb.h>
#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

// #include <iostream>
#include <cmath>
#include "spdlog/spdlog.h"
#include <spdlog/sinks/basic_file_sink.h>

using namespace gz;
using namespace sim;
using namespace systems;
 
class ignition::gazebo::systems::DronePluginPrivate
{
public:

  Entity base;
  Link link_base;

  Entity props[4];
  Link link_props[4];
  Entity props_joint[4];

  double props_speed[4] = {0, 0, 0, 0};
  double props_position[4] = {0, 0, 0, 0};
  double max_velocity;
  double max_acceleration;

  math::Vector3d force_base;

  math::Vector3d force[4];

  math::Vector3d torque_base;
  math::Vector3d torque[4];

  double kf;
  double km;

  bool enabled = false;

  transport::Node node;
  transport::Node::Publisher odomPub;

  Model model{kNullEntity};

  void OnEnable(const ignition::msgs::Boolean &_msg);
  void OnCommand(const ignition::msgs::Float_V &_msg);

  std::default_random_engine randGen;
  std::normal_distribution<double> noiseDist;

};

DronePlugin::DronePlugin()
  : dataPtr(std::make_unique<DronePluginPrivate>())
{
}

void DronePlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/){

      auto logger = spdlog::basic_logger_mt("drone_plugin_logger", "drone_plugin.log");
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v"); // optional, pretty
        logger->set_level(spdlog::level::debug);

    ignmsg << "Loading DronePlugin";

    this->dataPtr->model = Model(_entity);

    std::string base_link_name = _sdf->Get<std::string>("base_link_name");
    
    std::string prop_link_names[4] = {
        _sdf->Get<std::string>("prop_link_name1"),
        _sdf->Get<std::string>("prop_link_name2"),
        _sdf->Get<std::string>("prop_link_name3"),
        _sdf->Get<std::string>("prop_link_name4")
    };

    std::string prop_joint_names[4] = {
        _sdf->Get<std::string>("prop1_joint"),
        _sdf->Get<std::string>("prop2_joint"),
        _sdf->Get<std::string>("prop3_joint"),
        _sdf->Get<std::string>("prop4_joint")
    };

    this->dataPtr->max_velocity = _sdf->Get<double>("max_velocity");
    this->dataPtr->max_acceleration = _sdf->Get<double>("max_acceleration");

    double randomNoiseStdDev = _sdf->Get<double>("noise_std_dev");
    this->dataPtr->noiseDist = std::normal_distribution<double>(0.0, randomNoiseStdDev);
    
    this->dataPtr->base = this->dataPtr->model.LinkByName(_ecm, base_link_name);
    this->dataPtr->link_base = gz::sim::Link(this->dataPtr->base);


    if (!this->dataPtr->base) {
      ignerr << "DronePlugin: Unable to find the base link." << std::endl;
      return;
    }

    _ecm.CreateComponent(this->dataPtr->base, components::WorldPose());
    _ecm.CreateComponent(this->dataPtr->base,
                     components::LinearVelocity(math::Vector3d::Zero));
    _ecm.CreateComponent(this->dataPtr->base,
                     components::AngularVelocity(math::Vector3d::Zero));

    for (int i = 0; i < 4; i++) {
      this->dataPtr->props[i] = this->dataPtr->model.LinkByName(_ecm, prop_link_names[i]);
      this->dataPtr->props_joint[i] = this->dataPtr->model.JointByName(_ecm, prop_joint_names[i]);

      if (!this->dataPtr->props[i] || !this->dataPtr->props_joint[i]) {
        ignerr << "DronePlugin: Unable to find all required links or joints." << std::endl;
        return;
      }
      
      _ecm.CreateComponent(this->dataPtr->props_joint[i], components::JointVelocityCmd({this->dataPtr->props_speed[i]}));
      this->dataPtr->link_props[i] = gz::sim::Link(this->dataPtr->props[i]);


    }


    std::string config_file_path = _sdf->Get<std::string>("config_file_path");
    std::ifstream file(config_file_path);
    if (!file) {
        std::cerr << "Could not open config.json\n";
    }
    else{
      json config;
      file >> config;

      this->dataPtr->kf = config["k1"];
      this->dataPtr->km = config["k2"];
    }


                     



    {
      std::vector<std::string> enableTopics;
      enableTopics.push_back(_sdf->Get<std::string>("enable_topic"));
      auto enableTopic = validTopic(enableTopics);
      if (enableTopic.empty()){
        ignerr << "Empty Enable Topic\n";
      }
      else{
        ignmsg << "Subscribed to Enable Topic: " << enableTopic << std::endl;
      }

      this->dataPtr->node.Subscribe(enableTopic, &DronePluginPrivate::OnEnable,
          this->dataPtr.get());
    }

    {
      std::vector<std::string> commandTopics;
      commandTopics.push_back(_sdf->Get<std::string>("cmd_vel"));
      auto commandTopic = validTopic(commandTopics);
      if (commandTopic.empty()){
        ignerr << "Empty Command Topic\n";
      }
      else{
        ignmsg << "Subscribed to Command Topic: " << commandTopic << std::endl;
      }

      this->dataPtr->node.Subscribe(commandTopic, &DronePluginPrivate::OnCommand,
          this->dataPtr.get());
    }

    {
      std::vector<std::string> odomTopics;
      odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
      auto odomTopic = validTopic(odomTopics);
      this->dataPtr->odomPub = this->dataPtr->node.Advertise<ignition::msgs::Odometry>(
          odomTopic);
    }


}

void DronePlugin::PreUpdate(const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  // components::JointVelocityCmd({this->dataPtr->props_speed[0]});
  // components::JointVelocityCmd({-1 * this->dataPtr->props_speed[1]});
  // components::JointVelocityCmd({this->dataPtr->props_speed[2]});
  // components::JointVelocityCmd({-1 * this->dataPtr->props_speed[3]});  


  auto baseLinkPose = _ecm.Component<gz::sim::components::WorldPose>(this->dataPtr->base);

  gz::math::Pose3d worldPose = baseLinkPose->Data();
  gz::math::Quaterniond rotWorldToBody = worldPose.Rot().Inverse();

  if (!_info.paused){
    this->dataPtr->link_base.AddWorldWrench(
      _ecm,
      rotWorldToBody.RotateVector(this->dataPtr->force_base),
      rotWorldToBody.RotateVector(this->dataPtr->torque_base)
    );

  
    for (int i = 0; i < 4; i++){

      auto *velCmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(this->dataPtr->props_joint[i]);
      if (i % 2)
        velCmd->Data()[0] = this->dataPtr->props_speed[i];
      else
        velCmd->Data()[0] = -1 * this->dataPtr->props_speed[i];

      this->dataPtr->link_props[i].AddWorldWrench(
        _ecm,
        rotWorldToBody.RotateVector(this->dataPtr->force[i]),
        rotWorldToBody.RotateVector(this->dataPtr->torque[i])
      );
    }
  }

}

void DronePlugin::PostUpdate(const UpdateInfo &_info,
  const EntityComponentManager &_ecm)
{

    auto poseComp = _ecm.Component<gz::sim::components::WorldPose>(this->dataPtr->base);
    auto linVelComp = _ecm.Component<gz::sim::components::LinearVelocity>(this->dataPtr->base);
    auto angVelComp = _ecm.Component<gz::sim::components::AngularVelocity>(this->dataPtr->base);

    if (!linVelComp)
    {
      ignerr << "One or more components missing (pose/velocities).\n";
      return;
    }

    ignition::msgs::Odometry odomMsg;
    // odomMsg.set_name("Drone");
    msgs::Set(odomMsg.mutable_pose(), poseComp->Data());

    ignition::msgs::Header* header = odomMsg.mutable_header();

    auto simTimeNs = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
    int32_t simTimeSec = static_cast<int32_t>(simTimeNs / 1'000'000'000);
    int32_t simTimeNsec = static_cast<int32_t>(simTimeNs % 1'000'000'000);

    header->mutable_stamp()->set_sec(simTimeSec);
    header->mutable_stamp()->set_nsec(simTimeNsec);

    auto* frameEntry = header->add_data();
    frameEntry->set_key("frame_id");
    frameEntry->add_value("odom");

    auto* childFrameEntry = header->add_data();
    childFrameEntry->set_key("child_frame_id");
    childFrameEntry->add_value("base_link");

    auto twist = odomMsg.mutable_twist();
    msgs::Set(twist->mutable_linear(), linVelComp->Data());
    msgs::Set(twist->mutable_angular(), angVelComp->Data());


    this->dataPtr->odomPub.Publish(odomMsg);

}

void DronePluginPrivate::OnEnable(const ignition::msgs::Boolean &_msg)
{
  bool value = _msg.data();
  enabled = value;

  if (!enabled) {
    ignmsg << "Drone props enabled." << std::endl;
  } 
  else {

    ignmsg << "Drone props disabled." << std::endl;

    force_base = {0, 0, 0};
    torque_base = {0, 0, 0};

    for (int i = 0; i < 4; i++){
      props_speed[i] = 0;
      force[i] = {0, 0, 0};
    }
  }
}

void DronePluginPrivate::OnCommand(const ignition::msgs::Float_V &_msg)
{ 
  if (_msg.data_size() < 4){
    ignerr << "Less values are provided, Ignoring command" << std::endl;
    return;
  }
  if (_msg.data_size() > 4){
    ignerr << "More values are provided, Ignoring command" << std::endl;
    return;
  }

  if (enabled){

    for (int i = 0; i < 4; i++){
      if (std::isnan(_msg.data(i))){
        ignerr << "Nan values are passed, ignoring the command " << std::endl;
        return;
      }
    }




    for (int i = 0; i < 4; i++) {

      if (props_speed[i] < 0) {

        ignerr << "Negative speed is provided to one or more Props, Using 0 speeds for them" << std::endl;
        props_speed[i] = 0;
      }
      

      double noisy_speed = _msg.data(i) + noiseDist(randGen);
      props_speed[i] = std::max(0.0, noisy_speed);

      if (props_speed[i] > max_velocity)  props_speed[i] = max_velocity;
      force[i] = {0, 0, props_speed[i] * props_speed[i] * kf};
    }
    force_base = {0, 0, 0};

    torque_base = {0, 0, (props_speed[0] * props_speed[0] -
                          props_speed[1] * props_speed[1] +
                          props_speed[2] * props_speed[2] -
                          props_speed[3] * props_speed[3]) * km}; 
  }
  else {
    igndbg << "Drone props are disabled. Ignoring command." << std::endl;
  }
}

IGNITION_ADD_PLUGIN(DronePlugin,
                    gz::sim::System,
                    DronePlugin::ISystemConfigure,
                    DronePlugin::ISystemPreUpdate,
                    DronePlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DronePlugin, "gz::sim::systems::DronePlugin")