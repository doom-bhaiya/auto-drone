#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include "drone_sim/DronePlugin.hh"

#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>

using namespace gz;
using namespace sim;
using namespace systems;
 
class ignition::gazebo::systems::DronePluginPrivate
{
public:

  Entity base;

  Entity prop1;
  Entity prop2;
  Entity prop3;
  Entity prop4;

  Link link_base;
  Link link_prop1;
  Link link_prop2;
  Link link_prop3;
  Link link_prop4;

  Entity prop1_joint;
  Entity prop2_joint;
  Entity prop3_joint;  
  Entity prop4_joint;

  double prop1_speed = 0;
  double prop2_speed = 0;
  double prop3_speed = 0;
  double prop4_speed = 0;

  math::Vector3d force_base;
  math::Vector3d force1;
  math::Vector3d force2;
  math::Vector3d force3;
  math::Vector3d force4;

  math::Vector3d torque_base;
  math::Vector3d torque;

  double kf = 1e-6;
  double km = 1e-7;

  bool enabled = false;

  transport::Node node;

  Model model{kNullEntity};

  void OnEnable(const ignition::msgs::Boolean &_msg);
  void OnCommand(const ignition::msgs::Float_V &_msg);

};

DronePlugin::DronePlugin()
  : dataPtr(std::make_unique<DronePluginPrivate>())
{
}

void DronePlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/){

    ignmsg << "Loading DronePlugin";

    this->dataPtr->model = Model(_entity);


    std::string base_link_name = _sdf->Get<std::string>("base_link_name");
    std::string prop1_link_name = _sdf->Get<std::string>("prop_link_name1");
    std::string prop2_link_name = _sdf->Get<std::string>("prop_link_name2");
    std::string prop3_link_name = _sdf->Get<std::string>("prop_link_name3");
    std::string prop4_link_name = _sdf->Get<std::string>("prop_link_name4");

    this->dataPtr->base = this->dataPtr->model.LinkByName(_ecm, base_link_name);
    this->dataPtr->prop1 = this->dataPtr->model.LinkByName(_ecm, prop1_link_name);
    this->dataPtr->prop2 = this->dataPtr->model.LinkByName(_ecm, prop2_link_name);
    this->dataPtr->prop3 = this->dataPtr->model.LinkByName(_ecm, prop3_link_name);
    this->dataPtr->prop4 = this->dataPtr->model.LinkByName(_ecm, prop4_link_name);

    std::string prop1_joint_name = _sdf->Get<std::string>("prop1_joint");
    std::string prop2_joint_name = _sdf->Get<std::string>("prop2_joint");
    std::string prop3_joint_name = _sdf->Get<std::string>("prop3_joint");
    std::string prop4_joint_name = _sdf->Get<std::string>("prop4_joint");

    this->dataPtr->prop1_joint = this->dataPtr->model.JointByName(_ecm, prop1_joint_name);
    this->dataPtr->prop2_joint = this->dataPtr->model.JointByName(_ecm, prop2_joint_name);
    this->dataPtr->prop3_joint = this->dataPtr->model.JointByName(_ecm, prop3_joint_name);
    this->dataPtr->prop4_joint = this->dataPtr->model.JointByName(_ecm, prop4_joint_name);

    if (!this->dataPtr->base || 
        !this->dataPtr->prop1 || !this->dataPtr->prop1_joint ||
        !this->dataPtr->prop3 || !this->dataPtr->prop2_joint ||
        !this->dataPtr->prop3 || !this->dataPtr->prop3_joint ||
        !this->dataPtr->prop4 || !this->dataPtr->prop4_joint)
    {
      ignerr << "DronePlugin: Unable to find all required links or joints." << std::endl;
      return;
    }

    _ecm.CreateComponent(this->dataPtr->base, components::WorldPose());

    _ecm.CreateComponent(this->dataPtr->prop1_joint, components::JointVelocityCmd({this->dataPtr->prop1_speed}));
    _ecm.CreateComponent(this->dataPtr->prop2_joint, components::JointVelocityCmd({this->dataPtr->prop2_speed}));
    _ecm.CreateComponent(this->dataPtr->prop3_joint, components::JointVelocityCmd({this->dataPtr->prop3_speed}));
    _ecm.CreateComponent(this->dataPtr->prop4_joint, components::JointVelocityCmd({this->dataPtr->prop4_speed}));

    this->dataPtr->link_base = gz::sim::Link(this->dataPtr->base);
    this->dataPtr->link_prop1 = gz::sim::Link(this->dataPtr->prop1);
    this->dataPtr->link_prop2 = gz::sim::Link(this->dataPtr->prop2);
    this->dataPtr->link_prop3 = gz::sim::Link(this->dataPtr->prop3);
    this->dataPtr->link_prop4 = gz::sim::Link(this->dataPtr->prop4);

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


}

void DronePlugin::PreUpdate(const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  components::JointVelocityCmd({this->dataPtr->prop1_speed});
  components::JointVelocityCmd({-1 * this->dataPtr->prop2_speed});
  components::JointVelocityCmd({this->dataPtr->prop3_speed});
  components::JointVelocityCmd({-1 * this->dataPtr->prop4_speed});  


  auto baseLinkPose = _ecm.Component<gz::sim::components::WorldPose>(this->dataPtr->base);
  gz::math::Pose3d worldPose = baseLinkPose->Data();
  gz::math::Quaterniond rotWorldToBody = worldPose.Rot().Inverse();


  this->dataPtr->link_base.AddWorldWrench(_ecm, rotWorldToBody * this->dataPtr->force_base, rotWorldToBody * this->dataPtr->torque_base);
  this->dataPtr->link_prop1.AddWorldWrench(_ecm, rotWorldToBody * this->dataPtr->force1, rotWorldToBody * this->dataPtr->torque);
  this->dataPtr->link_prop2.AddWorldWrench(_ecm, rotWorldToBody * this->dataPtr->force2, rotWorldToBody * this->dataPtr->torque);
  this->dataPtr->link_prop3.AddWorldWrench(_ecm, rotWorldToBody * this->dataPtr->force3, rotWorldToBody * this->dataPtr->torque);
  this->dataPtr->link_prop4.AddWorldWrench(_ecm, rotWorldToBody * this->dataPtr->force4, rotWorldToBody * this->dataPtr->torque);


}

void DronePlugin::PostUpdate(const UpdateInfo &_info,
  const EntityComponentManager &_ecm)
{

}

void DronePluginPrivate::OnEnable(const ignition::msgs::Boolean &_msg)
{
  bool value = _msg.data();
  enabled = value;

  if (!enabled) {
    ignmsg << "Drone props enabled." << std::endl;
  } else {

    ignmsg << "Drone props disabled." << std::endl;
    prop1_speed = 0;
    prop2_speed = 0;
    prop3_speed = 0;
    prop4_speed = 0;

    force_base = {0, 0, 0};
    force1 = {0, 0, 0};
    force2 = {0, 0, 0};
    force3 = {0, 0, 0};
    force4 = {0, 0, 0};

    torque_base = {0, 0, 0};
  }
}

void DronePluginPrivate::OnCommand(const ignition::msgs::Float_V &_msg)
{ 

  if (enabled){
    prop1_speed = _msg.data(0);
    prop2_speed = _msg.data(1);
    prop3_speed = _msg.data(2);
    prop4_speed = _msg.data(3);

    force_base = {0, 0, 0};
    force1 = {0, 0, prop1_speed * prop1_speed * kf};
    force2 = {0, 0, prop2_speed * prop2_speed * kf};
    force3 = {0, 0, prop3_speed * prop3_speed * kf};
    force4 = {0, 0, prop4_speed * prop4_speed * kf};

    torque_base = {0, 0, (prop1_speed * prop1_speed -
                          prop2_speed * prop2_speed +
                          prop3_speed * prop3_speed -
                          prop4_speed * prop4_speed) * km}; 
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