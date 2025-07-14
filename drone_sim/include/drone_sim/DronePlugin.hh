#ifndef GZ_GAZEBO_SYSTEMS_DRONEPLUGIN_HH_
#define GZ_GAZEBO_SYSTEMS_DRONEPLUGIN_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace ignition
{
namespace gazebo
{

inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class DronePluginPrivate;

  /// \brief This system applies a force to the first axis of a specified joint.
  class DronePlugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate, 
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: DronePlugin();

    /// \brief Destructor
    public: ~DronePlugin() override = default;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<DronePluginPrivate> dataPtr;
  };
  }
}
}
}

#endif