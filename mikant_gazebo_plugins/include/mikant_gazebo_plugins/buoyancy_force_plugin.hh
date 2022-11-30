#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <map>
#include <string>
#include <vector>

namespace buoyancy_force_plugin
{
  /// \brief A class for storing buoyancy object properties
  class BuoyancyForcePlugin : public gazebo::ModelPlugin
  {
    public:
      /// \brief Default constructor
      
      void Load(const gazebo::physics::ModelPtr _model, const sdf::ElementPtr _sdf);
      
      /// \brief Associated link name
      std::string linkName;
      
      gazebo::physics::LinkPtr link;
      
      double volume;
      
      double subvolume;
      
      double fluidDensity;
      
      double fluidLevel;
      
      double lastSimTime;
      
    private:
      void OnUpdate();
      
      // Pointer to the model
      gazebo::physics::ModelPtr model;
      
      //Retrieved when the model is loaded.
      gazebo::physics::WorldPtr world;

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr updateConnection;
  };
}

