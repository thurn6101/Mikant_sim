#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>

#include "mikant_gazebo_plugins/buoyancy_force_plugin.hh"
#include <string>

namespace buoyancy_force_plugin
{
  void BuoyancyForcePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _model;
    
    this->world = this->model->GetWorld();
    this->linkName = _sdf->GetElement("link_name")->Get<std::string>();
    this->link = model->GetLink(linkName);
    
    this->volume = _sdf->Get<double>("volume");
    this->fluidDensity = _sdf->Get<double>("fluid_density");
    this->fluidLevel = _sdf->Get<double>("fluid_level");
    this->lastSimTime =  this->world->SimTime().Double();
    /*
    if(_sdf->HasElement("link_name"))
    {
      std::string linkName = _sdf->GetElement("link_name")->Get<std::string>();
      physics::LinkPtr link = _parent->GetLink(linkName);
    }
    */
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&BuoyancyForcePlugin::OnUpdate, this));
  }

  // Called by the world update start event
  void BuoyancyForcePlugin::OnUpdate()
  {
    // Apply a small linear velocity to the model.
    //this->model->GetLink(linkName)->SetForce(ignition::math::Vector3d(10000, 0, 0));
    //physics::LinkPtr link = model->GetLink("base_link");
    //link->SetForce(ignition::math::Vector3d(1000, 0, 0));
    
    
  }
    
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(BuoyancyForcePlugin)
}

