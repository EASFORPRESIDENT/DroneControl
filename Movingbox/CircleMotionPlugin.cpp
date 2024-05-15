#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>

namespace gazebo
{
  class CircleMotionPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      std::cout << "\n\nBox plugin loaded!\n\n" << std::endl;

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CircleMotionPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Get current simulation time
      common::Time currentTime = this->model->GetWorld()->SimTime();



      // Define the radius and speed of the circle motion
      double radius = 8; // Change this as needed
      double angularVelocity = 0.1; // Change this as needed

      // Calculate the x and y coordinates of the circle motion
      double x = radius * cos(angularVelocity * currentTime.Double());
      double y = radius * sin(angularVelocity * currentTime.Double());

      this->model->SetWorldPose(ignition::math::Pose3d(x, y, 0.1, 0, 0, 0));

      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CircleMotionPlugin)
}