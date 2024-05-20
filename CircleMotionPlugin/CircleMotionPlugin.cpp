#pragma once // Include guard to prevent multiple inclusion
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>


#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include <iostream>

#define RED "\033[31m"
#define CLEAR "\033[0m"


namespace gazebo
/**
 * @brief This file contains the implementation of the CircleMotionPlugin class.
 * 
 * The CircleMotionPlugin class is a Gazebo model plugin that enables a model to move in a circular motion.
 * It listens to the update event and calculates the x and y coordinates of the model's position based on the current simulation time.
 * The calculated position is then set as the world pose of the model.
 * The plugin also uses shared memory to store and share the position data with other processes.
 */
{
  class CircleMotionPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      std::cout << "\n\nBox plugin loaded!\n\n" << std::endl;

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CircleMotionPlugin::OnUpdate, this));


      // Shared memory
      this->memoryName = "boxpose";
      this->shm_fd = shm_open(this->memoryName, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR | S_IXUSR);
      if (this->shm_fd == -1) {
          std::cerr << RED << "shm_open" << CLEAR << std::endl;
      }

      if (ftruncate(this->shm_fd, sizeof(SharedData)) == -1) {
          std::cerr << RED << "ftruncate" << CLEAR << std::endl;
      }

      this->sharedData = (SharedData*) mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, this->shm_fd, 0);
      if (this->sharedData == MAP_FAILED) {
          std::cerr << RED << "mmap" << CLEAR << std::endl;
      }

        //localData = new SharedData();

        sharedData->reset = false;
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
      
      sharedData->posX = x;
      sharedData->posY = y;

      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: 
      physics::ModelPtr model;

      struct SharedData
      {
        bool reset;
        double posX;
        double posY;
        double posZ;
        double posYaw;
      };


      SharedData* localData;
      SharedData* sharedData;
      const char *memoryName;
      int shm_fd;
      char buffer[sizeof(SharedData)];



    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CircleMotionPlugin)
}