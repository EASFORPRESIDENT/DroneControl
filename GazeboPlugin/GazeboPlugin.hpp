#pragma once // Include guard to prevent multiple inclusion
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <random>

#define RED "\033[31m"
#define CLEAR "\033[0m"

namespace gazebo
{
    struct SharedData
    {
        bool reset;
        double posX;
        double posY;
        double posZ;
        double posYaw;
    };
    class SimulationResetPlugin : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
        void OnUpdate();
        void ResetWorld();

    private:
        const char *memoryName;
        int shm_fd;
        SharedData* sharedData;
        physics::WorldPtr world;
        event::ConnectionPtr updateConnection;
        ignition::math::Pose3d RandomPose();
        void SendDronePosition(ignition::math::Pose3d position);
        bool CheckReset();
        ~SimulationResetPlugin();
    };
}
