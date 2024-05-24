#pragma once // Include guard to prevent multiple inclusion
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include <iostream>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <random>

#define RED "\033[31m"
#define CLEAR "\033[0m"

using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

//hello from eas

namespace gazebo
{
    struct SharedData
    {
        bool reset;
        bool play;
        
        double posX;
        double posY;
        double posZ;
        double posYaw;

        double velX;
        double velY;
        double velZ;
        double velYaw;
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
        double stepTimeSec;
        gazebo::common::Time stepStartTime;
        char buffer[sizeof(SharedData)];
        bool aiConnected;
        bool resetActive;

        SharedData* localData;
        SharedData* sharedData;
        physics::WorldPtr world;
        event::ConnectionPtr updateConnection;
        gazebo::common::Time prevTime;
        ignition::math::Pose3d dronePose;
        ignition::math::Pose3d prevDronePose;
        ignition::math::Pose3d RandomPose();

        

        void PauseWorld();
        void SerializeSharedData(const SharedData& data, char* buffer);
        void SetDronePosition(ignition::math::Pose3d position);
        void UpdateVelocity();
        void SendSharedData();
        bool CheckReset();
        ~SimulationResetPlugin();
    };
}