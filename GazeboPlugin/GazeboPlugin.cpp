#include "GazeboPlugin.hpp"

namespace gazebo
{
    void SimulationResetPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        std::cout << "SimulationResetPlugin loaded successfully!" << std::endl;

        // Shared memory
        this->memoryName = "dronePosAndReset";
        this->shm_fd = shm_open(memoryName, O_CREAT | O_RDWR, 0666);
        ftruncate(shm_fd, sizeof(SharedData));
        this->sharedData = (SharedData*) mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

        // Plugin
        this->world = _world;
        // Listen to the update event which is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SimulationResetPlugin::OnUpdate, this));
    }

    void SimulationResetPlugin::OnUpdate()
    {
        // Logic to reset the world goes here
        static int count = 0;
        if (++count > 10)
        {
            auto model = this->world->ModelByName("iris").get()->WorldPose();
            SendDronePosition(model);
            if (CheckReset())
            {
                ResetWorld();
            }
            count = 0;
        }
    }

    void SimulationResetPlugin::ResetWorld()
    {
        ignition::math::Pose3d newPose = RandomPose();
        auto model = this->world->ModelByName("iris");
        if (model)
        {
            model->SetWorldPose(newPose);
        }
        else
        {
            std::cout << "Failed to find model 'iris'." << std::endl;
        }
    }


    ignition::math::Pose3d SimulationResetPlugin::RandomPose()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-10, 10);

        double x = dis(gen);
        double y = dis(gen);
        double z = 10; // Altitude

        return ignition::math::Pose3d(x, y, z, 0, 0, 0);
    }

    void SimulationResetPlugin::SendDronePosition(ignition::math::Pose3d position)
    {
        sharedData->posX = position.X();
        sharedData->posY = position.Y();
        sharedData->posZ = position.Z();
        sharedData->posYaw = position.Yaw();
    }
    bool SimulationResetPlugin::CheckReset()
    {
        switch (sharedData->reset)
        {
        case true:
            sharedData->reset = false;
            return true;
        default:
            return false;
        }
    }

    SimulationResetPlugin::~SimulationResetPlugin()
    {
        munmap(sharedData, sizeof(SharedData));
        close(shm_fd);
        shm_unlink(memoryName);
    }
}

GZ_REGISTER_WORLD_PLUGIN(gazebo::SimulationResetPlugin)
