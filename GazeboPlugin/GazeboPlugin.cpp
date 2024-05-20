#include "GazeboPlugin.hpp"

namespace gazebo
{
    int go = 0;
    void SimulationResetPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        std::cout << "SimulationResetPlugin loaded successfully!" << std::endl;

        // Shared memory
        this->memoryName = "dronePoseAndReset";
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

        localData = new SharedData();
        prevTime = 0;
        sharedData->reset = false;

        // Plugin
        this->world = _world;
        
        // Listen to the update event which is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SimulationResetPlugin::OnUpdate, this));
    }

    void SimulationResetPlugin::OnUpdate()
    {
        static int count = 0;
        if (++count > 150)
        {
            if (PauseWorld())
                return;

            if (CheckReset())
            {
                ResetWorld();
            }

            dronePose = this->world->ModelByName("iris").get()->WorldPose();

            SetDronePosition(dronePose);
            UpdateVelocity();
            SendSharedData();

            prevDronePose = dronePose;
            prevTime = this->world.get()->SimTime();
            count = 0;
        }
    }

    bool SimulationResetPlugin::PauseWorld()
    {
        auto thisWorld = this->world.get();
        if (!(sharedData->play))
        {
            thisWorld->SetPaused(true);
            return true;
        }
        thisWorld->SetPaused(false);
        return false;
    }

    void SimulationResetPlugin::ResetWorld()
    {
        ignition::math::Pose3d newPose = RandomPose();
        auto model = this->world->ModelByName("iris");
        if (model)
        {
            model->SetWorldPose(newPose);
            //this->world.get()->SetPaused(true);
            //std::this_thread::sleep_for(std::chrono::seconds(1));
            //this->world.get()->SetPaused(false);
        }
        else
        {
            std::cout << "Failed to find model 'iris'." << std::endl;
        }
    }

    bool SimulationResetPlugin::CheckReset()
    {
        switch (sharedData->reset)
        {
        case true:
            localData->reset = false;
            return true;
        default:
            return false;
        }
    }

    ignition::math::Pose3d SimulationResetPlugin::RandomPose()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.5, 1.5);

        double x = dis(gen);
        double y = dis(gen);
        double z = 10; // Altitude

        return ignition::math::Pose3d(x, y, z, 0, 0, 0);
    }

    void SimulationResetPlugin::SetDronePosition(ignition::math::Pose3d position)
    {
        localData->posX = position.X();
        localData->posY = position.Y();
        localData->posZ = position.Z();
        localData->posYaw = position.Yaw();
    }

    void SimulationResetPlugin::UpdateVelocity()
    {
        auto currentTime = this->world.get()->SimTime();
        auto deltaTime = (currentTime - prevTime).Double();
        localData->velX = (localData->posX - prevDronePose.X()) / deltaTime;
        localData->velY = (localData->posY - prevDronePose.Y()) / deltaTime;
        localData->velZ = (localData->posZ - prevDronePose.Z()) / deltaTime;
        localData->velYaw = (localData->posX - prevDronePose.Yaw()) / deltaTime;
    }

    void SimulationResetPlugin::SendSharedData()
    {
        // Serialize SharedData struct into a byte array
        SerializeSharedData(*localData, buffer);

        // Write serialized data to shared memory
        std::memcpy(sharedData, buffer, sizeof(SharedData));
    }

    void SimulationResetPlugin::SerializeSharedData(const SharedData& data, char* buffer)
    {
        std::memcpy(buffer, &data, sizeof(SharedData));
    }

    SimulationResetPlugin::~SimulationResetPlugin()
    {
        munmap(sharedData, sizeof(SharedData));
        close(shm_fd);
        shm_unlink(memoryName);
    }
}

GZ_REGISTER_WORLD_PLUGIN(gazebo::SimulationResetPlugin)