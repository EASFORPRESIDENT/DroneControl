#include "GazeboPlugin.hpp"

namespace gazebo
{
    void SimulationResetPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        std::cout << "SimulationResetPlugin loaded successfully!" << std::endl;
        this->world = _world;
        // Listen to the update event which is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SimulationResetPlugin::OnUpdate, this));
    }

    void SimulationResetPlugin::OnUpdate()
    {
        // Logic to reset the world goes here
        static int count = 0;
        if (++count > 1000)
        {
            auto model = this->world->ModelByName("iris").get()->WorldPose();
            std::cout << "PositionX: " << model.X() << " \t PositionY: " << model.Y() << " \t PositionZ: " << model.Z() << "\n";
            ResetWorld();
            model = this->world->ModelByName("iris").get()->WorldPose();
            std::cout << "PositionX: " << model.X() << "\t PositionY: " << model.Y() << "\t PositionZ: " << model.Z() << "\n";
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
}

GZ_REGISTER_WORLD_PLUGIN(gazebo::SimulationResetPlugin)
