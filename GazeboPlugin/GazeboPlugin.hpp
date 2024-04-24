#pragma once // Include guard to prevent multiple inclusion

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <random>

namespace gazebo
{
    class SimulationResetPlugin : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
        void OnUpdate();
        void ResetWorld();

    private:
        ignition::math::Pose3d RandomPose();

        physics::WorldPtr world;
        event::ConnectionPtr updateConnection;
    };
}
