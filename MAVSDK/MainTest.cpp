#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <cerrno>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

#define RED "\033[31m"
#define CLEAR "\033[0m"

using mavsdk::Mavsdk;
using mavsdk::ConnectionResult;
using mavsdk::Offboard;
using mavsdk::Action;
using mavsdk::Telemetry;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

float vel = 1.0;

struct SharedData
{
    int action;
    bool RunLoop;
};

void custom_control(mavsdk::Offboard& offboard, SharedData *sharedData);
bool offb_ctrl_body(mavsdk::Offboard& offboard, SharedData *sharedData);
Offboard::VelocityBodyYawspeed action_translate(int dqn_action);
void usage(const std::string& bin_name);

int main(int argc, char** argv) // To run: ./MainTest.out udp://:14540
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    // Shared memory
    const char* memoryName = "droneAction";
    int shm_fd = shm_open(memoryName, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR | S_IXUSR);
    if (shm_fd == -1) {
        std::cerr << RED << "shm_open" << CLEAR << std::endl;
    }

    if (ftruncate(shm_fd, sizeof(SharedData)) == -1) {
        std::cerr << RED << "ftruncate" << CLEAR << std::endl;
    }

    SharedData *sharedData = (SharedData*) mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (sharedData == MAP_FAILED) {
        std::cerr << RED << "mmap" << CLEAR << std::endl;
    }

    //MAVSDK stuff

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins.
    auto action = Action{system.value()};
    auto offboard = Offboard{system.value()};
    auto telemetry = Telemetry{system.value()};

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }
    std::cout << "System is ready\n";

    const auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    std::cout << "Armed\n";

    const auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    auto in_air_promise = std::promise<void>{};
    auto in_air_future = in_air_promise.get_future();
    Telemetry::LandedStateHandle handle = telemetry.subscribe_landed_state(
        [&telemetry, &in_air_promise, &handle](Telemetry::LandedState state) {
            if (state == Telemetry::LandedState::InAir) {
                std::cout << "Taking off has finished.\n";
                telemetry.unsubscribe_landed_state(handle);
                in_air_promise.set_value();
            }
        });
    in_air_future.wait_for(seconds(10));
    if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "Takeoff timed out.\n";
        return 1;
    }

    //  using body co-ordinates
    if (!offb_ctrl_body(offboard, sharedData)) {
        return 1;
    }

    const auto land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Landing failed: " << land_result << '\n';
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

    // We are relying on auto-disarming but let's keep watching the telemetry for
    // a bit longer.

    munmap(sharedData, sizeof(SharedData));
    close(shm_fd);

    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}

void custom_control(mavsdk::Offboard& offboard, SharedData *sharedData) // Drone control code goes here
{
    
    float degSpeed;
    bool RunLoop = true;
    std::cout << "Doing offboard stuff!\n";
    Offboard::VelocityBodyYawspeed velocity{};

    while (RunLoop)
    {
        velocity = action_translate(sharedData->action);
        //velocity.down_m_s = -1;
        RunLoop = sharedData->RunLoop;
        offboard.set_velocity_body(velocity);
        sleep_for(milliseconds(1));
    }

    std::cout << "Holding position...\n";
    velocity.down_m_s = 0.0f;
	velocity.forward_m_s = 0.0f;
	velocity.right_m_s = 0.0f;
    velocity.yawspeed_deg_s = 0.0f;
    offboard.set_velocity_body(velocity);
    sleep_for(seconds(5));

    std::cout << "Flying down...\n";
    velocity.down_m_s = 1.0f;
    offboard.set_velocity_body(velocity);
    sleep_for(seconds(6));
}

bool offb_ctrl_body(mavsdk::Offboard& offboard, SharedData *sharedData)
{
    std::cout << "Starting Offboard velocity control in body coordinates\n";

    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::VelocityBodyYawspeed stay{};
    offboard.set_velocity_body(stay);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard started\n";

    custom_control(offboard, sharedData);

    std::cout << "Wait for a bit\n";
    sleep_for(seconds(3));

    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard stopped\n";
    
    return true;
}

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

Offboard::VelocityBodyYawspeed action_translate(int dqn_action)
{
    Offboard::VelocityBodyYawspeed action{};
    std::cout << "Action: " << dqn_action << "\n";
    switch(dqn_action){
        case 0:
            action.down_m_s = 0.0f;
            action.forward_m_s = 0.0f;
            action.right_m_s = 0.0f;
            action.yawspeed_deg_s = 0.0f;
            break;
        case 1:
            action.down_m_s = 0.0f;
            action.forward_m_s = vel;
            action.right_m_s = 0.0f;
            action.yawspeed_deg_s = 0.0f;
            break;
        case 2:
            action.down_m_s = 0.0f;
            action.forward_m_s = -vel;
            action.right_m_s = 0.0f;
            action.yawspeed_deg_s = 0.0f;
            break;
        case 3:
            action.down_m_s = 0.0f;
            action.forward_m_s = 0.0f;
            action.right_m_s = vel;
            action.yawspeed_deg_s = 0.0f;
            break;
        case 4:
            action.down_m_s = 0.0f;
            action.forward_m_s = 0.0f;
            action.right_m_s = -vel;
            action.yawspeed_deg_s = 0.0f;
            break;
        default:
            action.down_m_s = 0.0f;
            action.forward_m_s = 0.0f;
            action.right_m_s = 0.0f;
            action.yawspeed_deg_s = 0.0f;
            break;
    }

    return action;
}
//testing