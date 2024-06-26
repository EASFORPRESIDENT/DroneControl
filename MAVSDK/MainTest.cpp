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

using mavsdk::Mavsdk;
using mavsdk::ConnectionResult;
using mavsdk::Offboard;
using mavsdk::Action;
using mavsdk::Telemetry;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

struct SharedData
{
    bool reset;
    double posX;
    double posY;
    double posZ;
    double posYaw;
};

void print_position(SharedData *sharedData);
void custom_control(mavsdk::Offboard& offboard, SharedData *sharedData);
bool offb_ctrl_body(mavsdk::Offboard& offboard, SharedData *sharedData);
void usage(const std::string& bin_name);

int main(int argc, char** argv) // To run: ./MainTest.out udp://:14540
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    const char* memoryName = "dronePoseAndReset";
    int shm_fd = shm_open(memoryName, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed"); // Print an error message specific to shm_open
        return 1;
    }

    SharedData* sharedData = (SharedData*) mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    if (sharedData == MAP_FAILED) {
        std::cerr << "Memory mapping failed." << std::endl;
        return 1;
    }

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
                std::cout << "Taking off has finished\n.";
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

    std::cout << "X: " << sharedData->posX << "\t Y: " << sharedData->posY
     << "\t Z: " << sharedData->posZ << "\t Yaw: " << sharedData->posYaw << "\n";

    munmap(sharedData, sizeof(SharedData));
    close(shm_fd);

    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}

void custom_control(mavsdk::Offboard& offboard, SharedData *sharedData) // Drone control code goes here
{
    
    float degSpeed;
    std::cout << "Doing offboard stuff!\n";
    Offboard::VelocityBodyYawspeed velocity{};
    print_position(sharedData);

    velocity.down_m_s = -1.0f;
	velocity.forward_m_s = 1.0f;
	velocity.right_m_s = 0.0f;
    velocity.yawspeed_deg_s = 10;
    offboard.set_velocity_body(velocity);
    sleep_for(seconds(5));
    print_position(sharedData);

    std::cout << "Holding position...\n";
    velocity.down_m_s = 0.0f;
	velocity.forward_m_s = 0.0f;
	velocity.right_m_s = 0.0f;
    velocity.yawspeed_deg_s = 0.0f;
    offboard.set_velocity_body(velocity);
    sleep_for(seconds(5));
    print_position(sharedData);

    std::cout << "Resetting...\n";
    sharedData->reset = true;
    sleep_for(seconds(5));
    print_position(sharedData);

    std::cout << "Flying down...\n";
    velocity.down_m_s = 1.0f;
    offboard.set_velocity_body(velocity);
    sleep_for(seconds(5));
    print_position(sharedData);
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
    offboard.set_velocity_body(stay);
    sleep_for(seconds(3));

    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard stopped\n";
    
    return true;
}

void print_position(SharedData *sharedData)
{
    std::cout << "X: " << sharedData->posX << "\t Y: " << sharedData->posY
     << "\t Z: " << sharedData->posZ << "\t Yaw: " << sharedData->posYaw << "\n";
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