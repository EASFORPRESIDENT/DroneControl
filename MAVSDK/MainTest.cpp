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
#include <mavsdk/plugins/param/param.h>

#define RED "\033[31m"
#define CLEAR "\033[0m"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

float vel = 1.0;




struct SharedData
{
    bool reset;
    double posX;
    double posY;
    double posZ;
    double posYaw;
};


void custom_control(mavsdk::Offboard& offboard, SharedData *sharedData, SharedData *boxdata);
bool offb_ctrl_body(mavsdk::Offboard& offboard, SharedData *sharedData, SharedData *boxdata);
void action_translate(SharedData *sharedData,  SharedData *boxdata, Offboard::VelocityBodyYawspeed *velocity);
void usage(const std::string& bin_name);

int main(int argc, char** argv) // To run: ./MainTest.out udp://:14540
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    // Shared memory dronepose
    const char* memoryName = "dronePoseAndReset";
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

    sharedData->reset = false;





    // Shared memory boxpose
    const char* memoryName = "boxpose";
    int shm_fd2 = shm_open(memoryName, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR | S_IXUSR);
    if (shm_fd2 == -1) {
        std::cerr << RED << "shm_open" << CLEAR << std::endl;
    }

    if (ftruncate(shm_fd2, sizeof(SharedData)) == -1) {
        std::cerr << RED << "ftruncate" << CLEAR << std::endl;
    }

    SharedData *boxdata = (SharedData*) mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd2, 0);
    if (boxdata == MAP_FAILED) {
        std::cerr << RED << "mmap" << CLEAR << std::endl;
    }

    boxdata->reset = false;

    //TODO: add shared memory for boxpose in the plugin


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

    // Set SDLOG_MODE parameter to 0 (disable logging)
    auto param = Param{system.value()};
    const std::string param_name = "SDLOG_MODE";
    std::pair<Param::Result, int32_t> param_result;
    const int param_value = -1;

    param_result.first = param.set_param_int(param_name, param_value);
    if (param_result.first != Param::Result::Success) {
        std::cerr << "Failed to set parameter!" << std::endl;
        return 1;
    }

    param_result = param.get_param_int(param_name);
    if (param_result.first != Param::Result::Success) {
        std::cerr << "Failed to get parameter!" << std::endl;
        return 1;
    }

    std::cout << "SDLOG_MODE is now set to: " << param_result.second << std::endl;

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
    if (!offb_ctrl_body(offboard, sharedData, boxdata)) {
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
    munmap(boxdata, sizeof(SharedData));
    close(shm_fd2);


    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}

void custom_control(mavsdk::Offboard& offboard, SharedData *sharedData, SharedData *boxdata) // Drone control code goes here
{
    
    float degSpeed;
    bool RunLoop = true;
    std::cout << "Doing offboard stuff!\n";
    Offboard::VelocityBodyYawspeed velocity{};

    while (true)
    {
        

        //std::cout << "Action: " << sharedData->action << "\n";
        action_translate(sharedData, boxdata, &velocity);
        //velocity.down_m_s = -1;
        //std::cout << "Forward: " << velocity.forward_m_s << "   Right: " << velocity.right_m_s << "   Down: " << velocity.down_m_s << "\n";
        //RunLoop = sharedData->Runloop;
        offboard.set_velocity_body(velocity);
        sleep_for(milliseconds(10));
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

bool offb_ctrl_body(mavsdk::Offboard& offboard, SharedData *sharedData, SharedData *boxdata)
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

    custom_control(offboard, sharedData, boxdata);

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

void action_translate(SharedData *sharedData, SharedData *boxdata, Offboard::VelocityBodyYawspeed *velocity)
{
    // PID controller gains
    float kp = 0.3; // Proportional gain
    int static counter = 0;
    counter++;
    if(counter == 500){

        std::cout << "X: " << sharedData->posX << "\nY: " << sharedData->posY << "\n" << std::endl;

        counter = 0;

    }

    // changed X and Y order. right seems to be y coordinate and forward seems to be x coordinate

    float drone_z = sharedData->posZ - 10.0f; // 10m altitude
    float drone_y = sharedData->posX;
    float drone_x = sharedData->posY;
    float drone_yaw = sharedData->posYaw;

    float box_y = boxdata->posX;
    float box_x = boxdata->posY;



    // Calculate control signals using P controller
    float control_z = kp * drone_z;
    float control_y = -kp * (drone_y - box_y);
    float control_x = kp * (drone_x - box_x);
    float control_yaw = -kp * drone_yaw;

    // Set desired velocity based on control signals
    velocity->down_m_s = control_z;
    velocity->forward_m_s = control_y;
    velocity->right_m_s = control_x;
    velocity->yawspeed_deg_s = control_yaw;
}
//testing