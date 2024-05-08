#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include <iostream>
#include <cstring>

#define SHM_SIZE 1024

struct SharedData
{
    bool reset;
    double posX;
    double posY;
    double posZ;
    double posYaw;
};

// Function to serialize SharedData struct into a byte array
void serializeSharedData(const SharedData& data, char* buffer) {
    std::memcpy(buffer, &data, sizeof(SharedData));
}

int main() {
    // Shared memory
    const char* memoryName = "dronePosAndReset";
    auto shm_fd = shm_open(memoryName, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        // Handle error, return or exit
    }

    if (ftruncate(shm_fd, sizeof(SharedData)) == -1) {
        perror("ftruncate");
        // Handle error, close shared memory and return or exit
    }

    auto sharedData = (SharedData*) mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (sharedData == MAP_FAILED) {
        perror("mmap");
        // Handle error, close shared memory and return or exit
    }

    // Modify shared data
    sharedData->posX = 123;
    sharedData->posY = 34;
    sharedData->posZ = 1;
    sharedData->reset = false;
    // Set other fields as needed

    // Serialize SharedData struct into a byte array
    char buffer[sizeof(SharedData)];
    serializeSharedData(*sharedData, buffer);

    // Write serialized data to shared memory
    std::memcpy(sharedData, buffer, sizeof(SharedData));

    std::string yo;
    std::cin >> yo;

    munmap(sharedData, sizeof(SharedData));
    close(shm_fd);
    shm_unlink(memoryName);
    return 0;
}
