#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

int main() {
    // Memory name (should match with the name used in Python)
    const char* memory_name = "droneAction";
    
    // Open the shared memory segment
    int fd = shm_open(memory_name, O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "Failed to open shared memory" << std::endl;
        return 1;
    }

    // Map the shared memory segment into the address space
    void* shared_memory = mmap(NULL, sizeof(int), PROT_READ, MAP_SHARED, fd, 0);
    if (shared_memory == MAP_FAILED) {
        std::cerr << "Failed to map shared memory" << std::endl;
        return 1;
    }

    // Read data from the mapped memory
    int* data = static_cast<int*>(shared_memory);
    int value;
    std::memcpy(&value, data, sizeof(int)); // Assuming integer data
    std::cout << "Received value from shared memory: " << value << std::endl;

    // Unmap and close shared memory
    munmap(shared_memory, sizeof(int));
    close(fd);

    return 0;
}