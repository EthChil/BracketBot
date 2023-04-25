#include <iostream>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <libsocketcan.h>
#include <chrono>


int main()
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame frame;
    struct canfd_frame response_frame;

    // Set up a raw CAN socket for CAN FD frames
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Error opening socket");
        return -1;
    }

    // Enable CAN FD support on the socket
    int enable_canfd = 1;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

    // Set the interface name (vcan0)
    strcpy(ifr.ifr_name, "vcan0");
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting interface index");
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error binding socket");
        return -1;
    }

    // WORKING VOLTAGE CALL
    // frame.can_id = 0x8001 | CAN_EFF_FLAG; // CAN ID (Extended format)
    // frame.len = 3;                         // Length of data
    // frame.flags = CANFD_ESI | CANFD_BRS;  // Use CAN FD flags
    // frame.data[0] = 0x10;                 // Read command (int8)
    // frame.data[1] = 0x01;                 // Number of registers (1)
    // frame.data[2] = 0x00D;                 // Start register number (0x00D for voltage)


    //query position, velocity, and torque
    frame.can_id = 0x8001 | CAN_EFF_FLAG; // CAN ID (Extended format)
    frame.flags = CANFD_ESI | CANFD_BRS;  // Use CAN FD flags
    frame.len = 3;                         // Length of data (3 bytes)
    frame.data[0] = 0x14;                 // Read int16 registers
    frame.data[1] = 0x03;                 // Number of registers to read (3 registers: Position and Velocity)
    frame.data[2] = 0x001;  


    // Send the CAN frame
    // write(s, &frame, sizeof(frame));

    


    // Number of iterations
    const int iterations = 100; // Change this value to test more/less messages

    // Start the timer
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; i++) {


        // Send the CAN frame
        write(s, &frame, sizeof(frame));

        // Receive the response CAN frame
        int nbytes = read(s, &response_frame, sizeof(response_frame));
        if (nbytes < 0) {
            perror("Error receiving CAN frame");
            return -1;
        } else if (nbytes < (int)sizeof(struct canfd_frame)) {
            fprintf(stderr, "Received incomplete CAN frame\n");
            return -1;  
        } else {
            printf("Received CAN frame: ID=0x%X DLC=%d Data=", response_frame.can_id, response_frame.len);
            for (int i = 0; i < response_frame.len; i++) {
                printf("%02X ", response_frame.data[i]);
            }
            printf("\n");
        }
    }

    // End the timer
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the time elapsed and messages per second
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    double seconds = duration / 1e6;
    double messages_per_second = iterations / seconds;

    std::cout << "Elapsed time: " << seconds << " seconds" << std::endl;
    std::cout << "Messages per second: " << messages_per_second << std::endl;


    // Close the socket
    close(s);
    
    return 0;
}