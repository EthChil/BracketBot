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

#include "moteus_protocol.h"

//build command
//g++ canfdtest.cpp -I. -lsocketcan -o send_can_message

mjbots::moteus::CanFrame ConvertToMoteusCanFrame(const canfd_frame& input_frame) {
    mjbots::moteus::CanFrame output_frame;
    output_frame.size = input_frame.len;
    std::memcpy(output_frame.data, input_frame.data, input_frame.len);
    return output_frame;
}


int main()
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame frame;
    struct canfd_frame response_frame;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Error opening socket");
        return -1;
    }

    int enable_canfd = 1;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));
    
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


        // Convert the response_frame to a Moteus CanFrame
        mjbots::moteus::CanFrame moteus_response_frame = ConvertToMoteusCanFrame(response_frame);

        // Create a parser with the Moteus CanFrame
        mjbots::moteus::MultiplexParser parser(&moteus_response_frame);


        while (true) {
            auto [valid, reg, res] = parser.next();
            if (!valid) {
                break;
            }

            // // Here, you'll need to call the appropriate decoding method based on the register.
            // // For example, assuming register 1 corresponds to position, 2 corresponds to velocity, and 3 corresponds to torque:
            // switch (reg) {
            //     case 1: {  // Position
            //     double position = parser.ReadPosition(res);
            //     std::cout << "Position: " << position << std::endl;
            //     break;
            //     }
            //     case 2: {  // Velocity
            //     double velocity = parser.ReadVelocity(res);
            //     std::cout << "Velocity: " << velocity << std::endl;
            //     break;
            //     }
            //     case 3: {  // Torque
            //     double torque = parser.ReadTorque(res);
            //     std::cout << "Torque: " << torque << std::endl;
            //     break;
            //     }
            //     default: {
            //     std::cout << "Unknown register: " << reg << std::endl;
            //     break;
            //     }
            // }
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