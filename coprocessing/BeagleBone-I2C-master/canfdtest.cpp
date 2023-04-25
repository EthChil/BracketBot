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

    frame.can_id = 0x8001 | CAN_EFF_FLAG; // CAN ID (Extended format)
    frame.len = 3;                         // Length of data
    frame.flags = CANFD_ESI | CANFD_BRS;  // Use CAN FD flags
    frame.data[0] = 0x10;                 // Read command (int8)
    frame.data[1] = 0x01;                 // Number of registers (1)
    frame.data[2] = 0x00D;                 // Start register number (0x00D for voltage)


    // Send the CAN frame
    if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("Error sending CAN frame");
        printf("write returned %d\n", errno);
        printf("frame ID: %x\n", frame.can_id);
        printf("frame length: %d\n", frame.len);
        printf("frame flags: %d\n", frame.flags);
        printf("frame data: ");
        for (int i = 0; i < frame.len; i++) {
            printf("%02X ", frame.data[i]);
        }
        printf("\n");
        printf("addr family: %d\n", addr.can_family);
        printf("addr interface index: %d\n", addr.can_ifindex);
        return -1;
    }

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

    // Close the socket
    close(s);
    
    return 0;
}