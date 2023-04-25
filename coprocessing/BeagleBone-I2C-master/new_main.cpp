#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>      // Include for sqrt and atan2
#include <math.h>     // Include for M_PI
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

#include "moteus_protocol.h"
#include "Lib/I2C/I2CDevice.h"


//We will run the IMU at 500hz and the moteus at 200hz

//build command:
// g++ new_main.cpp -pthread -I. -lsocketcan Lib/I2C/I2CDevice.cpp -o mainprogram -std=c++11

float getCurrentTime() {
    static const auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = now - start_time;
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count()/1000000.0F;
}

constexpr float ACCEL_SENSITIVITY = 0.000061; // g/LSB (±2g)
constexpr float GYRO_SENSITIVITY = 0.00875;    // dps/LSB (±245 dps)

class LSM9DS1_Base : public abI2C::I2CDevice {
public:
    LSM9DS1_Base( unsigned char deviceAddress, unsigned int busId ) {
        this->DeviceAddress = deviceAddress;
        this->BusId = busId;
        this->InitDevice( );
    }
};

class LSM9DS1_Accelerometer_Gyroscope : public LSM9DS1_Base {
public:
    LSM9DS1_Accelerometer_Gyroscope(unsigned char deviceAddress, unsigned int busId)
        : LSM9DS1_Base(deviceAddress, busId) {}

    void SetDeviceAddress(unsigned char deviceAddress) override {
        this->DeviceAddress = deviceAddress;
    }

    void SetBusId(int busId) override {
        this->BusId = busId;
    }

     void initializeAccelGyro() {
        SetRegisterAddress(0x20); // CTRL_REG6_XL
        SetRegisterValue(0xC0);  // ODR 952 Hz, FS 2g
        if (WriteToDevice(2) != 0) {
            std::cout << "Failed to write to CTRL_REG6_XL" << std::endl;
        }

        SetRegisterAddress(0x10); // CTRL_REG1_G
        SetRegisterValue(0xC0);  // ODR 952 Hz, FS 245 dps
        if (WriteToDevice(2) != 0) {
            std::cout << "Failed to write to CTRL_REG1_G" << std::endl;
        }
    }

    void readAccelerometer() {
        short values[6];
        GetValuesFromRegisters(0x28, values, 6);
        
        accelX = static_cast<int16_t>((values[1] << 8) | values[0]) * ACCEL_SENSITIVITY;
        accelY = static_cast<int16_t>((values[3] << 8) | values[2]) * ACCEL_SENSITIVITY;
        accelZ = static_cast<int16_t>((values[5] << 8) | values[4]) * ACCEL_SENSITIVITY;
    }

    void readGyroscope() {
        short values[6];
        GetValuesFromRegisters(0x18, values, 6);
        
        gyroX = static_cast<int16_t>((values[1] << 8) | values[0]) * GYRO_SENSITIVITY;
        gyroY = static_cast<int16_t>((values[3] << 8) | values[2]) * GYRO_SENSITIVITY;
        gyroZ = static_cast<int16_t>((values[5] << 8) | values[4]) * GYRO_SENSITIVITY;
    }

    // Add the ReadAccelGyro function
    void ReadAccelGyro() {
        uint8_t status_reg = GetValueFromRegister(0x17);

        if (status_reg & 0x01) { // Bit 0 (XLDA) is set
            readAccelerometer();
        }

        if (status_reg & 0x02) { // Bit 1 (GDA) is set
            readGyroscope();
        }
    }

    // Add the CalculateYawPitchRoll function
    void CalculateYawPitchRoll(float &pitch, float &roll, float &yaw, float delta_t) const {
        float ax = getAccelX();
        float ay = getAccelY();
        float az = getAccelZ();

        float gx = getGyroX();
        float gy = getGyroY();
        float gz = getGyroZ();

        pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
        roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
        yaw += gz * delta_t;
    }

    // Mark accessor functions as const
    double getAccelX() const { return accelX; }
    double getAccelY() const { return accelY; }
    double getAccelZ() const { return accelZ; }
    double getGyroX() const { return gyroX; }
    double getGyroY() const { return gyroY; }
    double getGyroZ() const { return gyroZ; }
    double getPitch() const { return gyroX; }
    double getYaw() const { return gyroY; }
    double getRoll() const { return gyroZ; }

    private:
    double accelX = 0.0;
    double accelY = 0.0;
    double accelZ = 0.0;

    double gyroX = 0.0;
    double gyroY = 0.0;
    double gyroZ = 0.0;

    double pitch = 0.0;
    double yaw = 0.0;
    double roll = 0.0;

};


struct SharedData {
    double pitch;
    double yaw;
    double pitchRate;
    double yawRate;

    std::mutex mtx;  // Mutex to protect access to the shared data
};

mjbots::moteus::CanFrame ConvertToMoteusCanFrame(const canfd_frame& input_frame) {
    mjbots::moteus::CanFrame output_frame;
    output_frame.size = input_frame.len;
    std::memcpy(output_frame.data, input_frame.data, input_frame.len);
    return output_frame;
}

void read_imu(SharedData& data) {

    LSM9DS1_Accelerometer_Gyroscope acc_gyro(0x6b, 2);
    acc_gyro.initializeAccelGyro();

    float pitch = 0;
    float roll = 0;
    float yaw = 0;

    double delta_tony = 0;
    double curr_time = 0;


    while (true) {
        std::unique_lock<std::mutex> lock(data.mtx);

        acc_gyro.ReadAccelGyro();
        acc_gyro.CalculateYawPitchRoll(pitch, roll, yaw, delta_tony);

        data.pitch = acc_gyro.getPitch();
        data.yaw = acc_gyro.getYaw();
        data.pitchRate = acc_gyro.getGyroY();
        data.yawRate = acc_gyro.getGyroZ();

        std::cout << "Reading IMU data: " << data.pitch << std::endl;
        lock.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Adjust the sleep time based on the required IMU update rate
    }
}

void parse_pvt(mjbots::moteus::MultiplexParser &parser){
         while (true) {
            auto [valid, reg, res] = parser.next();
            if (!valid) {
                break;
            }

            // Here, you'll need to call the appropriate decoding method based on the register.
            // For example, assuming register 1 corresponds to position, 2 corresponds to velocity, and 3 corresponds to torque:
            switch (reg) {
                case 1: {  // Position
                double position = parser.ReadPosition(res);
                std::cout << "Position: " << position << std::endl;
                break;
                }
                case 2: {  // Velocity
                double velocity = parser.ReadVelocity(res);
                std::cout << "Velocity: " << velocity << std::endl;
                break;
                }
                case 3: {  // Torque
                double torque = parser.ReadTorque(res);
                std::cout << "Torque: " << torque << std::endl;
                break;
                }
                default: {
                std::cout << "Unknown register: " << reg << std::endl;
                break;
                }
            }
        }
    }



void send_moteus_commands(SharedData& data) {


    // ----- INITIALIZE
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame read_pvt_frame; //frame predefined for reading position, velocity, and torque
    struct canfd_frame response_frame;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Error opening socket");
        // return -1;
    }

    int enable_canfd = 1;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));
    
    strcpy(ifr.ifr_name, "vcan0");
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting interface index");
        // return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error binding socket");
        // return -1;
    }
    // -------- END INITIALIZE

    //query position, velocity, and torque
    read_pvt_frame.can_id = 0x8001 | CAN_EFF_FLAG; // CAN ID (Extended format)
    read_pvt_frame.flags = CANFD_ESI | CANFD_BRS;  // Use CAN FD flags
    read_pvt_frame.len = 3;                         // Length of data (3 bytes)
    read_pvt_frame.data[0] = 0x14;                 // Read int16 registers
    read_pvt_frame.data[1] = 0x03;                 // Number of registers to read (3 registers: Position and Velocity)
    read_pvt_frame.data[2] = 0x001;  

    // Initial PVT frame
    write(s, &read_pvt_frame, sizeof(read_pvt_frame));
    int nbytes = read(s, &response_frame, sizeof(response_frame));

    mjbots::moteus::CanFrame moteus_response_frame = ConvertToMoteusCanFrame(response_frame);
    mjbots::moteus::MultiplexParser parser(&moteus_response_frame);



    while (true) {
        std::unique_lock<std::mutex> lock(data.mtx);

         // Initial PVT frame
        write(s, &read_pvt_frame, sizeof(read_pvt_frame));
        int nbytes = read(s, &response_frame, sizeof(response_frame));

        mjbots::moteus::CanFrame moteus_response_frame = ConvertToMoteusCanFrame(response_frame);
        mjbots::moteus::MultiplexParser parser(&moteus_response_frame);

        parse_pvt(parser);


        std::cout << "Sending Moteus command with IMU data: " << data.pitch << std::endl;
        lock.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Adjust the sleep time based on the required Moteus command rate
    }
}

int main() {
    SharedData data;

    std::thread imu_thread(read_imu, std::ref(data));
    std::thread moteus_thread(send_moteus_commands, std::ref(data));

    imu_thread.join();
    moteus_thread.join();

    return 0;
}