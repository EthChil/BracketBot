#include <iostream>
#include <thread>
#include <chrono>

#include "Lib/I2C/I2CDevice.h"
#include <cmath>      // Include for sqrt and atan2
#include <math.h>     // Include for M_PI
#include <tuple>     // Include for std::tuple

//We will run the IMU at 500hz and the moteus at 200hz

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





void read_imu() {
    while (true) {
        // Read IMU data here
        std::cout << "Reading IMU data..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Adjust the sleep time based on the required IMU update rate
    }
}

void send_moteus_commands() {
    while (true) {
        // Send Moteus commands here
        std::cout << "Sending Moteus commands..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust the sleep time based on the required Moteus command rate
    }
}

int main() {
    std::thread imu_thread(read_imu);
    std::thread moteus_thread(send_moteus_commands);

    imu_thread.join();
    moteus_thread.join();

    return 0;
}