#include "Lib/I2C/I2CDevice.h"
#include <cmath>      // Include for sqrt and atan2
#include <math.h>     // Include for M_PI
#include <iostream>
#include <tuple>     // Include for std::tuple


#include <chrono>

float getCurrentTime() {
    static const auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = now - start_time;
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count()/1000000.0F;
}

constexpr float ACCEL_SENSITIVITY = 0.000061; // g/LSB (±2g)
constexpr float GYRO_SENSITIVITY = 0.00875;    // dps/LSB (±245 dps)

#define SENSORS_GRAVITY_STANDARD 9.80665



/*
 * Simplest implementation of a usable class extending an I2C device
 */
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

    // int16_t GetValueFromRegister16(uint8_t low_reg, uint8_t high_reg) {
    //     uint16_t high_byte = static_cast<uint16_t>(GetValueFromRegister(high_reg));
    //     uint16_t low_byte = static_cast<uint16_t>(GetValueFromRegister(low_reg));
    //     uint16_t combined = (high_byte << 8) | low_byte;
    //     return static_cast<int16_t>(combined);
    // }

    
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

    // Mark accessor functions as const
    double getAccelX() const { return accelX; }
    double getAccelY() const { return accelY; }
    double getAccelZ() const { return accelZ; }
    double getGyroX() const { return gyroX; }
    double getGyroY() const { return gyroY; }
    double getGyroZ() const { return gyroZ; }

    private:
    double accelX = 0.0;
    double accelY = 0.0;
    double accelZ = 0.0;

    double gyroX = 0.0;
    double gyroY = 0.0;
    double gyroZ = 0.0;

};



/*
 * Test I2C functionality
 * Prints a short (16 bits) with the current X reading from the accelerometer.
 */

void ReadAccelGyro(LSM9DS1_Accelerometer_Gyroscope &sensor) {
    uint8_t status_reg = sensor.GetValueFromRegister(0x17);

    if (status_reg & 0x01) { // Bit 0 (XLDA) is set
        sensor.readAccelerometer();
        // std::cout << "accel read" << std::endl;
    }

    if (status_reg & 0x02) { // Bit 1 (GDA) is set
        sensor.readGyroscope();
        // std::cout << "gyro read" << std::endl;
    }
}

void CalculateYawPitchRoll(const LSM9DS1_Accelerometer_Gyroscope &sensor, float &pitch, float &roll, float &yaw, float delta_t) {
    float ax = sensor.getAccelX();
    float ay = sensor.getAccelY();
    float az = sensor.getAccelZ();

    float gx = sensor.getGyroX();
    float gy = sensor.getGyroY();
    float gz = sensor.getGyroZ();

    // Calculate the pitch and roll from the accelerometer data
    pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
    roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

    // Integrate the gyroscope data to calculate the yaw
    yaw += gz * delta_t; // Use delta_t for the yaw calculation
}

int main( void ) {

    using namespace abI2C;

    LSM9DS1_Accelerometer_Gyroscope acc_gyro(0x6b, 2);



    acc_gyro.SetRegisterAddress(0x20); // CTRL_REG6_XL
    acc_gyro.SetRegisterValue(0xC0);  // ODR 952 Hz, FS 2g
    if (acc_gyro.WriteToDevice(2) != 0) {
        std::cout << "Failed to write to CTRL_REG6_XL" << std::endl;
    }

    // Enable the accelerometer and gyroscope
    acc_gyro.SetRegisterAddress(0x10); // CTRL_REG1_G
    acc_gyro.SetRegisterValue(0xC0);  // ODR 952 Hz, FS 245 dps
    if (acc_gyro.WriteToDevice(2) != 0) {
        std::cout << "Failed to write to CTRL_REG1_G" << std::endl;
    }


    float pitch = 0;
    float roll = 0;
    float yaw = 0;

    double delta_tony = 0;
    double curr_time = 0;


    double prev_time = 0.0;
    double loop_time_target = 2e3; // 2 milliseconds in microseconds

    while (true) {
        curr_time = getCurrentTime(); // Replace with function to get current time
        delta_tony = curr_time - prev_time;
        
        ReadAccelGyro(acc_gyro);
        CalculateYawPitchRoll(acc_gyro, pitch, roll, yaw, delta_tony);
        
        printf("Pitch: %f, Yaw: %f, Pitch Rate: %f, Yaw Rate: %f, Delta Time: %f\n", pitch, yaw, acc_gyro.getGyroY(),acc_gyro.getGyroZ(), delta_tony);

        prev_time = curr_time;
        curr_time = getCurrentTime(); // Replace with function to get current time
        double elapsed_time = (curr_time - prev_time) * 1000000.0; // Time elapsed since the start of the loop, in microseconds

        double sleep_duration = loop_time_target - elapsed_time;

        if (sleep_duration > 0) {
            usleep(static_cast<useconds_t>(sleep_duration));
        }

    }
}
