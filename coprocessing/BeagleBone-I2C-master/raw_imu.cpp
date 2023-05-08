#include "Lib/I2C/I2CDevice.h"
#include <cmath>      // Include for sqrt and atan2
#include <math.h>     // Include for M_PI
#include <iostream>
#include <tuple>     // Include for std::tuple
#include <chrono>

//build command:
// g++ main.cpp Lib/I2C/I2CDevice.cpp -o i2c_test -std=c++11

float getCurrentTime() {
    static const auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = now - start_time;
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count()/1000000.0F;
}

constexpr float ACCEL_SENSITIVITY = 0.000061; // g/LSB (±2g)
constexpr float GYRO_SENSITIVITY = 0.00875;    // dps/LSB (±245 dps)
constexpr float MAG_SENSITIVITY = 0.0000305; // gauss/LSB (±4 gauss)

class LSM9DS1_Base : public abI2C::I2CDevice {
public:
    LSM9DS1_Base( unsigned char deviceAddress, unsigned int busId ) {
        this->DeviceAddress = deviceAddress;
        this->BusId = busId;
        this->InitDevice( );
    }
};

class LSM9DS1_Magnetometer : public LSM9DS1_Base {
public:
    LSM9DS1_Magnetometer(unsigned char deviceAddress, unsigned int busId)
        : LSM9DS1_Base(deviceAddress, busId) {}

    void SetDeviceAddress(unsigned char deviceAddress) override {
        this->DeviceAddress = deviceAddress;
    }

    void SetBusId(int busId) override {
        this->BusId = busId;
    }

     void initializeMag() {
        SetRegisterAddress(0x20);
        SetRegisterValue(0x7F);
        if (WriteToDevice(2) != 0) {
            std::cout << "Failed to write to CTRL_REG1_M" << std::endl;
        }

        SetRegisterAddress(0x22);
        SetRegisterValue(0x00);
        if (WriteToDevice(2) != 0) {
            std::cout << "Failed to write to CTRL_REG3_G" << std::endl;
        }

        SetRegisterAddress(0x23);
        SetRegisterValue(0x0C);
        if (WriteToDevice(2) != 0) {
            std::cout << "Failed to write to CTRL_REG4_G" << std::endl;
        }
    }

    void readMagnetometer() {
        short values[6];
        GetValuesFromRegisters(0x28, values, 6);
        
        magX = static_cast<int16_t>((values[1] << 8) | values[0]) * MAG_SENSITIVITY;
        magY = static_cast<int16_t>((values[3] << 8) | values[2]) * MAG_SENSITIVITY;
        magZ = static_cast<int16_t>((values[5] << 8) | values[4]) * MAG_SENSITIVITY;
    }

    // Add the ReadMag function
    void ReadMag() {
        uint8_t status_reg = GetValueFromRegister(0x27);

        if (status_reg & 0x8) { // Bit 4 (ZYXDA) is set
            readMagnetometer();
        }
    }

    // Mark accessor functions as const
    double getMagX() const { return magX; }
    double getMagY() const { return magY; }
    double getMagZ() const { return magZ; }

    private:
    double magX = 0.0;
    double magY = 0.0;
    double magZ = 0.0;
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

    void GravityVectorToAngle(float &pitch, float &roll, float &yaw, float delta_t) const {
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

    double getAccelX() const { return accelX; }
    double getAccelY() const { return accelY; }
    double getAccelZ() const { return accelZ; }
    double getGyroX() const { return gyroX; }
    double getGyroY() const { return gyroY; }
    double getGyroZ() const { return gyroZ; }
    double getPitch() const { return pitch; }
    double getYaw() const { return yaw; }
    double getRoll() const { return roll; }

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

int main( void ) {

    using namespace abI2C;

    LSM9DS1_Accelerometer_Gyroscope acc_gyro(0x6b, 2);
    LSM9DS1_Magnetometer mag(0x1e, 2);
    acc_gyro.initializeAccelGyro();
    mag.initializeMag();

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

        acc_gyro.ReadAccelGyro();
        mag.ReadMag();
        acc_gyro.GravityVectorToAngle(pitch, roll, yaw, delta_tony);
        
        printf("Pitch: %f, Yaw: %f, Roll: %f, Pitch Rate: %f, Yaw Rate: %f, Roll Rate: %f, Delta Time: %f\n", -roll, -yaw, pitch, acc_gyro.getGyroY(), -acc_gyro.getGyroZ(), acc_gyro.getGyroX(), delta_tony);
        // printf("Mag X: %f, Mag Y: %f, Mag Z: %f\n",mag.getMagX(), mag.getMagY(), mag.getMagZ());
        prev_time = curr_time;
        curr_time = getCurrentTime(); // Replace with function to get current time
        double elapsed_time = (curr_time - prev_time) * 1000000.0; // Time elapsed since the start of the loop, in microseconds

        double sleep_duration = loop_time_target - elapsed_time;

        if (sleep_duration > 0) {
            usleep(static_cast<useconds_t>(sleep_duration));
        }

    }
}