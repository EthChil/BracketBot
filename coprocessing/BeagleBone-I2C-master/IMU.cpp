#include "Lib/I2C/I2CDevice.h"
#include "Lib/Attitude_Estimator/attitude_estimator.h"
#include <cmath>      // Include for sqrt and atan2
#include <math.h>     // Include for M_PI
#include <iostream>
#include <fstream>
#include <tuple>     // Include for std::tuple
#include <chrono>
#include <vector>

//build command:
// g++ IMU.cpp Lib/I2C/I2CDevice.cpp Lib/Attitude_Estimator/attitude_estimator.cpp -o i2c_test -std=c++11

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

        accelX = static_cast<int16_t>((values[3] << 8) | values[2]) * ACCEL_SENSITIVITY;
        accelY = static_cast<int16_t>((values[1] << 8) | values[0]) * ACCEL_SENSITIVITY;
        accelZ = static_cast<int16_t>((values[5] << 8) | values[4]) * ACCEL_SENSITIVITY;
    }

    void readGyroscope() {
        short values[6];
        GetValuesFromRegisters(0x18, values, 6);
        
        gyroX = static_cast<int16_t>((values[1] << 8) | values[0]) * GYRO_SENSITIVITY * M_PI / 180;
        gyroY = static_cast<int16_t>((values[3] << 8) | values[2]) * GYRO_SENSITIVITY * M_PI / 180;
        gyroZ = static_cast<int16_t>((values[5] << 8) | values[4]) * GYRO_SENSITIVITY * M_PI / 180;
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

bool exportData(std::vector<std::vector<double>> data) {
    std::ofstream outFile("data.txt");
    if (outFile.is_open()) {
        for (const auto &row : data) {
            for (size_t j = 0; j < row.size(); ++j) {
                outFile << row[j];
                if (j < row.size() - 1) {
                    outFile << ",";
                }
            }
            outFile << std::endl;
        }
        outFile.close();
        return true;
    } else {
         std::cout << "Unable to open file for writing." << std::endl;
        return false;
    }
}

int main( void ) {
    using namespace abI2C;

    double delta_tony = 0;
    double curr_time = 0;
    double prev_time = 0.0;
    double loop_time_target = 2e3; // 2 milliseconds in microseconds

    std::vector<std::vector<double>> est_data;

    LSM9DS1_Accelerometer_Gyroscope acc_gyro(0x6b, 2);
    LSM9DS1_Magnetometer mag(0x1e, 2);
    acc_gyro.initializeAccelGyro();
    mag.initializeMag();

    stateestimation::AttitudeEstimator Est;
    Est.setPIGains(0.1, 0, 0, 0); // Kp, Ki, Quick Kp, Quick Ki
    Est.setAttitudeEuler(M_PI, 0.0, 0.0); // Sets default
    while(getCurrentTime()<10)
    {
        // Measure tony
        curr_time = getCurrentTime();
        delta_tony = curr_time - prev_time;
        prev_time = curr_time;
        // Update IMU
        acc_gyro.ReadAccelGyro();
        mag.ReadMag();

        Est.update(loop_time_target, acc_gyro.getGyroX(), acc_gyro.getGyroY(), acc_gyro.getGyroZ(), acc_gyro.getAccelX(), acc_gyro.getAccelY(), -acc_gyro.getAccelZ(), mag.getMagX(), mag.getMagY(), mag.getMagZ());
        std::vector<double> row = {Est.eulerPitch(), Est.eulerYaw(), Est.eulerRoll()};
        est_data.push_back(row);
        cout << "Pitch: " << Est.eulerPitch() << " Yaw: " << Est.eulerYaw() << " Roll: " << Est.eulerRoll() << endl;

        double sleep_duration = loop_time_target - delta_tony * 1000000;
        if (sleep_duration > 0) {
            usleep(static_cast<useconds_t>(sleep_duration));
        }
    }
    exportData(est_data);
    std::cout << "Data exported..." << std::endl;
}
