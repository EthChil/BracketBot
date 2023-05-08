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
#include <Eigen/Dense>
#include <condition_variable>
#include <vector>
#include <array>
#include <fstream>



#include "moteus_protocol.h"
#include "Lib/I2C/I2CDevice.h"


//We will run the IMU at 500hz and the moteus at 200hz

//build command:
// g++ new_main.cpp -pthread -I. -lsocketcan -I/usr/include/eigen3 Lib/I2C/I2CDevice.cpp -o mainprogram -std=c++11

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
    void CalculateYawPitchRoll(float &pitch, float &roll, float &yaw, float delta_time) const {
        float ax = getAccelX();
        float ay = getAccelY();
        float az = getAccelZ();

        float gx = getGyroX();
        float gy = getGyroY();
        float gz = getGyroZ();

        pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
        roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
        yaw += gz * delta_time;
    }

    // Mark accessor functions as const
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

struct ControllerData {
    double position;
    double velocity;
    double torque;
};

struct SharedData {
    double pitch;
    double yaw;
    double pitchRate;
    double yawRate;

    std::mutex mtx;  // Mutex to protect access to the shared data
    std::condition_variable cv; // Condition variable to signal when the initial data is available
    bool initialDataReady = false; // Flag to indicate if the initial data is ready
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
    double prev_time = 0.0;

    acc_gyro.ReadAccelGyro();
    acc_gyro.CalculateYawPitchRoll(pitch, roll, yaw, delta_tony);

    std::unique_lock<std::mutex> lock(data.mtx);

    data.initialDataReady = true;
    data.cv.notify_one(); // Signal that the initial data is ready
    lock.unlock();


    int cycle_count = 0;
    auto start = std::chrono::high_resolution_clock::now();
    auto last_print_time = start;

    while (true) {
        curr_time = getCurrentTime(); // Replace with function to get current time
        delta_tony = curr_time - prev_time;
        prev_time = curr_time;

        acc_gyro.ReadAccelGyro();
        acc_gyro.CalculateYawPitchRoll(pitch, roll, yaw, delta_tony);

        std::unique_lock<std::mutex> lock(data.mtx);
        data.pitch = roll * M_PI / 180.0;
        data.yaw = yaw * M_PI / 180.0;
        data.pitchRate = acc_gyro.getGyroY() * M_PI / 180.0;
        data.yawRate = acc_gyro.getGyroZ() * M_PI / 180.0;
        lock.unlock(); //might be unnessary

        cycle_count++;

        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time);
        if (elapsed.count() >= 1000) {
            std::cout << "IMU Cycles per second: " << cycle_count << std::endl;
            cycle_count = 0;
            last_print_time = now;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));

    }

}


// Update the parse_pvt function to take a ControllerData reference
void parse_pvt(mjbots::moteus::MultiplexParser &parser, ControllerData &data) {
    while (true) {
        auto [valid, reg, res] = parser.next();
        if (!valid) {
            break;
        }

        switch (reg) {
            case 1: {  // Position
                data.position = parser.ReadPosition(res);
                // std::cout << "Position: " << data.position << std::endl;
                break;
            }
            case 2: {  // Velocity
                data.velocity = parser.ReadVelocity(res);
                // std::cout << "Velocity: " << data.velocity << std::endl;
                break;
            }
            case 3: {  // Torque
                data.torque = parser.ReadTorque(res);
                // std::cout << "Torque: " << data.torque << std::endl;
                break;
            }
            default: {
                std::cout << "Unknown register: " << reg << std::endl;
                break;
            }
        }
    }
}


void update_position(float &x, float &y, float &theta, float d_left, float d_right) {
    float W = 0.567;

    double distance_left = d_left;
    double distance_right = d_right;

    double distance_avg = (distance_left + distance_right) / 2;
    double delta_theta = (distance_right - distance_left) / W;

    x += distance_avg * std::cos(theta + delta_theta / 2);
    y += distance_avg * std::sin(theta + delta_theta / 2);
    theta += delta_theta;
}

void predict(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const Eigen::VectorXd &u, const Eigen::MatrixXd &G) {
    x = F * x + G * u;
    P = F * P * F.transpose() + Q;
}

void update(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &H) {
    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * y;
    Eigen::MatrixXd KH = K * H;
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(KH.rows(), KH.cols()) - KH;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
}

Eigen::MatrixXd read_matrix_from_file(const std::string& file_path, int rows, int cols) {
    std::ifstream file(file_path);
    std::vector<double> data;
    std::string line;

    if (file.is_open()) {
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            double value;
            while (ss >> value) {
                data.push_back(value);
                if (ss.peek() == ',') {
                    ss.ignore();
                }
            }
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << file_path << std::endl;
    }

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat(data.data(), rows, cols);
    return mat;
}



void send_moteus_commands(SharedData& data) {

    struct ControllerData moteus1;
    struct ControllerData moteus2;

    double pitch;
    double yaw;
    double pitchRate;
    double yawRate;

    float Cl = 0;
    float Cr = 0;


    // Set these
    int moteus1_direction = -1;
    int moteus2_direction = -1;


    float t2m = 0.528;
    int N = 6;

    // ------- Kalman filter stuff
    Eigen::MatrixXd A = read_matrix_from_file("./lqr_params/A.txt", N, N);
    Eigen::VectorXd B = read_matrix_from_file("./lqr_params/B.txt", N, 1);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(N, N);
    Eigen::MatrixXd Q_cov = read_matrix_from_file("./lqr_params/Q_cov.txt", N, N);
    Eigen::MatrixXd P;

    std::string p_file_path = "./lqr_params/P.txt";
    if (std::ifstream(p_file_path).good()) {
        P = read_matrix_from_file(p_file_path, N, N);
    } else {
        P = Eigen::MatrixXd::Identity(N, N) * 1e-6;
    }

    Eigen::MatrixXd H = C;
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(N, N);
    R.diagonal() << 1, 1e4, 1, 1e7, 1, 1e8;
    R *= 1e-7;
    Eigen::MatrixXd Q = Q_cov.block(0, 0, N, N);

    // ------ End kalman filter stuff


    // ----- INITIALIZE socket
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame read_pvt_frame_moteus1; //frame predefined for reading position, velocity, and torque
    struct canfd_frame read_pvt_frame_moteus2; //frame predefined for reading position, velocity, and torque
    struct canfd_frame response_frame_moteus1;
    struct canfd_frame response_frame_moteus2;

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

    //Moteus 1 read frame
    read_pvt_frame_moteus1.can_id = 0x8001 | CAN_EFF_FLAG; // CAN ID (Extended format)
    read_pvt_frame_moteus1.flags = CANFD_ESI | CANFD_BRS;  // Use CAN FD flags
    read_pvt_frame_moteus1.len = 3;                         // Length of data (3 bytes)
    read_pvt_frame_moteus1.data[0] = 0x14;                 // Read int16 registers
    read_pvt_frame_moteus1.data[1] = 0x03;                 // Number of registers to read (3 registers: Position and Velocity)
    read_pvt_frame_moteus1.data[2] = 0x001;  

    //Moteus 2 read frame
    read_pvt_frame_moteus2.can_id = 0x8002 | CAN_EFF_FLAG; // CAN ID (Extended format)
    read_pvt_frame_moteus2.flags = CANFD_ESI | CANFD_BRS;  // Use CAN FD flags
    read_pvt_frame_moteus2.len = 3;                         // Length of data (3 bytes)
    read_pvt_frame_moteus2.data[0] = 0x14;                 // Read int16 registers
    read_pvt_frame_moteus2.data[1] = 0x03;                 // Number of registers to read (3 registers: Position and Velocity)
    read_pvt_frame_moteus2.data[2] = 0x001;  


    // INITAL READING
    write(s, &read_pvt_frame_moteus1, sizeof(read_pvt_frame_moteus1));
    read(s, &response_frame_moteus1, sizeof(response_frame_moteus1));

    write(s, &read_pvt_frame_moteus2, sizeof(read_pvt_frame_moteus2));
    read(s, &response_frame_moteus2, sizeof(response_frame_moteus2));

    mjbots::moteus::CanFrame moteus_response_frame_moteus1 = ConvertToMoteusCanFrame(response_frame_moteus1);
    mjbots::moteus::CanFrame moteus_response_frame_moteus2 = ConvertToMoteusCanFrame(response_frame_moteus2);

    mjbots::moteus::MultiplexParser parser1(&moteus_response_frame_moteus1);
    parse_pvt(parser1, moteus1);

    mjbots::moteus::MultiplexParser parser2(&moteus_response_frame_moteus2);
    parse_pvt(parser2, moteus2);


    // Assuming you have m1state and m2state with values for position
    float moteus1_initial_position = moteus1.position * t2m * moteus1_direction;
    float moteus2_initial_position = moteus2.position * t2m * moteus2_direction;
    float combined_initial_position = (moteus1_initial_position + moteus2_initial_position) / 2;

    float moteus1_delta_position = 0;
    float moteus2_delta_position = 0;
    
    float combined_current_position = 0;
    float combined_current_velocity = 0;


    float moteus1_previous_position = 0;
    float moteus2_previous_position = 0;

    float moteus1_previous_torque_command = 0;
    float moteus2_previous_torque_command = 0;
    float x_ego = 0, y_ego = 0, theta_ego = 0;


    //get thie initial imu readings
    std::unique_lock<std::mutex> lock(data.mtx);
    data.cv.wait(lock, [&data] { return data.initialDataReady; });

    pitch = -data.pitch;
    yaw = -data.yaw;
    pitchRate = data.pitchRate;
    yawRate = -data.yawRate;

    lock.unlock(); //might not need


    Eigen::Matrix<double, 1, 6> x_hat;
    x_hat << 0, 0, pitch, pitchRate, yaw, yawRate;

    // Initialize U
    Eigen::VectorXd U;


    Eigen::Matrix<double, 2, 2> D;
    D << 0.5,  0.5,
         0.5, -0.5;

    Eigen::Matrix<double, 2, 6> K;
    K << -1.00, -2.43, -25.97, -12.60, -0.00, -0.00,
         -0.00, -0.00, -0.00, -0.00,  1.00,  1.27;

    Eigen::Matrix<double, 1, 6> Xf;
    Xf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    int cycle_count = 0;
    auto start = std::chrono::high_resolution_clock::now();
    auto last_print_time = start;

    while (true) {

         // Initial PVT frame
        write(s, &read_pvt_frame_moteus1, sizeof(read_pvt_frame_moteus1));
        read(s, &response_frame_moteus1, sizeof(response_frame_moteus1));

        write(s, &read_pvt_frame_moteus2, sizeof(read_pvt_frame_moteus2));
        read(s, &response_frame_moteus2, sizeof(response_frame_moteus2));

        mjbots::moteus::CanFrame moteus_response_frame_moteus1 = ConvertToMoteusCanFrame(response_frame_moteus1);
        mjbots::moteus::CanFrame moteus_response_frame_moteus2 = ConvertToMoteusCanFrame(response_frame_moteus2);

        mjbots::moteus::MultiplexParser parser1(&moteus_response_frame_moteus1);
        parse_pvt(parser1, moteus1);
        mjbots::moteus::MultiplexParser parser2(&moteus_response_frame_moteus2);
        parse_pvt(parser2, moteus2);

        std::unique_lock<std::mutex> lock(data.mtx);
        pitch = -data.pitch;
        yaw = -data.yaw;
        pitchRate = data.pitchRate;
        yawRate = -data.yawRate;
        lock.unlock(); //might not need


        // MORE STUFF

        moteus1_delta_position = moteus1.position*t2m*moteus1_direction - moteus1_previous_position;
        moteus2_delta_position = moteus2.position*t2m*moteus2_direction - moteus2_previous_position;

        update_position(x_ego, y_ego, theta_ego, moteus1_delta_position, moteus2_delta_position);

        combined_current_position = (moteus1.position*t2m + moteus2.position*t2m )/2.0 * moteus1_direction;
        combined_current_velocity = (moteus1.velocity*t2m + moteus2.velocity*t2m )/2.0 * moteus2_direction;

        x_hat[0] = combined_current_position;
        x_hat[1] = combined_current_velocity;
        x_hat[2] = pitch;
        x_hat[3] = pitchRate;
        x_hat[4] = yaw;
        x_hat[5] = yawRate;

        // Calculate U
        U = -K * (x_hat - Xf).transpose();

        // Calculate Cl and Cr
        Eigen::Vector2d wheel_commands = D * U;
        Cl = wheel_commands(0);
        Cr = wheel_commands(1);

        // std::cout << "Position: " << combined_current_position << " Velocity: " << combined_current_velocity << " Pitch: " << pitch << " PitchRate: " << pitchRate << " Yaw: " << yaw << " YawRate: " << yawRate << " Cl: " << Cl << " Cr: " << Cr << std::endl;

        moteus1_previous_position = moteus1.position*t2m*moteus1_direction;
        moteus2_previous_position = moteus2.position*t2m*moteus2_direction;

        cycle_count++;

        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time);
        if (elapsed.count() >= 1000) {
            std::cout << "Moteus Cycles per second: " << cycle_count << std::endl;
            cycle_count = 0;
            last_print_time = now;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500)); // Adjust the sleep time based on the required Moteus command rate
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