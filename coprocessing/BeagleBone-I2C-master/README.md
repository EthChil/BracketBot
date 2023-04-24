# BeagleBone-I2C
C++ Class for handling I2C devices on the BeagleBone

ReadMe
-------------

**This class is intended to be extended by classes for specific I2C devices (e.g LSM303DLHC)**

Designed to run on the BeagleBone Black and provide a class that can easily be extended.

This project is a sub project of Quadro - Quadro contains the compiled binary for this class.

**Requirements**
- Beagle Bone Black [Purchase Info](https://www.adafruit.com/products/1996)
- An I2C Device *recommended* [10DOF IMU BreakOut](https://www.adafruit.com/products/1604)
  - Has onboard : L3GD20H + LSM303 + BMP180 (Accelerometer, Magnetometer, Gyro and Barometer)
- For testing purposes perhaps a [BreadBoard](https://www.adafruit.com/products/702) and [Jumper cables](https://www.adafruit.com/products/826)

**Setup**
- I used the default Debian install - Ubuntu will work just as well.
- JetBrains CLion IDE
- On a Mac book pro I use a SSHFS connection to automatically publish compiled binaries to the BBB *just to save time*
- xtools cross compiler for compiling ARM-LINUX compatible executables for the BBB.

**Notes**
By default I2C2 is available for use on the BBB 

**nb** this I2C Bus is available on Block P9 pins 19 (SCL) and 20 (SDA) see image below.

The header file can very easily be modified for other I2C Buses as required, just add a new constant I2C_# with the relevant file path and increment the total I2C Buses constant.

**Testing**
- First, modify main.cpp to connect to your device (You can obtain the hex address in the devices documentation)
- Next, test writing a configuration to the device, in the current sample writing the value 0x77 to the register 0x20 on the device at 0x19 will enable the accelerometer.
- Next, test reading a value from the device, in the current sample I read registers 0x28 and 0x29, the registers combined contain the X axis details.

**Issues**
Please let me know of any problems and I will endeavour to assist!

![alt text](http://flyingbeaglebone.eu/images/BBblack_lg.jpg "I2C Pin Location")
