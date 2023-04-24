//
// Created by Michael Brookes on 09/04/2016.
//

#ifndef I2CDEVICE_H
#define I2CDEVICE_H

#include "../Device/IDevice.h"
#include "../Exceptions/ExceptionAid.h"

#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/stat.h>

#define I2C_BUS_COUNT   2               //!< How many I2C buses are available. Add paths below as required.
#define I2C_1           "/dev/i2c-1"    //!< File Path for the i2c-1 bus.
#define I2C_2          "/dev/i2c-2"    //!< File Path for the i2c-1 bus.


#define ONE_BYTE        1               //!< Used for specifying how many bytes to read
#define TWO_BYTES       2               //!< Used for specifying how many bytes to write
#define MAX_BUFF        1024            //!< Used to store fatal error.

namespace abI2C {

    using namespace std;

    class I2CDevice : public abIDevice::IDevice {

    public:
        /**
         \fn Public Destructor
         */
        ~I2CDevice( );

        /**
         \fn Public Constructor
         */
        I2CDevice( );

        /*I2C Specific Members*/

        /**
         \fn Public InitDevice
         \brief Requires the device address and bus id to be configured.
         */
        void InitDevice( ) throw( I2CSetupException& );

        /**
         \fn Public GetValueFromRegister
         \param unsigned char _RegisterValue
         \brief Writes the register that you want to read, then performs a read on that register.
         */
        short GetValueFromRegister( unsigned char _RegisterAddress );

        /**
         \fn Public SetRegisterValue
         \param unsigned char _RegisterValue
         \brief Set the value that will next be written to the I2C device.
         */
        void SetRegisterValue( unsigned char _RegisterValue );

        /**
         \fn Public SetRegisterAddress
         \param unsigned char _RegisterAddress
         \brief Set the Register address that the _RegisterValue will be written to.
         */
        void SetRegisterAddress( unsigned char _RegisterAddress );

        /**
         \fn Public WriteToDevice
         \param size_t _BufferSize
         \brief Perform the write request. The _BufferSize is used to differentiate between a read or write request.
         */
        int WriteToDevice( size_t _BufferSize ) throw( I2CSetupException& );

    private:

        /**
         \fn Private Struct I2CBus
         \brief used to store Bus Paths for ease of access.
         */
        struct I2CBus { const char * BusPath; } _Bus[ I2C_BUS_COUNT ];

        /**
         \fn Private Struct stat
         \brief used to check file paths.
         */
        struct stat buffer;

    protected:

        /**
         \fn Protected GetDeviceHandle
         \param none
         \brief Returns the current FileHandle for reading and writing to the I2C device.
         */
        int GetDeviceFileHandle( );

        /**
         \fn Protected GetFilePath
         \param none
         \brief Returns then FilePath for accessing the I2C device.
         */
        const char * GetFilePath( );

        /**
         \fn Protected SetBusPaths
         \param none
         \brief Set Path to all the available buses. As set with I2CBus (struct) and Defines.
         */
        void SetBusPaths( );

        /**
         \fn Protected ValidateBusId
         \param int _BusId
         \brief Make sure the BusId being used is valid.
         */
        int ValidateBusId( ) throw( I2CSetupException& );

        /**
         \fn Protected ValidateBusPath
         \param I2CBus _BusId
         \brief Make sure the BusPath exists.
         */
        char * ValidateBusPath( char * _BusProposedPath ) throw( I2CSetupException& );

        /**
         \fn Protected SelectABusPath
         \param none
         \brief Select which Bus Path we can find your I2C device at.
         */
        void SelectABusPath( );

        /**
         \fn Protected SetDeviceAddress
         \param int _DeviceAddress
         \brief Used to store the device address (Hex)
         */
        virtual void SetDeviceAddress( unsigned char _DeviceAddress ) = 0;

        /**
         \fn Protected SetBusId
         \param int _BusId
         \brief Used to store the bus id (int)
         */
        virtual void SetBusId( int _BusId ) = 0;

        /* From IDevice Interface*/

        /**
         \fn Protected ConnectToDevice
         \param none
         \brief Sets up an IOCTL connection to the I2C device as a Slave for Linux.
         */
        int ConnectToDevice( ) { return ioctl( this->FileHandle, I2C_SLAVE, this->DeviceAddress ); };

        /**
         \fn Protected OpenDevice
         \param none
         \brief Attempt to open the FileHandle.
         */
        int OpenDevice( ) throw( I2CSetupException& );

        /**
         \fn Protected ReadDevice
         \param size_t _BufferSize
         \brief Reads the current buffer from the I2C device - first writes the register address that will be read.
         */
        short ReadDevice( size_t _BufferSize ) throw( I2CSetupException& );

        const char * DeviceBusPath;

        unsigned char DeviceAddress;
        unsigned char RegisterValue;
        unsigned char RegisterAddress;

        char ReadAndWriteBuffer[ TWO_BYTES ];
        char WriteBufferOnly[ ONE_BYTE ];

        char ErrMessage[ MAX_BUFF ];
        int FileHandle;
        int BusId;

        bool DeviceInitialised;

    };
}

#endif //I2CDEVICE_H
