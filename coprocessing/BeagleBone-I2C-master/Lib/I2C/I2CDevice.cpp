//
// Created by Michael Brookes on 14/04/2016.
//

#include "./I2CDevice.h"


namespace abI2C {

    I2CDevice::I2CDevice( ) {
 
        this->DeviceInitialised = false;
    }


    I2CDevice::~I2CDevice( ) { close( this->FileHandle ); }
    

    void I2CDevice::InitDevice( ) throw( I2CSetupException& ) {
        if(!this->DeviceAddress) throw I2CSetupException( "I2C Device Not Configured ( try : 'obj->SetDeviceAddress([hex address])' )" );
        if(!this->BusId) throw I2CSetupException( "I2C Device Not Configured ( try : 'obj->SetBusId([bus number])' )" );
        /*
         * ** ## -- Setup Stage -- ## ** *
         * SetBusPaths : Saves the file paths to the available buses for ease of access.
         */
        this->SetBusPaths( );

        /*
         * ** ## -- Assignment Stage ( based on args ) -- ## ** *
         * ValidateBusId : Make sure we have a valid bus ID before proceeding.
         * SelectABusPath : Used to specify which bus your I2C device is on.
         * SetDeviceAddress: Hex value for your specific I2C Device.
         */
        this->ValidateBusId( );
        this->SelectABusPath( );

        /*
         * ** ## -- Init Stage -- ## ** *
         * OpenDevice : Creates a file handle for the device, should it be closed? Probably... :)
         * ConnectToDevice : Assigns the device as an I2C Slave and checks availability using IOCTL
         *
         * More info on IOCTL : http://man7.org/linux/man-pages/man2/ioctl.2.html
         */
        this->OpenDevice( );
        this->ConnectToDevice( );

        this->DeviceInitialised = true;
    }

    void I2CDevice::SetBusPaths( ) {
        this->_Bus[ 2 ].BusPath = this->ValidateBusPath( (char *)I2C_2 );
    }

    void I2CDevice::SelectABusPath( ) { this->DeviceBusPath = _Bus[ this->BusId ].BusPath; }

    void I2CDevice::SetRegisterAddress( unsigned char _RegisterAddress ) { this->RegisterAddress = _RegisterAddress; }

    void I2CDevice::SetRegisterValue( unsigned char _RegisterValue ){ this->RegisterValue = _RegisterValue; }

    const char * I2CDevice::GetFilePath( ) { return this->DeviceBusPath; }

    int I2CDevice::GetDeviceFileHandle( ) { return this->FileHandle; }

    int I2CDevice::ValidateBusId( ) throw( I2CSetupException& ) {
        if( this->BusId > I2C_BUS_COUNT || this->BusId < 1 ) {
            snprintf( this->ErrMessage, sizeof( this->ErrMessage ), "Bus ID : %d  is not a valid BUS for this device.", this->BusId );
            throw( I2CSetupException( this->ErrMessage ) );
        }
        else
            return EXIT_SUCCESS;
    }

    char * I2CDevice::ValidateBusPath( char * _I2CBusProposedPath ) throw( I2CSetupException& ) {
        if( stat ( _I2CBusProposedPath, &buffer) == 0 )
            return _I2CBusProposedPath;
        else{
            snprintf( this->ErrMessage, sizeof( this->ErrMessage ), "Fatal I2C Error - Unable to locate the I2C Bus file : %s", _I2CBusProposedPath );
            throw I2CSetupException( this->ErrMessage );
        }
    }

    short I2CDevice::GetValueFromRegister( unsigned char _RegisterAddress ) {
        if(!this->DeviceInitialised) throw I2CSetupException( "I2C Device Not Initialised ( try : 'obj->InitDevice()' )" );
        this->SetRegisterAddress( _RegisterAddress );
        this->WriteBufferOnly[ 0 ] = this->RegisterAddress;
        if( write( this->GetDeviceFileHandle( ), this->WriteBufferOnly, 1 ) == 1 ) {
            return this->ReadDevice( ONE_BYTE );
        }
        else {
            snprintf( this->ErrMessage, sizeof( this->ErrMessage ), "Fatal I2C Error - Unable to write to file : %s", this->GetFilePath( ));
            throw I2CSetupException( this->ErrMessage );
        }
    }

    short I2CDevice::ReadDevice( size_t _BufferSize ) throw( I2CSetupException& ) {
        if(!this->DeviceInitialised) throw I2CSetupException( "I2C Device Not Initialised ( try : 'obj->InitDevice()' )" );
        unsigned char buff[ _BufferSize ];
        if( read( this->GetDeviceFileHandle( ), buff, _BufferSize ) != _BufferSize ) {
            snprintf( this->ErrMessage, sizeof( this->ErrMessage ), "Fatal I2C Error - Unable to read from file : %s", this->GetFilePath( ) );
            throw I2CSetupException( this->ErrMessage );
        }
        else
            return buff[ 0 ];
    }

    int I2CDevice::OpenDevice( ) throw( I2CSetupException& ) {
        this->FileHandle = open( this->GetFilePath( ), O_RDWR );
        if( this->FileHandle == 0 ) {
            snprintf( this->ErrMessage, sizeof( this->ErrMessage ), "Fatal I2C Error - Unable to open file : %s", this->GetFilePath( ) );
            throw I2CSetupException( this->ErrMessage );
        }
        return this->FileHandle;
    }

    int I2CDevice::WriteToDevice(size_t _BufferSize) throw(I2CSetupException &) {
    if (!this->DeviceInitialised)
        throw I2CSetupException("I2C Device Not Initialised ( try : 'obj->InitDevice()' )");

    try {
        if (_BufferSize > ONE_BYTE) {
            this->ReadAndWriteBuffer[0] = this->RegisterAddress;
            this->ReadAndWriteBuffer[1] = this->RegisterValue;
            if (write(this->GetDeviceFileHandle(), this->ReadAndWriteBuffer, _BufferSize) != _BufferSize) {
                snprintf(this->ErrMessage, sizeof(this->ErrMessage),
                         "Fatal I2C Error - Unable to write to file : %s (Register Address: 0x%02X, Register Value: 0x%02X)",
                         this->GetFilePath(), this->RegisterAddress, this->RegisterValue);
                throw I2CSetupException(this->ErrMessage);
            }
        } else {
            this->WriteBufferOnly[0] = this->RegisterAddress;
            if (write(this->GetDeviceFileHandle(), this->WriteBufferOnly, _BufferSize) != _BufferSize) {
                snprintf(this->ErrMessage, sizeof(this->ErrMessage),
                         "Fatal I2C Error - Unable to write to file : %s (Register Address: 0x%02X)",
                         this->GetFilePath(), this->RegisterAddress);
                throw I2CSetupException(this->ErrMessage);
            }
        }
    } catch (exception &e) {
        snprintf(this->ErrMessage, sizeof(this->ErrMessage),
                 "Fatal I2C Error - Unable to write to file : %s (Exception: %s)",
                 this->GetFilePath(), e.what());
        throw I2CSetupException(this->ErrMessage);
    }

    return EXIT_SUCCESS;
}


}