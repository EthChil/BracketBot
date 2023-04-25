//
// Created by Michael Brookes on 09/04/2016.
//

#ifndef IDEVICE_H
#define IDEVICE_H

#include <cstdio>

namespace abIDevice {
    /*
     /brief Interface class for all devices
     */
    class IDevice {
    public:
        virtual ~IDevice( ){ };

        /*
        /fn pure virtual OpenDevice
        /brief All devices implementing this interface will require this functionality.
        */
        virtual void InitDevice( ) = 0;

        /*
        /fn pure virtual OpenDevice
        /brief All devices implementing this interface will require this functionality.
        */
        virtual int OpenDevice( ) = 0;

        /*
        /fn pure virtual WriteToDevice
        /brief All devices implementing this interface will require this functionality.
        */
        virtual int WriteToDevice( size_t _BufferSize ) = 0;

        /*
        /fn pure virtual ReadDevice
        /brief All devices implementing this interface will require this functionality.
        */
        virtual short ReadDevice( size_t _BufferSize ) = 0;

        /*
        /fn pure virtual ConnectToDevice
        /brief All devices implementing this interface will require this functionality.
        */
        virtual int ConnectToDevice( ) = 0;
    protected:
    };
}
#endif //IDEVICE_H