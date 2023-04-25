//
// Created by Michael Brookes on 19/04/2016.
//

#ifndef I2C_EXCEPTION_AID_H
#define I2C_EXCEPTION_AID_H

#include <exception>
#include <iostream>
#include <string>

namespace abI2C {
    using namespace std;

    class I2CSetupException : public exception {
    public:
        I2CSetupException( string errMessage ):errMessage_(errMessage){}
        const char* what() const throw() { return errMessage_.c_str( ); }

    private:
        string errMessage_;
    };
}

#endif //I2C_EXCEPTION_AID_H
