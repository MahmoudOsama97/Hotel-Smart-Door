#ifndef _I2C_DRIVER_H
#define _I2C_DRIVER_H

#include "tm4c123gh6pm.h"
#include <stdint.h>
//#include "gpio.h"

void I2C_initCommunication();

void I2C_initMaster(uint8_t mode);

void I2C_setSlaveAddress();

void I2C_sendChar(char x);

void I2C_sendString(char* data);

#endif