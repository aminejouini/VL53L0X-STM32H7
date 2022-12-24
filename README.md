# VL53L0X-STM32H7
VL53L0X time of flight sensor library for STM32 microcontrollers

This is a C port of pololu's Arduino library for the ST VL53L0X time of flight sensor based on HAL functions to interfacing this sensor using the I2C protocol. It is targeted at the STM32H7 microcontroller but can be modified to be used with another µC.

Make the following connections between the STM32F103C8T6 and the VL53L0X board:

STM32H7  VL53L0X board
-------   -------------
    3V3 - VIN
    GND - GND
    PB6 - SCL
    PB7 - SDA
    PC0 - XSHUT 
and don't forget the pullup resistors on SDA and SCL. (using CubeMx)
