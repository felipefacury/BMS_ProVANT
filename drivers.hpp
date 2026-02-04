#ifndef __DRIVERS_HPP__
#define __DRIVERS_HPP__

#include "definitions.hpp"

#include <Wire.h>


// ----------------------------------------- I2C interface change BEGIN  ------------------------------------------------ //

//Must be changed to the pins in use
#define _SDA_ 0
#define _SCL_ 1

#define I2C0 // If defned the interface I2C-0 will be used

#ifdef I2C0
  #define WIRE Wire
#else
  #define WIRE Wire1
#endif
// ----------------------------------------- I2C interface change BEGIN  ------------------------------------------------ //

class CommDriver {
  private:
    //The bq769x0 without CRC has the address 0x08 (section 5 "Device Comparison Table")
    const uint8_t bqI2CAddress = 0x08; //7-bit I2C address

    /**
    * @brief Write a byte inside a register at the BQ7694
    * 
    * @param regAddress address to the regiter to be written
    * @param regData data to be written inside the register
    *
    * @return Boolean indicating success(1) ou fail(0) 
    */
    byte calculateCRC(byte data_buffer[], byte len);

  public:


    /**
    * @brief Initialize the I2C communication
    * 
    * @param SDA, SCL SDAand SCL pins to be used
    */
    CommDriver(pin_size_t SDA, pin_size_t SCL);

    /**
    * @brief Read a byte from a register at the BQ7694
    * 
    * @param regAddress address to the regiter to be read

    * @return Byte read at the register requested 
    */
    byte registerRead(byte regAddress);

    /**
    * @brief Read 2 bytes from 2 consecutive registers at the BQ7694
    * 
    * @param regAddress address to the first regiter to be read

    * @return Integer with the 2 bytes read already combined 
    */
    uint16_t registerDoubleRead(byte regAddress);

    /**
    * @brief Write a byte inside a register at the BQ7694
    * 
    * @param regAddress address to the regiter to be written
    * @param regData data to be written inside the register
    *
    * @return Boolean indicating success(1) ou fail(0) 
    */
    bool registerWrite(byte regAddress, byte regData);

};

#endif