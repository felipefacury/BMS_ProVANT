#ifndef __DEFINITONS_HPP__
#define __DEFINITONS_HPP__

#include "Arduino.h"


const uint16_t shunt = 10; // miliOhm



// ----------------------------------------- DEBUG BEGIN  ------------------------------------------------ //
//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINTLN1(x) (Serial.println(x))
  #define DEBUG_PRINT1(x) (Serial.print(x))
  #define DEBUG_PRINTLN2(x,a) (Serial.println(x,a))
  #define DEBUG_PRINT2(x,a) (Serial.print(x,a))
#else
  #define DEBUG_PRINT1
  #define DEBUG_PRINTLN1
  #define DEBUG_PRINT2
  #define DEBUG_PRINTLN2
#endif
// ----------------------------------------- DEBUG END  ------------------------------------------------ //


// ----------------------------------------- DEFINITIONS BEGIN  ------------------------------------------------ //

 // Register addresses (Can be found in section "8.5 Register Maps" in the IC datasheet)
 
#define bq796x0_SYS_STAT    0x00
#define bq796x0_CELLBAL1    0x01
#define bq796x0_CELLBAL2    0x02
#define bq796x0_CELLBAL3    0x03
#define bq796x0_SYS_CTRL1   0x04
#define bq796x0_SYS_CTRL2   0x05
#define bq796x0_PROTECT1    0x06
#define bq796x0_PROTECT2    0x07
#define bq796x0_PROTECT3    0x8
#define bq796x0_OV_TRIP     0x09
#define bq796x0_UV_TRIP     0x0A
#define bq796x0_CC_CFG      0x0B

//Read-only
#define bq796x0_VC1_HI  0x0C
#define bq796x0_VC1_LO  0x0D
//Other VC registers are done with an offset in software
#define bq796x0_BAT_HI  0x2A
#define bq796x0_BAT_LO  0x2B
#define bq796x0_TS1_HI  0x2C
#define bq796x0_TS1_LO  0x2D
#define bq796x0_TS2_HI  0x2E
#define bq796x0_TS2_LO  0x2F
#define bq796x0_TS3_HI  0x30
#define bq796x0_TS3_LO  0x31
#define bq796x0_CC_HI  0x32
#define bq796x0_CC_LO  0x33
#define bq796x0_ADCGAIN1  0x50
#define bq796x0_ADCOFFSET  0x51
#define bq796x0_ADCGAIN2  0x59

//SYS_STAT bit masks
#define bq796x0_CC_READY  1<<7
#define bq796x0_DEVICE_XREADY 1<<5
#define bq796x0_OVRD_ALERT 1<<4
#define bq796x0_UV 1<<3
#define bq796x0_OV 1<<2
#define bq796x0_SCD 1<<1
#define bq796x0_OCD 1<<0

//SYS_CTRL1 bit masks
#define bq796x0_LOAD_PRESENT 1<<7
#define bq796x0_ADC_EN 1<<4
#define bq796x0_TEMP_SEL 1<<3
#define bq796x0_SHUT_A 1<<1
#define bq796x0_SHUT_B 1<<0

//SYS_CTRL2 bit masks
#define bq796x0_DELAY_DIS 1<<7
#define bq796x0_CC_EN 1<<6
#define bq796x0_CC_ONESHOT 1<<5
#define bq796x0_DSG_ON 1<<1
#define bq796x0_CHG_ON 1<<0

//Bit operations
#define setBit(reg, mask) (reg |= mask)
#define resetBit(reg, mask) (reg &= ~mask)
#define toggleBit(reg, mask) (reg ^= mask)
#define testBit(reg, mask) (reg & mask)

// ----------------------------------------- DEFINITIONS END  ------------------------------------------------ //


// ----------------------------------------- ENUMERATIONS BEGIN  ------------------------------------------------ //

enum class OCdelay : byte {
  OCD_8ms,
  OCD_20ms,
  OCD_40ms,
  OCD_80ms,
  OCD_160ms,
  OCD_320ms,
  OCD_640ms,
  OCD_1280ms
};

inline int operator<<(OCdelay valor, int shift) {
    return static_cast<int>(static_cast<byte>(valor)) << shift;
}

inline uint8_t operator!=(uint8_t val, OCdelay e) {
    return val != static_cast<uint8_t>(e);
}

inline uint8_t operator==(uint8_t val, OCdelay e) {
    return val == static_cast<uint8_t>(e);
}


enum class OCthresh : byte {
  OCT_8mv,
  OCT_11mv,
  OCT_14mv,
  OCT_17mv,
  OCT_19mv,
  OCT_22mv,
  OCT_25mv,
  OCT_28mv,
  OCT_31mv,
  OCT_33mv
};



inline uint8_t& operator|=(uint8_t& val, OCthresh e) {

    val = val | static_cast<uint8_t>(e);

    return val;
}

inline uint8_t operator!=(uint8_t val, OCthresh e) {
    return val != static_cast<uint8_t>(e);
}

inline uint8_t operator==(uint8_t val, OCthresh e) {
    return val == static_cast<uint8_t>(e);
}



enum class SCthresh : byte {
  SCT_22mv,
  SCT_33mv,
  SCT_44mv,
  SCT_56mv,
  SCT_67mv,
  SCT_78mv,
  SCT_89mv,
  SCT_100mv
};

inline uint8_t& operator|=(uint8_t& val, SCthresh e) {

    val = val | static_cast<uint8_t>(e);

    return val;
}

inline uint8_t operator!=(uint8_t val, SCthresh e) {
    return val != static_cast<uint8_t>(e);
}

inline uint8_t operator==(uint8_t val, SCthresh e) {
    return val == static_cast<uint8_t>(e);
}



enum class SCdelay : byte{
  SCD_70us,
  SCD_100us,
  SCD_200us,
  SCD_400us
};

inline int operator<<(SCdelay valor, int shift) {
    return static_cast<int>(static_cast<byte>(valor)) << shift;
}

inline uint8_t operator!=(uint8_t val, SCdelay e) {
    return val != static_cast<uint8_t>(e);
}

inline uint8_t operator==(uint8_t val, SCdelay e) {
    return val == static_cast<uint8_t>(e);
}

enum class OVdelay : byte{
  OVD_1s,
  OVD_2s,
  OVD_4s,
  OVD_8s
};

inline int operator<<(OVdelay valor, int shift) {
    return static_cast<int>(static_cast<byte>(valor)) << shift;
}

inline uint8_t operator!=(uint8_t val, OVdelay e) {
    return val != static_cast<uint8_t>(e);
}

inline uint8_t operator==(uint8_t val, OVdelay e) {
    return val == static_cast<uint8_t>(e);
}

enum class UVdelay : byte{
  UVD_1s,
  UVD_4s,
  UVD_8s,
  UVD_16s
};

inline int operator<<(UVdelay valor, int shift) {
    return static_cast<int>(static_cast<byte>(valor)) << shift;
}

inline uint8_t operator!=(uint8_t val, UVdelay e) {
    return val != static_cast<uint8_t>(e);
}

inline uint8_t operator==(uint8_t val, UVdelay e) {
    return val == static_cast<uint8_t>(e);
}


enum class BQstates : byte{
    OK,
    FAULT
};

// ----------------------------------------- ENUMERATIONS END  ------------------------------------------------ //


//Max number of ms before timeout error. 100 is pretty good
#define MAX_I2C_TIME 100

// ProVant-EMERGENTIA pack has a 12 cell lipo that runs at 48V:
#define NUMBER_OF_CELLS 15 // Although ProVant Emergentia has only 12 cells, all the IC pins are required in order to operate properly. On the readings, ignore cells 4, 9 and 14, they are short circuited.

#define CELLS_IN_USE 12

#define DCG_FET 0
#define CHG_FET 1

#endif