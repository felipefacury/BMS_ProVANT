#ifndef __BQ76940__HPP__
#define __BQ76940__HPP__

#include "definitions.hpp"
#include "drivers.hpp"

void bq769x0IRQ();

class BQ76940{
  private:
    CommDriver* driver;

    //OV integer value must range 8200-12280
    //UV integer value must range 4096-8176
    const unsigned max_OV_tripValue = 12280;
    const unsigned max_UV_tripValue = 8176;
    const unsigned min_OV_tripValue = 8200;
    const unsigned min_UV_tripValue = 4096;

    float gain = 0; //These are two internal factory set values.
    int offset = 0; //We read them once at boot up and use them in many functions

    float current;

    byte tripCalculator(float tripVoltage);

  public:

    BQ76940(pin_size_t SDA, pin_size_t SCL);

    bool initBQ(byte irqPin);

    int readGAIN(void);
    int readADCoffset(void);

    float readOVtrip(void);
    void writeOVtrip(float tripVoltage);
    float readUVtrip(void);
    void writeUVtrip(float tripVoltage);

    float readCellVoltage(byte cellNumber);

    int thermistorLookup(float resistance);
    int readTemp(byte thermistorNumber);

    float calcVoltage(unsigned rawValue);

    byte readSysStat();
    void resetSysStatBit(byte mask);

    void readCurrent();

    

    float getCurrent();
    byte getOCtrip();
    byte getSCtrip();
    byte getOCdelay();
    byte getSCdelay();


    void setOCtrip(byte threshold);
    void setSCtrip(byte threshold);
    void setOCdelay(byte delay);
    void setSCdelay(byte delay);


    void setCConeshot();

};

#endif