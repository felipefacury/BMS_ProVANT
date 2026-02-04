#ifndef __BQ76940__HPP__
#define __BQ76940__HPP__

#include "definitions.hpp"
#include "drivers.hpp"

void bq769x0IRQ();

class BQ76940{
  private:
    CommDriver* driver; //I2C communication driver

    //OV integer value must range 8200-12280
    //UV integer value must range 4096-8176
    const unsigned max_OV_tripValue = 12280;
    const unsigned max_UV_tripValue = 8176;
    const unsigned min_OV_tripValue = 8200;
    const unsigned min_UV_tripValue = 4096;

    // Normaly G = 0.38 mV/LSB and O = 42 mV
    float gain = 0; //These are two internal factory set values.
    int offset = 0; //We read them once at boot up and use them in many functions

    byte irqPin; //Pint to use interrupt

    float current; //Current given by the coulomb counter

    OCdelay ocd; //Delay before Overcurrent warning
    SCdelay scd; //Delay before Shortcut warning

    OCthresh oct; //Threshold before Overcurrent cut
    SCthresh sct; //Threshold before Shortcut cut

    UVdelay uvd; //Delay before Undervoltage warning
    OVdelay ovd; //Delay before Overvoltage warning

    float OV_trip; //Trheshold before Overvoltage cut
    float UV_trip; //Trheshold before Undervoltage cut

    byte RSNNS = 0;

    byte tripCalculator(float tripVoltage);

    float getSCtripCurrent(SCthresh thr);
    float getOCtripCurrent(OCthresh thr);

    uint8_t getLowerCell();
    uint8_t getHigherCell();

  public:

    BQ76940(pin_size_t SDA, pin_size_t SCL);

    bool initBQ(byte irqPin, SCthresh sct, SCdelay scd, OCthresh oct, OCdelay ocd, OVdelay ovd, UVdelay uvd, float ovTrip, float uvTrip);

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

    
    void dealInterruption();
    

    float getCurrent();
    byte getOCtrip();
    byte getSCtrip();
    byte getOCdelay();
    byte getSCdelay();
    byte getOVdelay();
    byte getUVdelay();
    byte getRSNNS();
    float getBatteryPack();


    void setOCtrip(OCthresh threshold);
    void setSCtrip(SCthresh threshold);
    void setOCdelay(OCdelay delay);
    void setSCdelay(SCdelay delay);
    void setOVdelay(OVdelay delay);
    void setUVdelay(UVdelay delay);
    void setRSNNS(bool value);


    void test(){

      uint8_t c = this->getLowerCell();
      Serial.print("Lower cell voltage ");
      Serial.println(c);
      Serial.print("Cell voltage ");
      Serial.println(this->readCellVoltage(c));

      c = this->getHigherCell();
      Serial.print("Higher cell voltage ");
      Serial.println(c);
      Serial.print("Cell voltage ");
      Serial.println(this->readCellVoltage(c));
    }

    void setCConeshot();

};

#endif