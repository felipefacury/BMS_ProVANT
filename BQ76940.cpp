#include <sys/_types.h>
#include "SerialUSB.h"
#include "Arduino.h"
#include "BQ76940.hpp"

extern volatile boolean bq769x0_IRQ_Triggered;
extern float cellVoltage[NUMBER_OF_CELLS + 1];


byte BQ76940::readSysStat(){
  return driver->registerRead(bq796x0_SYS_STAT);
}

void BQ76940::resetSysStatBit(byte mask){
  driver->registerWrite(bq796x0_SYS_STAT, mask);
}


// ------------------------ init BQ ----------------------------------------------------- // 

BQ76940::BQ76940(pin_size_t SDA, pin_size_t SCL)
{
  driver = new CommDriver(SDA, SCL);
  this->current = 0;
}
  
bool BQ76940::initBQ(byte irqPin, SCthresh sct, SCdelay scd, OCthresh oct, OCdelay ocd, OVdelay ovd, UVdelay uvd, float ovTrip, float uvTrip)
{

  
  // ------------------------------ Test to see if we have correct I2C communication --------------------------------------//

  byte testByte = driver->registerRead(bq796x0_ADCGAIN2); // This reading is used only to check if the read value is plausible, it should be something other than zero on POR.

  for(byte x = 0 ; x < 10 && testByte == 0 ; x++)
  {
    DEBUG_PRINT1(".");
    testByte = driver->registerRead(bq796x0_ADCGAIN2);
    delay(100);
  }
  
  if(testByte == 0x00) return false; //Something is very wrong. Check wiring.



  // ------------------------------ Set CC_CFG to 0x19 --------------------------------------//

  //"For optimal performance, [CC_CFG] should be programmed to 0x19 upon device startup." page 40
  driver->registerWrite(bq796x0_CC_CFG, 0x19); //address, value


  // ------------------------------ Enabling ADC --------------------------------------//

  DEBUG_PRINTLN1("// Testing ADC \\\\");
  //Double check that ADC is enabled
  byte sysVal = driver->registerRead(bq796x0_SYS_CTRL1);
  if(sysVal & bq796x0_ADC_EN)
  {
    DEBUG_PRINTLN1("ADC Already Enabled");
  }
  sysVal |= bq796x0_ADC_EN; //Set the ADC_EN bit
  if (!driver->registerWrite(bq796x0_SYS_CTRL1, sysVal))
    DEBUG_PRINTLN1("Erro ao escrever no registrador do ADC"); //address, value

  DEBUG_PRINT1("\n");


  // ------------------------------ Enabling coulomb counter --------------------------------------//

  DEBUG_PRINTLN1("// Coulomb Counter \\\\");
  //Enable countinous reading of the Coulomb Counter
  sysVal = driver->registerRead(bq796x0_SYS_CTRL2);
  sysVal |= bq796x0_CC_EN; //Set the CC_EN bit
  if (!driver->registerWrite(bq796x0_SYS_CTRL2, sysVal))
    DEBUG_PRINTLN1("Erro para ativar o CC"); //address, value
  sysVal = driver->registerRead(bq796x0_SYS_CTRL2);
  DEBUG_PRINTLN1("Coulomb counter enabled");

  DEBUG_PRINT1("\n");




  // ------------------------------ Configuring interruption --------------------------------------//

  pinMode(irqPin, INPUT); //No pull up
  attachInterrupt(digitalPinToInterrupt(irqPin), bq769x0IRQ, RISING);
  this->irqPin = irqPin;
  

  // ------------------------------ Getting voltage gain and offset for voltage read --------------------------------------//

  DEBUG_PRINTLN1("// Gain and Offset \\\\");
  //Gain and offset are used in multiple functions
  //Read these values into global variables
  gain = readGAIN() / (float)1000; //Gain is in uV so this converts it to mV. Example: 0.370mV/LSB
  offset = readADCoffset(); //Offset is in mV. Example: 65mV

  DEBUG_PRINT1("gain: ");
  DEBUG_PRINT1(gain);
  DEBUG_PRINTLN1("mV/LSB");

  DEBUG_PRINT1("offset: ");
  DEBUG_PRINT1(offset);
  DEBUG_PRINTLN1("mV");

  DEBUG_PRINT1("\n");


  // ------------------------------ Reset system faults --------------------------------------//

  //Read the system status register
  byte sysStat = driver->registerRead(bq796x0_SYS_STAT);
  if(sysStat & bq796x0_DEVICE_XREADY)
  {
    DEBUG_PRINTLN1("Device X Ready Error");
    //Try to clear it
    driver->registerWrite(bq796x0_SYS_STAT, bq796x0_DEVICE_XREADY);

    delay(500);
    //Check again  
    byte sysStat = driver->registerRead(bq796x0_SYS_STAT);
    if(sysStat & bq796x0_DEVICE_XREADY)
    {
      DEBUG_PRINTLN1("Device X Ready Not Cleared");
    }
  }

  //Ressetting SYS_STAT
  driver->registerWrite(bq796x0_SYS_STAT, driver->registerRead(bq796x0_SYS_STAT));

  DEBUG_PRINT1("\n");



  this->setRSNNS(0);


  // ------------------------------ Settin SC trip and delay --------------------------------------//

  DEBUG_PRINTLN1("// SC trip and delay \\\\");

  this->scd = scd;
  this->sct = sct;
  this->setSCdelay(this->scd);
  this->setSCtrip(this->sct);

  DEBUG_PRINT1("SC trip set to: 0x");
  DEBUG_PRINTLN2(this->getSCtrip(), HEX);

  DEBUG_PRINT1("SC delay set to: 0x");
  DEBUG_PRINTLN2(this->getSCdelay(), HEX);

  DEBUG_PRINT1("\n");


  // ------------------------------ Settin SC trip and delay --------------------------------------//

  DEBUG_PRINTLN1("// OC trip and delay \\\\");

  this->ocd = ocd;
  this->oct = oct;
  this->setOCdelay(this->ocd);
  this->setOCtrip(this->oct);

  DEBUG_PRINT1("OC trip set to: 0x");
  DEBUG_PRINTLN2(this->getOCtrip(), HEX);

  DEBUG_PRINT1("OC delay set to: 0x");
  DEBUG_PRINTLN2(this->getOCdelay(), HEX);

  DEBUG_PRINT1("\n");


  // ------------------------------ Settin OV and UV delay --------------------------------------//

  DEBUG_PRINTLN1("// UV and OV delay \\\\");

  this->uvd = uvd;
  this->ovd = ovd;
  this->setOVdelay(this->ovd);
  this->setUVdelay(this->uvd);

  DEBUG_PRINT1("OV delay set to: 0x");
  DEBUG_PRINTLN2(this->getOVdelay(), HEX);

  DEBUG_PRINT1("UV delay set to: 0x");
  DEBUG_PRINTLN2(this->getUVdelay(), HEX);

  DEBUG_PRINT1("\n");

  // ------------------------------ Settin OV and UV trip --------------------------------------//  

  DEBUG_PRINTLN1("// Trip voltages \\\\");
  //Set any other settings such as OVTrip and UVTrip limits
  float under = readUVtrip();
  float over = readOVtrip();

  DEBUG_PRINT1("Undervoltage trip: ");
  DEBUG_PRINT1(under);
  DEBUG_PRINTLN1("V");

  DEBUG_PRINT1("Overvoltage trip: ");
  DEBUG_PRINT1(over);
  DEBUG_PRINTLN1("V");

  if(under != uvTrip)
  {
    writeUVtrip(uvTrip); //Set undervoltage to 3.32V
    this->UV_trip = uvTrip; 
    DEBUG_PRINT1("New undervoltage trip: ");
    DEBUG_PRINT1(readUVtrip());
    DEBUG_PRINTLN1("V"); //should print 3.32V
  }

  if(over != ovTrip)
  {
    writeOVtrip(ovTrip); //Set overvoltage to 4.27V
    this->OV_trip = ovTrip;
    DEBUG_PRINT1("New overvoltage trip: ");
    DEBUG_PRINT1(readOVtrip());
    DEBUG_PRINTLN1("V"); //should print 4.27V
  }


  DEBUG_PRINT1("\n");
  return true;
}

//-------------------------------------------------------- init bq function end ----------------------------------------- //


//-------------------------------------------------------- deal interruption begin ----------------------------------------- //

// void BQ76940::dealInterruption()
// {
//   if (!bq769x0_IRQ_Triggered) return;

//   bq769x0_IRQ_Triggered = false;

//   byte SS = driver->registerRead(bq796x0_SYS_STAT);

//   if (testBit(SS, bq796x0_DEVICE_XREADY)){

//     this->initBQ(this->irqPin);
//     //this->resetSysStatBit(bq796x0_DEVICE_XREADY);
//     Serial.println("DEVICE_XREADY_FAULT");
//   }

//   if (testBit(SS, bq796x0_CC_READY)){
//     this->readCurrent();
//     //this->resetSysStatBit(bq796x0_CC_READY);
//     Serial.println("CC_READY");
//   }

//   if (testBit(SS, bq796x0_UV)){

//     Serial.println("UV_FAULT");
//   }

//   if (testBit(SS, bq796x0_OV)){

//     Serial.println("OV_FAULT");
//   }
  
//   if (testBit(SS, bq796x0_SCD)){
    
//     Serial.println("SHORTCUT_FAULT");
//   }

//   if (testBit(SS, bq796x0_OCD)){

//     Serial.println("OC_FAULT");
//   }

//   driver->registerWrite(bq796x0_SYS_STAT, SS);

//   if (digitalRead(this->irqPin)) 
//     bq769x0_IRQ_Triggered = true;

// }

//-------------------------------------------------------- deal interruption end ----------------------------------------- //


//-------------------------------------------------------- read gain function  ----------------------------------------- //
int BQ76940::readGAIN(void)
{
  byte val1 = driver->registerRead(bq796x0_ADCGAIN1);
  byte val2 = driver->registerRead(bq796x0_ADCGAIN2);
  val1 &= 0b00001100; //There are some unknown reservred bits around val1 that need to be cleared

  //Recombine the bits into one ADCGAIN
  byte adcGain = (val1 << 1) | (val2 >> 5);

  int gain = 365 + adcGain;

  return(gain);
}

//-------------------------------------------------------- read gain function end ----------------------------------------- //



//-------------------------------------------------------- read ADC offset function  ----------------------------------------- //
//Returns the factory trimmed ADC offset
//Offset is -127 to 128 in mV
int BQ76940::readADCoffset(void)
{
  //Here we need to convert a 8bit 2's compliment to a 16 bit int
  char offset = driver->registerRead(bq796x0_ADCOFFSET);

  return((int)offset); //8 bit char is now a full 16-bit int. Easier math later on.
}
//-------------------------------------------------------- read ADC offset function end ----------------------------------------- //




//-------------------------------------------------------- read Overvoltage function  ----------------------------------------- //
//Returns the over voltage trip threshold
//Default is 0b.10.OVTRIP(0xAC).1000 = 0b.10.1010.1100.1000 = 0x2AC8 = 10,952
//OverVoltage = (OV_TRIP * GAIN) + ADCOFFSET
//Gain and Offset is different for each IC
//Example: voltage = (10,952 * 0.370) + 56mV = 4.108V
float BQ76940::readOVtrip(void)
{
  int trip = driver->registerRead(bq796x0_OV_TRIP);

  trip <<= 4; //Shuffle the bits to align to 0b.10.XXXX.XXXX.1000
  trip |= 0x2008;

  float overVoltage = ((float)trip * gain) + offset;
  overVoltage /= 1000; //Convert to volts

  //DEBUG_PRINT("overVoltage should be around 4.108: ");
  //DEBUG_PRINTLN(overVoltage, 3);

  return(overVoltage);
}


//-------------------------------------------------------- read Overvoltage function end  ----------------------------------------- //







//-------------------------------------------------------- write Overvoltage limit value function  ----------------------------------------- //

//Given a voltage (4.22 for example), set the over voltage trip register
//Example: voltage = 4.2V = (4200mV - 56mV) / 0.370mv = 11,200
//11,200 = 0x2BC0 = 
void BQ76940::writeOVtrip(float tripVoltage)
{

  if ((tripVoltage < (calcVoltage(min_OV_tripValue)/1000)) || (tripVoltage > (calcVoltage(max_OV_tripValue)/1000))){

    return;
  }

  byte val = tripCalculator(tripVoltage); //Convert voltage to an 8-bit middle value
  driver->registerWrite(bq796x0_OV_TRIP, val); //address, value
}

//-------------------------------------------------------- write Overvoltage limit value function end  ----------------------------------------- //






//-------------------------------------------------------- read Undervoltage function  ----------------------------------------- //

//Returns the under voltage trip threshold
//Default is 0b.01.UVTRIP(0x97).0000 = 0x1970 = 6,512
//UnderVoltage = (UV_TRIP * GAIN) + ADCOFFSET
//Gain and Offset is different for each IC
//Example: voltage = (6,512 * 0.370) + 56mV = 2.465V
float BQ76940::readUVtrip(void)
{
  byte reg = driver->registerRead(bq796x0_UV_TRIP);

  uint16_t trip = (uint16_t)reg;
  trip <<= 4; //Shuffle the bits to align to 0b.01.XXXX.XXXX.0000
  trip |= 0x1000;

  float underVoltage = ((float)trip * gain) + offset;
  underVoltage /= 1000; //Convert to volts

  //DEBUG_PRINT("underVoltage should be around 2.465: ");
  //DEBUG_PRINTLN(underVoltage, 3);

  return(underVoltage);
}


//-------------------------------------------------------- read Undervoltage function end  ----------------------------------------- //



//-------------------------------------------------------- write Undervoltage limit value function  ----------------------------------------- //

//Given a voltage (2.85V for example), set the under voltage trip register
void BQ76940::writeUVtrip(float tripVoltage)
{
  if ((tripVoltage < (calcVoltage(min_UV_tripValue)/1000)) || (tripVoltage > (calcVoltage(max_UV_tripValue)/1000)))
    return;

  byte val = tripCalculator(tripVoltage); //Convert voltage to an 8-bit middle value
  driver->registerWrite(bq796x0_UV_TRIP, val); //address, value
}

//-------------------------------------------------------- write Undervoltage limit value function end  ----------------------------------------- //







//-------------------------------------------------------- Voltage reading convertion function ----------------------------------------------------------- //

//Under voltage and over voltage use the same rules for calculating the 8-bit value
//Given a voltage this function uses gain and offset to get a 14 bit value
//Then strips that value down to the middle-ish 8-bits
//No registers are written, that's up to the caller
//OV integer value must range 8200-12280
//UV integer value must range 4096-8176
byte BQ76940::tripCalculator(float tripVoltage)
{

  tripVoltage *= 1000; //Convert volts to mV

  //DEBUG_PRINT("tripVoltage to be: ");
  //DEBUG_PRINTLN(tripVoltage, 3);

  tripVoltage -= offset;
  tripVoltage /= gain;


  int tripValue = (int)tripVoltage; //We only want the integer - drop decimal portion.


  //DEBUG_PRINT("tripValue should be something like 0x2BC0: ");
  //DEBUG_PRINTLN(tripValue, HEX);

  tripValue >>= 4; //Cut off lower 4 bits
  tripValue &= 0x00FF; //Cut off higher bits

  //DEBUG_PRINT("About to report tripValue: ");
  //DEBUG_PRINTLN(tripValue, HEX);


  return(tripValue);
}

//-------------------------------------------------------- Voltage reading convertion function end ----------------------------------------------------------- //


//-------------------------------------------------------- Calculate Voltage (aux function)  ----------------------------------------------------------- //
//Return the real voltage in mv
inline float BQ76940::calcVoltage(unsigned rawValue)
{
  return gain*rawValue+offset;
}

//-------------------------------------------------------- Calculate Voltage (aux function)  ----------------------------------------------------------- //

//-------------------------------------------------------- Read the voltage of a cell function begin ---------------------------------------------------------- //

float BQ76940::readCellVoltage(byte cellNumber)
{
  if(cellNumber < 1 || cellNumber > 15) return(-1); //Return error

  DEBUG_PRINT1("Read cell number: ");
  DEBUG_PRINTLN1(cellNumber);

  //Reduce the caller's cell number by one so that we get register alignment
  cellNumber--;

  byte registerNumber = bq796x0_VC1_HI + (cellNumber * 2);

  DEBUG_PRINT1("register: 0x");
  DEBUG_PRINTLN2(registerNumber, HEX);

  int cellValue = (int)driver->registerDoubleRead(registerNumber);

  //int cellValue = 0x1800; //6,144 - Should return 2.365
  //int cellValue = 0x1F10l; //Should return 3.052

  //Cell value should now contain a 14 bit value

  DEBUG_PRINT1("Cell value (dec): ");
  DEBUG_PRINTLN1(cellValue);
  
  if(cellValue == 0) return(0);

  float cellVoltage = cellValue * gain + offset; //0x1800 * 0.37 + 60 = 3,397mV
  cellVoltage /= (float)1000;

  DEBUG_PRINT1("Cell voltage: ");
  DEBUG_PRINTLN2(cellVoltage, 3);

  return(cellVoltage);
}


//-------------------------------------------------------- Read the voltage of a cell function end ---------------------------------------------------------- //



//-------------------------------------------------------- Read the temperature function begin ---------------------------------------------------------- //

int BQ76940::readTemp(byte thermistorNumber) //Não está lendo os registradores corretos para os termistores 2 e 3, verificar por que. Para o transistor 1 funciona bem. 
{

  if(thermistorNumber > 3) return -1; // Invalid input 

  byte sysValue = driver->registerRead(bq796x0_SYS_CTRL1);

  if(thermistorNumber > 0)
  {
    if((sysValue & bq796x0_TEMP_SEL) == 0)
    {
      sysValue |= bq796x0_TEMP_SEL;
      driver->registerWrite(bq796x0_SYS_CTRL1, sysValue);
      DEBUG_PRINTLN1("Waiting 2 seconds to switch thermistors");
      delay(2000);      
    }

    int registerNumber = 0;
    
    if(thermistorNumber == 0){
      int registerNumber = bq796x0_TS1_HI; 
    }
    else if(thermistorNumber == 1){
      int registerNumber = bq796x0_TS2_HI; 
    }
    else if(thermistorNumber == 2){
      int registerNumber = bq796x0_TS3_HI;
    }

    DEBUG_PRINT1("Este é o meu thermistorrnumber");
    DEBUG_PRINTLN1(thermistorNumber);
    DEBUG_PRINT1("Este é o meu registernumber");
    DEBUG_PRINTLN1(registerNumber);
    int thermValue = (int)driver->registerDoubleRead(registerNumber);

    float thermVoltage = thermValue * (float)382;
    thermVoltage /= (float)1000000;

    float thermResistance = ((float)10000 * thermVoltage) / (3.3 - thermVoltage);
    int temperatureC = thermistorLookup(thermResistance);

    return temperatureC;
  }
  else if(thermistorNumber == 0)
  {
    if((sysValue & (1<<3)) != 0)
    {
      sysValue &= ~(1<<3);
      driver->registerWrite(bq796x0_SYS_CTRL1, sysValue);
      DEBUG_PRINTLN1("Waiting 2 seconds to switch to internal die thermistors");
      delay(2000);      
    }

    int thermValue = (int)driver->registerDoubleRead(bq796x0_TS1_HI);
    float thermVoltage = thermValue * (float)382;
    thermVoltage /= (float)1000000;

    float temperatureC = 25.0 - ((thermVoltage - 1.2) / 0.0042);

    return (int)temperatureC;
  }

  // Safety catch-all:
  return -1;
}


int BQ76940::thermistorLookup(float resistance)
{
  //Resistance is coming in as Ohms, this lookup table assume kOhm
  resistance /= 1000; //Convert to kOhm

  int temp = 0;

  if(resistance > 329.5) temp = -50;
  if(resistance > 247.7) temp = -45;
  if(resistance > 188.5) temp = -40;
  if(resistance > 144.1) temp = -35;
  if(resistance > 111.3) temp = -30;
  if(resistance > 86.43) temp = -25;
  if(resistance > 67.77) temp = -20;
  if(resistance > 53.41) temp = -15;
  if(resistance > 42.47) temp = -10;
  if(resistance > 33.90) temp = -5;
  if(resistance > 27.28) temp = 0;
  if(resistance > 22.05) temp = 5;
  if(resistance > 17.96) temp = 10;
  if(resistance > 14.69) temp = 15;
  if(resistance > 12.09) temp = 20;
  if(resistance > 10.00) temp = 25;
  if(resistance > 8.313) temp = 30;

  return(temp);  
}

//-------------------------------------------------------- Read the temperature function end  ---------------------------------------------------------- //



//-------------------------------------------------------- Read the current begin  ---------------------------------------------------------- //

void BQ76940::setCConeshot()
{
  byte buff = driver->registerRead(bq796x0_SYS_CTRL2);

  resetBit(buff, bq796x0_CC_EN);
  setBit(buff, bq796x0_CC_ONESHOT);

  driver->registerWrite(bq796x0_SYS_CTRL2, buff);
}


void BQ76940::readCurrent()
{
  //if (!bq769x0_IRQ_Triggered) return;

  //if (!testBit(driver->registerRead(bq796x0_SYS_STAT), bq796x0_CC_READY)) return;

  //this->resetSysStatBit(bq796x0_CC_READY);

  // if (!driver->registerRead(bq796x0_SYS_STAT))
  //   bq769x0_IRQ_Triggered = false;

  int16_t rawVoltage = (int16_t)driver->registerDoubleRead(bq796x0_CC_HI);
  Serial.print("Raw voltage: ");
  Serial.println(rawVoltage, HEX);

  float shuntVoltage = rawVoltage * ((float)8.44/1000); // uV -> mV
  Serial.print("Shunt voltage: ");
  Serial.println(shuntVoltage);


  this->current = shuntVoltage/(shunt/(float)1000); //mA
  Serial.print("Current: ");
  Serial.println(this->current);
}

//-------------------------------------------------------- Read the current begin  ---------------------------------------------------------- //


//-------------------------------------------------------- get methods begin  ---------------------------------------------------------- //

float BQ76940::getCurrent()
{
  return this->current;
}

byte BQ76940::getOCtrip()
{
  return 0x0F & driver->registerRead(bq796x0_PROTECT2); // Return only the 4 last bits
}

byte BQ76940::getSCtrip()
{
  return 0x07 & driver->registerRead(bq796x0_PROTECT1); // Return only the last 3 last bits
}

byte  BQ76940::getOCdelay()
{
  return 0x07 & (driver->registerRead(bq796x0_PROTECT2) >> 4); // Return bits 6, 5 and 4 
}

byte BQ76940::getSCdelay()
{
  return 0x07 & (driver->registerRead(bq796x0_PROTECT1) >> 3); // Return bits 5, 4 and 3 
}

float BQ76940::getBatteryPack()
{
  return 4 * this->gain * driver->registerDoubleRead(bq796x0_BAT_HI) + (this->offset * CELLS_IN_USE);
}

byte BQ76940::getOVdelay()
{
  return (0x30 & driver->registerRead(bq796x0_PROTECT3)) >> 4; // Return bits 5 and 4 from the register
}

byte BQ76940::getUVdelay()
{
  return driver->registerRead(bq796x0_PROTECT3) >> 6; // Return bits 7 and 6 from the register
}


float BQ76940::getSCtripCurrent(SCthresh thr)
{

  float ret = 0.0;
  switch (thr) {

  case SCthresh::SCT_22mv:
    ret = (float)22/shunt;
    break;

  case SCthresh::SCT_33mv:
    ret = (float)33/shunt;
    break;

  case SCthresh::SCT_44mv:
    ret = (float)44/shunt;
    break;

  case SCthresh::SCT_56mv:
    ret = (float)56/shunt;
    break;

  case SCthresh::SCT_67mv:
    ret = (float)67/shunt;
    break;

  case SCthresh::SCT_78mv:
    ret = (float)78/shunt;
    break;

  case SCthresh::SCT_89mv:
    ret = (float)89/shunt;
    break;

  case SCthresh::SCT_100mv:
    ret = (float)100/shunt;
    break;
  }
  
  if (this->getRSNNS())
    return 2*ret;
  else
    return ret;
}

float BQ76940::getOCtripCurrent(OCthresh thr)
{
  float ret = 0.0;
  switch (thr) {

  case OCthresh::OCT_8mv:
    ret = (float)8/shunt;
    break;

  case OCthresh::OCT_11mv:
    ret = (float)11/shunt;
    break;

  case OCthresh::OCT_14mv:
    ret = (float)14/shunt;
    break;

  case OCthresh::OCT_17mv:
    ret = (float)17/shunt;
    break;

  case OCthresh::OCT_19mv:
    ret = (float)19/shunt;
    break;

  case OCthresh::OCT_22mv:
    ret = (float)22/shunt;
    break;

  case OCthresh::OCT_25mv:
    ret = (float)25/shunt;
    break;

  case OCthresh::OCT_28mv:
    ret = (float)28/shunt;
    break;

  case OCthresh::OCT_31mv:
    ret = (float)31/shunt;
    break;

  case OCthresh::OCT_33mv:
    ret = (float)33/shunt;
    break;
  }
  
  if (this->getRSNNS())
    return 2*ret;
  else
    return ret;
}

byte BQ76940::getRSNNS()
{
  return this->RSNNS;
}

uint8_t BQ76940::getLowerCell()
{
  uint8_t lowerCell = cellVoltage[1];
  for(int i = 1 ; i < NUMBER_OF_CELLS+1 ; i++){
      if (cellVoltage[i] < 1)
        continue;
      if (cellVoltage[i] < lowerCell)
        lowerCell = cellVoltage[i];
    }
  return lowerCell;
}

uint8_t BQ76940::getHigherCell()
{
  uint8_t higherCell = cellVoltage[1];
  for(int i = 1 ; i < NUMBER_OF_CELLS+1 ; i++){
      if (cellVoltage[i] < 1)
        continue;
      if (cellVoltage[i] > higherCell)
        higherCell = cellVoltage[i];
    }
  return higherCell;
}
//-------------------------------------------------------- get methods end  ---------------------------------------------------------- //


//-------------------------------------------------------- set methods begin  ---------------------------------------------------------- //

void BQ76940::setOCtrip(OCthresh threshold)
{
  byte buff = driver->registerRead(bq796x0_PROTECT2);

  buff &= 0xF0;
  buff |= threshold;

  driver->registerWrite(bq796x0_PROTECT2, buff);
}

void BQ76940::setSCtrip(SCthresh threshold)
{
  byte buff = driver->registerRead(bq796x0_PROTECT1);

  buff &= 0xF8;
  buff |= threshold;

  driver->registerWrite(bq796x0_PROTECT1, buff);
}

void BQ76940::setOCdelay(OCdelay delay)
{
  byte buff = driver->registerRead(bq796x0_PROTECT2);

  buff &= 0x8F;

  buff |= (delay << 4);

  driver->registerWrite(bq796x0_PROTECT2, buff);
}

void BQ76940::setSCdelay(SCdelay delay)
{
  byte buff = driver->registerRead(bq796x0_PROTECT1);

  buff &= 0xE7;

  buff |= (delay << 3);

  driver->registerWrite(bq796x0_PROTECT1, buff);
}

void BQ76940::setOVdelay(OVdelay delay)
{
  byte buff = driver->registerRead(bq796x0_PROTECT3);

  buff &= 0xCF;

  buff |= (delay << 4);

  driver->registerWrite(bq796x0_PROTECT3, buff);
}

void BQ76940::setUVdelay(UVdelay delay)
{
  byte buff = driver->registerRead(bq796x0_PROTECT3);

  buff &= 0x3F;

  buff |= (delay << 6);

  driver->registerWrite(bq796x0_PROTECT3, buff);
}

void BQ76940::setRSNNS(bool value)
{
  byte buff = driver->registerRead(bq796x0_PROTECT1);

  if (value){
    setBit(buff, 1<<7);
    this->RSNNS = 1;
  } else {
    this->RSNNS = 0;
    resetBit(buff, 1 <<7);
  }

  driver->registerWrite(bq796x0_PROTECT1, buff);
}
//-------------------------------------------------------- set methods end  ---------------------------------------------------------- //

//-------------------------------------------------------- interrupt function  ----------------------------------------------------------- //

// this is irq handler for bq769x0 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void bq769x0IRQ()
{
  bq769x0_IRQ_Triggered = true;
}
//-------------------------------------------------------- interrupt function end ----------------------------------------------------------- //
