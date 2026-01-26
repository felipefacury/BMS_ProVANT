/*This is the BQ76940 IC code. The purpose of this code is to communicate with the BQ76940 IC via I2C, while explaining, throughout the code, from which part of the datasheet (https://www.ti.com/lit/ds/symlink/bq76940.pdf?ts=1745214968047) 
 * the information was taken. The code was adapted from (https://github.com/nseidle/BMS/blob/master/firmware/SparkFun_bq769x0/SparkFun_bq769x0.ino#L33)
 * Keep in mind that the code is developed thinking of the IC configuration for 12 battery cells, as described in the ProVANT-EMERGENTIA UAV project. 
 *
 *
 * Code by: Henrique de Sá
 *
 *
 */


// ----------------------------------------- INCLUDES BEGIN  ------------------------------------------------ //
#include <Wire.h>
// ----------------------------------------- INCLUDES END  ------------------------------------------------ //


#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINTLN Serial.println
  #define DEBUG_PRINT Serial.print
#else
  #define DEBUG_PRINT
  #define DEBUG_PRINTLN
#endif



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

// ----------------------------------------- DEFINITIONS END  ------------------------------------------------ //

// ----------------------------------------- FUNCTIONS PROTOTYPES ------------------------------------------------ //

boolean initBQ(byte irqPin);

byte registerRead(byte regAddress);

bool registerWrite(byte regAddress, byte regData);
int registerDoubleRead(byte regAddress);
byte calculateCRC(byte data_buffer[], byte len);

int readGAIN(void);
int readADCoffset(void);

byte tripCalculator(float tripVoltage);
float readOVtrip(void);
void writeOVtrip(float tripVoltage);
float readUVtrip(void);
void writeUVtrip(float tripVoltage);

float readCellVoltage(byte cellNumber);

int thermistorLookup(float resistance);
int readTemp(byte thermistorNumber);

float calcVoltage(unsigned rawValue);

void bq769x0IRQ();
// ----------------------------------------- FUNCTIONS PROTOTYPES END  ------------------------------------------------ //

//OV integer value must range 8200-12280
//UV integer value must range 4096-8176
const unsigned max_OV_tripValue = 12280;
const unsigned max_UV_tripValue = 8176;
const unsigned min_OV_tripValue = 8200;
const unsigned min_UV_tripValue = 4096;

//The bq769x0 without CRC has the address 0x08 (section 5 "Device Comparison Table")

int bqI2CAddress = 0x08; //7-bit I2C address

// ProVant-EMERGENTIA pack has a 12 cell lipo that runs at 48V:

#define NUMBER_OF_CELLS 15 // Although ProVant Emergentia has only 12 cells, all the IC pins are required in order to operate properly. On the readings, ignore cells 4, 9 and 14, they are short circuited.

//Max number of ms before timeout error. 100 is pretty good
#define MAX_I2C_TIME 100

volatile boolean bq769x0_IRQ_Triggered = false; //Keeps track of when the Alert pin has been raised

float gain = 0; //These are two internal factory set values.
int offset = 0; //We read them once at boot up and use them in many functions

long lastTime; //Used to blink the status LED

long totalCoulombCount = 0; //Keeps track of overall pack fuel gauage

float cellVoltage[NUMBER_OF_CELLS + 1]; //Keeps track of the cell voltages

//GPIO declarations
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

byte irqPin = 2; //Interrupt enabled, connected to bq pin ALERT
byte statLED = 13; //Ob board status LED

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, HIGH);

  Serial.begin(9600);
  delay(5000);
  Serial.println("vamos vamos a testar");
  Serial.println("bq76940 example");
 
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin(); //Start I2C communication

  delay(5000);
  pinMode(statLED, OUTPUT);
  digitalWrite(statLED, LOW); //Turn off the LED for now

  if(initBQ(2) == false)
  {
    Serial.println("bq76940 failed to respond - check your wiring");
    Serial.println("Hanging.");
    while(1);
  }
  else
  {
    Serial.println("bq76940 initialized!");
  }
  
  lastTime = millis();
 
}



void loop()
{


 Serial.println("sempre sempre a testar");
 
 delay(1000);
 
 if(millis() - lastTime > 3000)
  {
    for(int i = 0 ; i < NUMBER_OF_CELLS+1 ; i++){
      cellVoltage[i] = readCellVoltage(i);
    }
    lastTime = millis();
  }

 for(int i = 0 ; i < NUMBER_OF_CELLS+1 ; i++){
      Serial.println(cellVoltage[i]);
    }
    

  /*
  byte a = registerRead(bq796x0_SYS_STAT);
  byte ov = a & bq796x0_OV;
  if (ov != 0)
    Serial.println("Overvoltage detected !!!!!!");

  byte uv = a & bq796x0_UV;
  if (uv != 0)
    Serial.println("Undervoltage detected !!!!!!");

  if (a & bq796x0_SCD){
    Serial.println("Short circuit on discharge !!!!!!");
    registerWrite(bq796x0_SYS_STAT, 0);
    registerRead(bq796x0_SYS_STAT);
  }

  byte cc = registerRead(bq796x0_SYS_CTRL2);
  if (!(cc & bq796x0_CC_EN)){
    registerWrite(bq796x0_SYS_CTRL2, cc | bq796x0_CC_EN);
    Serial.println("Novo valor de CC:");
    registerRead(bq796x0_SYS_CTRL2);
  }

*/

/*int temp = readTemp(1);
Serial.print("Die temp = ");
Serial.println(temp);*/
delay(3000);

}















// *************************************************************************    FUNCTIONs   **************************************************************************************************** //






// ------------------------ init BQ ----------------------------------------------------- // 

boolean initBQ(byte irqPin)
{
  
  //Test to see if we have correct I2C communication
  byte testByte = registerRead(bq796x0_ADCGAIN2); // This reading is used only to check if the read value is plausible, it should be something other than zero on POR.

  for(byte x = 0 ; x < 10 && testByte == 0 ; x++)
  {
    Serial.print(".");
    testByte = registerRead(bq796x0_ADCGAIN2);
    delay(100);
  }
  
  if(testByte == 0x00) return false; //Something is very wrong. Check wiring.

  //"For optimal performance, [CC_CFG] should be programmed to 0x19 upon device startup." page 40
  registerWrite(bq796x0_CC_CFG, 0x19); //address, value


  DEBUG_PRINTLN("// Testing ADC \\");
  //Double check that ADC is enabled
  byte sysVal = registerRead(bq796x0_SYS_CTRL1);
  if(sysVal & bq796x0_ADC_EN)
  {
    Serial.println("ADC Already Enabled");
  }
  sysVal |= bq796x0_ADC_EN; //Set the ADC_EN bit
  if (!registerWrite(bq796x0_SYS_CTRL1, sysVal))
    Serial.println("Erro ao escrever no registrador do ADC"); //address, value

  DEBUG_PRINTLN();

  DEBUG_PRINTLN("// Coulomb Counter \\");
  //Enable countinous reading of the Coulomb Counter
  sysVal = registerRead(bq796x0_SYS_CTRL2);
  sysVal |= bq796x0_CC_EN; //Set the CC_EN bit
  if (!registerWrite(bq796x0_SYS_CTRL2, sysVal))
    Serial.println("Erro para ativar o CC"); //address, value
  sysVal = registerRead(bq796x0_SYS_CTRL2);
  Serial.println("Coulomb counter enabled");

  DEBUG_PRINTLN();

  //Attach interrupt
  pinMode(irqPin, INPUT); //No pull up

  if(irqPin == 2)
    //Interrupt zero on Uno is pin 2
    attachInterrupt(0, bq769x0IRQ, RISING);
  else if (irqPin == 3)
    //Interrupt one on Uno is pin 3
    attachInterrupt(1, bq769x0IRQ, RISING);
  else
    Serial.println("irqPin invalid. Alert IRQ not enabled.");

  DEBUG_PRINTLN("// Gain and Offset \\");
  //Gain and offset are used in multiple functions
  //Read these values into global variables
  gain = readGAIN() / (float)1000; //Gain is in uV so this converts it to mV. Example: 0.370mV/LSB
  offset = readADCoffset(); //Offset is in mV. Example: 65mV

  Serial.print("gain: ");
  Serial.print(gain);
  Serial.println("mV/LSB");

  Serial.print("offset: ");
  Serial.print(offset);
  Serial.println("mV");

  DEBUG_PRINTLN();

  //Read the system status register
  byte sysStat = registerRead(bq796x0_SYS_STAT);
  if(sysStat & bq796x0_DEVICE_XREADY)
  {
    Serial.println("Device X Ready Error");
    //Try to clear it
    registerWrite(bq796x0_SYS_STAT, bq796x0_DEVICE_XREADY);

    delay(500);
    //Check again  
    byte sysStat = registerRead(bq796x0_SYS_STAT);
    if(sysStat & bq796x0_DEVICE_XREADY)
    {
      Serial.println("Device X Ready Not Cleared");
    }
  }

  DEBUG_PRINTLN("// Trip voltages \\");
  //Set any other settings such as OVTrip and UVTrip limits
  float under = readUVtrip();
  float over = readOVtrip();

  Serial.print("Undervoltage trip: ");
  Serial.print(under);
  Serial.println("V");

  Serial.print("Overvoltage trip: ");
  Serial.print(over);
  Serial.println("V");

  if(under != 3.13)
  {
    writeUVtrip(3.13); //Set undervoltage to 3.32V
    Serial.print("New undervoltage trip: ");
    Serial.print(readUVtrip());
    Serial.println("V"); //should print 3.32V
  }

  if(over != 4.27)
  {
    writeOVtrip(4.27); //Set overvoltage to 4.27V
    Serial.print("New overvoltage trip: ");
    Serial.print(readOVtrip());
    Serial.println("V"); //should print 4.27V
  }

  DEBUG_PRINTLN();
  return true;
}

//-------------------------------------------------------- init bq function end ----------------------------------------- //






//-------------------------------------------------------- read gain function  ----------------------------------------- //
int readGAIN(void)
{
  byte val1 = registerRead(bq796x0_ADCGAIN1);
  byte val2 = registerRead(bq796x0_ADCGAIN2);
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
int readADCoffset(void)
{
  //Here we need to convert a 8bit 2's compliment to a 16 bit int
  char offset = registerRead(bq796x0_ADCOFFSET);

  return((int)offset); //8 bit char is now a full 16-bit int. Easier math later on.
}
//-------------------------------------------------------- read ADC offset function end ----------------------------------------- //




//-------------------------------------------------------- read Overvoltage function  ----------------------------------------- //
//Returns the over voltage trip threshold
//Default is 0b.10.OVTRIP(0xAC).1000 = 0b.10.1010.1100.1000 = 0x2AC8 = 10,952
//OverVoltage = (OV_TRIP * GAIN) + ADCOFFSET
//Gain and Offset is different for each IC
//Example: voltage = (10,952 * 0.370) + 56mV = 4.108V
float readOVtrip(void)
{
  int trip = registerRead(bq796x0_OV_TRIP);

  trip <<= 4; //Shuffle the bits to align to 0b.10.XXXX.XXXX.1000
  trip |= 0x2008;

  float overVoltage = ((float)trip * gain) + offset;
  overVoltage /= 1000; //Convert to volts

  //Serial.print("overVoltage should be around 4.108: ");
  //Serial.println(overVoltage, 3);

  return(overVoltage);
}


//-------------------------------------------------------- read Overvoltage function end  ----------------------------------------- //







//-------------------------------------------------------- write Overvoltage limit value function  ----------------------------------------- //

//Given a voltage (4.22 for example), set the over voltage trip register
//Example: voltage = 4.2V = (4200mV - 56mV) / 0.370mv = 11,200
//11,200 = 0x2BC0 = 
void writeOVtrip(float tripVoltage)
{

  if ((tripVoltage < (calcVoltage(min_OV_tripValue)/1000)) || (tripVoltage > (calcVoltage(max_OV_tripValue)/1000))){

    return;
  }

  byte val = tripCalculator(tripVoltage); //Convert voltage to an 8-bit middle value
  registerWrite(bq796x0_OV_TRIP, val); //address, value
}

//-------------------------------------------------------- write Overvoltage limit value function end  ----------------------------------------- //






//-------------------------------------------------------- read Undervoltage function  ----------------------------------------- //

//Returns the under voltage trip threshold
//Default is 0b.01.UVTRIP(0x97).0000 = 0x1970 = 6,512
//UnderVoltage = (UV_TRIP * GAIN) + ADCOFFSET
//Gain and Offset is different for each IC
//Example: voltage = (6,512 * 0.370) + 56mV = 2.465V
float readUVtrip(void)
{
  byte reg = registerRead(bq796x0_UV_TRIP);

  uint16_t trip = (uint16_t)reg;
  trip <<= 4; //Shuffle the bits to align to 0b.01.XXXX.XXXX.0000
  trip |= 0x1000;

  float underVoltage = ((float)trip * gain) + offset;
  underVoltage /= 1000; //Convert to volts

  //Serial.print("underVoltage should be around 2.465: ");
  //Serial.println(underVoltage, 3);

  return(underVoltage);
}


//-------------------------------------------------------- read Undervoltage function end  ----------------------------------------- //



//-------------------------------------------------------- write Undervoltage limit value function  ----------------------------------------- //

//Given a voltage (2.85V for example), set the under voltage trip register
void writeUVtrip(float tripVoltage)
{
  if ((tripVoltage < (calcVoltage(min_UV_tripValue)/1000)) || (tripVoltage > (calcVoltage(max_UV_tripValue)/1000)))
    return;

  byte val = tripCalculator(tripVoltage); //Convert voltage to an 8-bit middle value
  registerWrite(bq796x0_UV_TRIP, val); //address, value
}

//-------------------------------------------------------- write Undervoltage limit value function end  ----------------------------------------- //


//-------------------------------------------------------- Read from a register function  ----------------------------------------------------------- //


// Returns a given register
byte registerRead(byte regAddress) {
  byte value = 0;
  
  // Primeiro enviamos o endereço do registrador que queremos ler

  Wire.beginTransmission(bqI2CAddress);
  Wire.write(regAddress);

  
  byte error = Wire.endTransmission(false); // Não envia STOP condition

  
  if (error != 0) {
    Serial.print("Erro na transmissão I2C na tentativa de ler o registrador 0x"); 
    Serial.println(regAddress, HEX);
    Serial.print("I2C Error code:");
    Serial.println(error);
    return 0;
  }
  


  
  // Agora solicitamos 1 byte de dados
  Wire.requestFrom(bqI2CAddress, (uint8_t)1);
  
  // Esperamos pelos dados com timeout
  unsigned long startTime = millis();
  while (Wire.available() == 0) {
    if (millis() - startTime > 100) { // Timeout de 100ms
      Serial.println("Timeout na leitura I2C");
      return 0;
    }
  }
  
  value = Wire.read();
  
  Serial.print("Valor lido do registrador 0x");
  Serial.print(regAddress, HEX);
  Serial.print(": 0x");
  Serial.println(value, HEX);
  
  return value;
}

//-------------------------------------------------------- Read from a register function end ----------------------------------------------------------- //


//-------------------------------------------------------- Write in a register function  ----------------------------------------------------------- //
//Write a given value to a given register
bool registerWrite(byte regAddress, byte regData){
  // O CRC é calculado sobre: Endereço I2C (com bit de escrita) + Reg Address + Dado
  byte i2cWriteAddr = (bqI2CAddress << 1) | 0; // Endereço de 8 bits para escrita
  byte dataForCRC[3] = {i2cWriteAddr, regAddress, regData};
  
  byte crcValue = calculateCRC(dataForCRC, 3);

  Wire.beginTransmission(bqI2CAddress);
  Wire.write(regAddress);
  Wire.write(regData);
  Wire.write(crcValue); // Envia o CRC calculado
  byte error = Wire.endTransmission();

  if (error != 0) return false;

  Serial.print("Valor escrito no registrador 0x");
  Serial.print(regAddress, HEX);
  Serial.print(": 0x");
  Serial.println(regData, HEX);

  return true;
}


//The BQ7694001 demands a CRC at the last byte in a write transmission
byte calculateCRC(byte data_buffer[], byte len)
{
  byte crc = 0;
  for (byte i = 0; i < len; i++) {
    crc ^= data_buffer[i];
    for (byte j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

//-------------------------------------------------------- Write in a register function end  ----------------------------------------------------------- //


//-------------------------------------------------------- interrupt function  ----------------------------------------------------------- //

// this is irq handler for bq769x0 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void bq769x0IRQ()
{
  bq769x0_IRQ_Triggered = true;
}
//-------------------------------------------------------- interrupt function end ----------------------------------------------------------- //







//-------------------------------------------------------- Voltage reading convertion function ----------------------------------------------------------- //

//Under voltage and over voltage use the same rules for calculating the 8-bit value
//Given a voltage this function uses gain and offset to get a 14 bit value
//Then strips that value down to the middle-ish 8-bits
//No registers are written, that's up to the caller
//OV integer value must range 8200-12280
//UV integer value must range 4096-8176
byte tripCalculator(float tripVoltage)
{

  tripVoltage *= 1000; //Convert volts to mV

  //Serial.print("tripVoltage to be: ");
  //Serial.println(tripVoltage, 3);

  tripVoltage -= offset;
  tripVoltage /= gain;


  int tripValue = (int)tripVoltage; //We only want the integer - drop decimal portion.


  //Serial.print("tripValue should be something like 0x2BC0: ");
  //Serial.println(tripValue, HEX);

  tripValue >>= 4; //Cut off lower 4 bits
  tripValue &= 0x00FF; //Cut off higher bits

  //Serial.print("About to report tripValue: ");
  //Serial.println(tripValue, HEX);


  return(tripValue);
}

//-------------------------------------------------------- Voltage reading convertion function end ----------------------------------------------------------- //

//-------------------------------------------------------- Double register read function begin (aux function)  ----------------------------------------------------------- //


//Returns the atmoic int from two sequentials reads
int registerDoubleRead(byte regAddress)
{
  Wire.beginTransmission(bqI2CAddress);
  Wire.write(regAddress);
  Wire.endTransmission();

  Wire.requestFrom(bqI2CAddress, 2);

  byte reg1 = Wire.read();
  byte reg2 = Wire.read();

  Serial.print("Reading from register addresses: 0x");
  Serial.print(regAddress, HEX);
  Serial.print(" and 0x");
  Serial.println(regAddress + 1, HEX);

  Serial.print("reg1 (0x");
  Serial.print(regAddress, HEX);
  Serial.print("): 0x");
  Serial.println(reg1, HEX);

  Serial.print("reg2 (0x");
  Serial.print(regAddress + 1, HEX);
  Serial.print("): 0x");
  Serial.println(reg2, HEX);

  int combined = (int)reg1 << 8;
  combined |= reg2;

  return(combined);
}


//-------------------------------------------------------- Double register read function end (aux function)  ----------------------------------------------------------- //


//-------------------------------------------------------- Calculate Voltage (aux function)  ----------------------------------------------------------- //
//Return the real voltage in mv
inline float calcVoltage(unsigned rawValue){
  return gain*rawValue+offset;
}

//-------------------------------------------------------- Calculate Voltage (aux function)  ----------------------------------------------------------- //

//-------------------------------------------------------- Read the voltage of a cell function begin ---------------------------------------------------------- //

float readCellVoltage(byte cellNumber)
{
  if(cellNumber < 1 || cellNumber > 15) return(-1); //Return error

  Serial.print("Read cell number: ");
  Serial.println(cellNumber);

  //Reduce the caller's cell number by one so that we get register alignment
  cellNumber--;

  byte registerNumber = bq796x0_VC1_HI + (cellNumber * 2);

  Serial.print("register: 0x");
  Serial.println(registerNumber, HEX);

  int cellValue = registerDoubleRead(registerNumber);

  //int cellValue = 0x1800; //6,144 - Should return 2.365
  //int cellValue = 0x1F10l; //Should return 3.052

  //Cell value should now contain a 14 bit value

  Serial.print("Cell value (dec): ");
  Serial.println(cellValue);
  
  if(cellValue == 0) return(0);

  float cellVoltage = cellValue * gain + offset; //0x1800 * 0.37 + 60 = 3,397mV
  cellVoltage /= (float)1000;

  Serial.print("Cell voltage: ");
  Serial.println(cellVoltage, 3);

  return(cellVoltage);
}


//-------------------------------------------------------- Read the voltage of a cell function end ---------------------------------------------------------- //



//-------------------------------------------------------- Read the temperature function begin ---------------------------------------------------------- //

int readTemp(byte thermistorNumber) //Não está lendo os registradores corretos para os termistores 2 e 3, verificar por que. Para o transistor 1 funciona bem. 
{

  if(thermistorNumber > 3) return -1; // Invalid input 

  byte sysValue = registerRead(bq796x0_SYS_CTRL1);

  if(thermistorNumber > 0)
  {
    if((sysValue & bq796x0_TEMP_SEL) == 0)
    {
      sysValue |= bq796x0_TEMP_SEL;
      registerWrite(bq796x0_SYS_CTRL1, sysValue);
      Serial.println("Waiting 2 seconds to switch thermistors");
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

    Serial.print("Este é o meu thermistorrnumber");
    Serial.println(thermistorNumber);
    Serial.print("Este é o meu registernumber");
    Serial.println(registerNumber);
    int thermValue = registerDoubleRead(registerNumber);

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
      registerWrite(bq796x0_SYS_CTRL1, sysValue);
      Serial.println("Waiting 2 seconds to switch to internal die thermistors");
      delay(2000);      
    }

    int thermValue = registerDoubleRead(bq796x0_TS1_HI);
    float thermVoltage = thermValue * (float)382;
    thermVoltage /= (float)1000000;

    float temperatureC = 25.0 - ((thermVoltage - 1.2) / 0.0042);

    return (int)temperatureC;
  }

  // Safety catch-all:
  return -1;
}


int thermistorLookup(float resistance)
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