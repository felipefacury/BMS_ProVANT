#include "drivers.hpp"

CommDriver::CommDriver(pin_size_t SDA, pin_size_t SCL){
  Wire.setSDA(SDA); 
  Wire.setSCL(SCL);
  Wire.begin(); //Start I2C communication
}


//-------------------------------------------------------- Read from a register function  ----------------------------------------------------------- //
byte CommDriver::registerRead(byte regAddress) 
{
  byte value = 0;
  
  // Primeiro enviamos o endereço do registrador que queremos ler

  WIRE.beginTransmission(bqI2CAddress);
  WIRE.write(regAddress);

  
  byte error = WIRE.endTransmission(false); // Não envia STOP condition

  
  if (error != 0) {
    DEBUG_PRINT1("Erro na transmissão I2C na tentativa de ler o registrador 0x"); 
    DEBUG_PRINTLN2(regAddress, HEX);
    DEBUG_PRINT1("I2C Error code:");
    DEBUG_PRINTLN1(error);
    return 0;
  }
  


  
  // Agora solicitamos 1 byte de dados
  WIRE.requestFrom(bqI2CAddress, (uint8_t)1);
  
  // Esperamos pelos dados com timeout
  unsigned long startTime = millis();
  while (WIRE.available() == 0) {
    if (millis() - startTime > 100) { // Timeout de 100ms
      DEBUG_PRINTLN1("Timeout na leitura I2C");
      return 0;
    }
  }
  
  value = WIRE.read();
  
  DEBUG_PRINT1("Valor lido do registrador 0x");
  DEBUG_PRINT2(regAddress, HEX);
  DEBUG_PRINT1(": 0x");
  DEBUG_PRINTLN2(value, HEX);
  
  return value;
}
//-------------------------------------------------------- Read from a register function end ----------------------------------------------------------- //



//-------------------------------------------------------- Write in a register function  ----------------------------------------------------------- //
//Write a given value to a given register
bool CommDriver::registerWrite(byte regAddress, byte regData)
{
  // O CRC é calculado sobre: Endereço I2C (com bit de escrita) + Reg Address + Dado
  byte i2cWriteAddr = (bqI2CAddress << 1) | 0; // Endereço de 8 bits para escrita
  byte dataForCRC[3] = {i2cWriteAddr, regAddress, regData};
  
  byte crcValue = calculateCRC(dataForCRC, 3);

  WIRE.beginTransmission(bqI2CAddress);
  WIRE.write(regAddress);
  WIRE.write(regData);
  WIRE.write(crcValue); // Envia o CRC calculado
  byte error = WIRE.endTransmission();

  if (error != 0) return false;

  DEBUG_PRINT1("Valor escrito no registrador 0x");
  DEBUG_PRINT2(regAddress, HEX);
  DEBUG_PRINT1(": 0x");
  DEBUG_PRINTLN2(regData, HEX);

  return true;
}


//The BQ7694001 demands a CRC at the last byte in a write transmission
byte CommDriver::calculateCRC(byte data_buffer[], byte len)
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



//-------------------------------------------------------- Double register read function begin (aux function)  ----------------------------------------------------------- //

//Returns the atmoic int from two sequentials reads
uint16_t CommDriver::registerDoubleRead(byte regAddress)
{
  WIRE.beginTransmission(bqI2CAddress);
  WIRE.write(regAddress);
  WIRE.endTransmission();

  WIRE.requestFrom(bqI2CAddress, 2);

  byte reg1 = WIRE.read();
  byte reg2 = WIRE.read();

  DEBUG_PRINT1("Reading from register addresses: 0x");
  DEBUG_PRINT2(regAddress, HEX);
  DEBUG_PRINT1(" and 0x");
  DEBUG_PRINTLN2(regAddress + 1, HEX);

  DEBUG_PRINT1("reg1 (0x");
  DEBUG_PRINT2(regAddress, HEX);
  DEBUG_PRINT1("): 0x");
  DEBUG_PRINTLN2(reg1, HEX);

  DEBUG_PRINT1("reg2 (0x");
  DEBUG_PRINT2(regAddress + 1, HEX);
  DEBUG_PRINT1("): 0x");
  DEBUG_PRINTLN2(reg2, HEX);

  uint16_t combined = (uint16_t)reg1 << 8;
  combined |= reg2;

  return(combined);
}

//-------------------------------------------------------- Double register read function end (aux function)  ----------------------------------------------------------- //