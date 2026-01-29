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
#include "definitions.hpp"
#include "drivers.hpp"
#include "BQ76940.hpp"
// ----------------------------------------- INCLUDES END  ------------------------------------------------ //



// ----------------------------------------- FUNCTIONS PROTOTYPES ------------------------------------------------ //


void bq769x0IRQ();
// ----------------------------------------- FUNCTIONS PROTOTYPES END  ------------------------------------------------ //


volatile boolean bq769x0_IRQ_Triggered = false; //Keeps track of when the Alert pin has been raised

long lastTime; //Used to blink the status LED

long totalCoulombCount = 0; //Keeps track of overall pack fuel gauage

float cellVoltage[NUMBER_OF_CELLS + 1]; //Keeps track of the cell voltages

//GPIO declarations
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

byte irqPin = 2; //Interrupt enabled, connected to bq pin ALERT
byte statLED = 13; //Ob board status LED

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

BQ76940 *bq;

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, HIGH);

  Serial.begin(9600);
  delay(5000);
  DEBUG_PRINTLN1("vamos vamos a testar");
  DEBUG_PRINTLN1("bq76940 example");
 
  bq = new BQ76940(0, 1);

  delay(5000);
  pinMode(statLED, OUTPUT);
  digitalWrite(statLED, LOW); //Turn off the LED for now

  if(bq->initBQ(21) == false)
  {
    DEBUG_PRINTLN1("bq76940 failed to respond - check your wiring");
    DEBUG_PRINTLN1("Hanging.");
    while(1);
  }
  else
  {
    DEBUG_PRINTLN1("bq76940 initialized!");
  }
  
  lastTime = millis();

 
}



void loop()
{


 DEBUG_PRINTLN1("sempre sempre a testar");
 
 delay(1000);

 
 
 if(millis() - lastTime > 3000)
  {
    for(int i = 0 ; i < NUMBER_OF_CELLS+1 ; i++){
      cellVoltage[i] = bq->readCellVoltage(i);
    }
    lastTime = millis();
  }

 for(int i = 0 ; i < NUMBER_OF_CELLS+1 ; i++){
      Serial.println(cellVoltage[i]);
    }
    
  Serial.println(bq->readSysStat(),BIN);
  if (testBit(bq->readSysStat(), bq796x0_SCD)){
    bq->resetSysStatBit(bq796x0_SCD);
  }
  if (testBit(bq->readSysStat(), bq796x0_UV)){
    bq->resetSysStatBit(bq796x0_UV);
  }

  bq->readCurrent();
  //Serial.println(bq->getCurrent());

  bq->setCConeshot();
  
  // byte a = registerRead(bq796x0_SYS_STAT);
  // byte ov = a & bq796x0_OV;
  // if (ov != 0)
  //   DEBUG_PRINTLN("Overvoltage detected !!!!!!");

  // byte uv = a & bq796x0_UV;
  // if (uv != 0)
  //   DEBUG_PRINTLN("Undervoltage detected !!!!!!");

  // if (a & bq796x0_SCD){
  //   DEBUG_PRINTLN("Short circuit on discharge !!!!!!");
  //   registerWrite(bq796x0_SYS_STAT, a | bq796x0_SCD);
  //   registerRead(bq796x0_SYS_STAT);
  // }

  // byte cc = registerRead(bq796x0_SYS_CTRL2);
  // if (!(cc & bq796x0_CC_EN)){
  //   registerWrite(bq796x0_SYS_CTRL2, cc | bq796x0_CC_EN);
  //   DEBUG_PRINTLN("Novo valor de CC:");
  //   registerRead(bq796x0_SYS_CTRL2);
  // }

  // cc = registerRead(bq796x0_SYS_CTRL2);
  // if (testBit(cc, bq796x0_DSG_ON)){
  //   toggleBit(cc, bq796x0_DSG_ON);
  //   registerWrite(bq796x0_SYS_CTRL2, cc);
  // }

  // cc = registerRead(bq796x0_SYS_CTRL2);
  // if (testBit(cc, bq796x0_CHG_ON)){
  //   toggleBit(cc, bq796x0_CHG_ON);
  //   registerWrite(bq796x0_SYS_CTRL2, cc);
  // }


/*int temp = readTemp(1);
DEBUG_PRINT("Die temp = ");
DEBUG_PRINTLN(temp);*/
delay(3000);

}




