#include <SPI.h>                //builtin
#include <RHReliableDatagram.h> //part of RadioHead - Platformio ID#124
#include <SPIFlash.h>           //Platformio ID#125
#include <Wire.h>               //builtin
#include <SparkFunBME280.h>     //Platformio ID#684
#include <LowPower.h>           //Platformio ID#38

//******************************************************
//*******           Hardware Config              *******
//******************************************************
// Select radio - only uncomment one!
#define IS_RF95
//#define IS_RFM69
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define RF95_MODEM_CONFIG RH_RF95::Bw125Cr45Sf128

// Singleton instance of the radio driver
#ifdef IS_RF95
  #include <RH_RF95.h>            //part of RadioHead - Platformio ID#124
  RH_RF95 driver;
  #define MAX_MESSAGE_LEN     RH_RF95_MAX_MESSAGE_LEN
  #define MINTXPOWER          5
  #define MAXTXPOWER          15
#endif
#if defined (IS_RFM69) || defined (IS_RFM69HW)
  #include <RH_RF69.h>            //part of RadioHead - Platformio ID#124
  RH_RF69 driver;
  #define MAX_MESSAGE_LEN     RH_RF69_MAX_MESSAGE_LEN
  #define MINTXPOWER          -2
  #define MAXTXPOWER          20
#endif

// Select frequency - only uncomment one!
//#define FREQUENCY       433
//#define FREQUENCY       868
#define FREQUENCY       915

#define BATT_MONITOR  A7   //The 1.4e and greater nodes can read the VIN line directly. VIN should be 1.6V or less and after it stabilizing the boost side will provide a 3.3V reference.
#define BATT_FORMULA(reading) ((reading * 0.00437)+ .54)/1.68  // >>> Tuned for 1.4h node

#define POWER_BOOST_EN 3

#define LED           9 // SensorIoT Nodes have LEDs on D9
//#define FLASH_EN
#define FLASH_SS      8 // and FLASH SS on D8 (Like Moteinos)
#define SERIAL_EN       // uncomment to enable Serial output

#define BLINK_EN        //uncomment to blink LED on every send

#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif



//******************************************************
//*******              Node Config               *******
//******************************************************
#define VERSION      "1.1.1"
#define GATEWAY_ID   1
#define NODE_ID      96 //94 was the last - below 40 are test&debug nodes
#define SECSBETWEENXMITS  600 //(20 for test, 600 for prod) wait this many secs between xmits. This is approximate based on how long the node sleeps every loop.
/*
  LOOPSWITHACK is the number of successful xmit loops that will happen before the node stops waiting for ACKs and CtrlMsgs. Waiting
  several seconds with the reciever on and then ACKing the ACK is quite a drain on the battery over months. Everytime the node starts
  it will wait this many loops to let the GW and Node settle on the appropriate tx power. Everytime the node restarts this process will
  begin again.
*/
#define LOOPSWITHACK      50 
//period_t is an enum type defined in the LowPower library (LowPower.h)
period_t currentSleepTime = SLEEP_1S; //start at 1S, this will be dynamically increased if the xmit loop is stable.
/* BATT_READ_LOOPS - read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min).
                     For 450 cycles you would get ~1 hour intervals between readings
*/
#define BATT_READ_LOOPS  SEND_LOOPS*10  //  





// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram radio(driver, NODE_ID);


#ifdef FLASH_EN
  SPIFlash flash(FLASH_SS, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)
#endif

//The nodeStatus byte is 8 status bits as defined below
uint8_t nodeStatus;
#define NODESTATUS_FIRSTLOOP    1
#define NODESTATUS_XMIT_SUCCESS 2
#define NODESTATUS_WAIT_FOR_ACK 4 //Will be true if the node is requesting and waiting for ACKs from the GW
#define NODESTATUS_MAX_POWER    8
#define NODESTATUS_MIN_POWER    16



BME280 bme280;
char Pstr[10];
char Fstr[10];
char Hstr[10];
uint8_t buffer[MAX_MESSAGE_LEN];
double F,P,H;
uint8_t _txPower;
char STStr[4];
char TxStr[4];
uint8_t sleepLevel;
uint8_t stateByte;
char input=0;
uint8_t loopSuccess=0;
uint16_t loopCounter=0;
byte battReadLoops=0;
float batteryVolts = 1.5;
char BATstr[10]="BAT:1.50v"; //longest battery voltage reading message = 9chars
byte sendLen;
uint16_t secsSinceXmit, secsBetweenXmit;
uint16_t currentMillis=0;



void enablePowerBoost();
void disablePowerBoost();
void readBattery();
void Blink(byte PIN, byte DELAY_MS);
void handleControlMsg(char* msg);
uint8_t getCurrentSleepTime();
void setNextSleepTime();
void sendWithoutCtrlMsg();
void sendWithCtrlMsg();
void printMillis();

