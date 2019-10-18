//node_prod_pwr_boost is based on the low power labs Moteino and WeatherSheild R2 with subsequent changes.

// **********************************************************************************************************
// WeatherShield R2 (BME280 sensor) sameple sketch that works with Moteinos equipped with RFM69W/RFM69HW
// It sends periodic weather readings (temp, hum, atm pressure) from WeatherShield to the base node Moteino
// For use with MoteinoMEGA you will have to revisit the pin definitions defined below
// http://www.LowPowerLab.com/WeatherShield
// Example setup (with R1): http://lowpowerlab.com/blog/2015/07/24/attic-fan-cooling-tests/
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// License can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <node_prod_pwr_boost.h>
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <Wire.h>          //included in Arduino IDE (www.arduino.cc)
#include <SparkFunBME280.h>//get it here: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/tree/master/Libraries/Arduino/src
#include <LowPower.h>      //get it here: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define GATEWAYID   1
#define NODEID      84 //86 was the last - below 40 are test&debug nodes
#define NETWORKID   100
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
//*********************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75
//*********************************************************************************************
#define SEND_LOOPS   75 //send data this many sleep loops (75 loops of 8sec cycles = 600sec ~ 10 minutes)
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_4S; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************
#define BATT_MONITOR_EN A3 //enables battery voltage divider to get a reading from a battery, disable it to save power
#define BATT_MONITOR  A7   //The 1.4h node can read the VIN line directly due to the boost circuit that is used as a reference
#define BATT_CYCLES   2    //read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cyclesyou would get ~1 hour intervals
//#define BATT_FORMULA(reading) reading * 0.00322 // >>> fine tune this parameter to match your voltage when fully charged
#define BATT_FORMULA(reading) ((reading * 0.00437)+ .54)/1.68  // >>> Tuned for 1.4h node
#define BATT_LOW      3.6  //(volts)
#define BATT_READ_LOOPS  SEND_LOOPS*10  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
//*****************************************************************************************************************************
//*****************************
#define POWER_BOOST_EN 3
#define TRANSMIT_COUNTER 150 //150*4s = 10min - multiply the transmit counter * sleep time to determine transmit interval
//*****************************

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#define BLINK_EN                 //uncomment to blink LED on every send
#define SERIAL_EN                //comment out if you don't want any serial output

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
//*****************************************************************************************************************************

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(FLASH_SS, 0xEF30); //WINDBOND 4MBIT flash chip on CS pin D8 (default for Moteino)

BME280 bme280;
char Pstr[10];
char Fstr[10];
char Hstr[10];
char buffer[50];
int cycles,powerCycles,readCycles;
double F,P,H;

void setup(void)
{
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif
  pinMode(LED, OUTPUT);

  sprintf(buffer, "SensorIoT Node - Network ID %d Gateway ID %d", NETWORKID, NODEID);
  DEBUGln(buffer);

  DEBUGln("Setting up device");

  enablePowerBoost(); //enable the power boost circuit during setup and initialization

  cycles=0;
  powerCycles=0;
  readCycles=0;
  
  //initialize weather shield BME280 sensor
  bme280.settings.commInterface = I2C_MODE;
  bme280.settings.I2CAddress = 0x77;
  bme280.settings.runMode = 3; //Normal mode
  bme280.settings.tStandby = 0;
  bme280.settings.filter = 0;
  bme280.settings.tempOverSample = 1;
  bme280.settings.pressOverSample = 1;
  bme280.settings.humidOverSample = 1;

  DEBUGln("Initializing and sleeping BME280");

  
  bme280.begin();
  P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
  F = bme280.readTempF();
  H = bme280.readFloatHumidity();
  bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280
  
  
  DEBUGln("Blinking LED");
  
  Blink(LED, 100);Blink(LED, 100);Blink(LED, 100);

  DEBUGln("Initializing radio");

  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);


//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  sprintf(buffer, "Transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);

  radio.sleep();
  SERIALFLUSH();
}

unsigned long doorPulseCount = 0;
char input=0;
byte sendLoops=0;
byte battReadLoops=0;
float batteryVolts = 5;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
byte sendLen;

void loop()
{
  enablePowerBoost();

  if(readCycles > TRANSMIT_COUNTER) {
    Blink(LED, 8); // 50ma * 5ms
    delay(2); //Dont read bat voltage for at least 10 millis (3 for enable, 5 for led blink, so 2 more here)
    readBattery(); //50ma * .35ms
    readCycles=0;


    //read BME sensor
    bme280.begin();
    P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
    F = bme280.readTempF();
    H = bme280.readFloatHumidity();
    bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280


    dtostrf(F, 3,2, Fstr);
    dtostrf(H, 3,2, Hstr);
    dtostrf(P, 3,2, Pstr);
    DEBUGln(Fstr);

    sprintf(buffer, "BAT:%sv F:%s H:%s P:%s", BATstr, Fstr, Hstr, Pstr);

    sendLen = strlen(buffer);
    radio.sendWithRetry(GATEWAYID, buffer, sendLen, 1); //retry one time
    DEBUG(buffer); DEBUG(" (packet length:"); DEBUG(sendLen); DEBUGln(")");
 
  }
  
  radio.sleep();

  //The rest of the loop costs 10ma * 4ms
  disablePowerBoost();
  SERIALFLUSH();
  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);
  readCycles++;
  cycles++;
  DEBUG(cycles);
  DEBUGln(" WAKEUP");
}

void enablePowerBoost()
{
  pinMode(POWER_BOOST_EN, OUTPUT);
  digitalWrite(POWER_BOOST_EN, HIGH);
  DEBUGln("Power boost on");
  delay(3); //hackey atm, but lets try holding the boost on a bit longer and see if the power stabilizes a bit
}

void disablePowerBoost()
{
  digitalWrite(POWER_BOOST_EN, LOW);
  pinMode(POWER_BOOST_EN, INPUT);
  DEBUGln("Power boost off");
}

void readBattery()
{
  unsigned int readings=0;
  
  //enable battery monitor on WeatherShield (via mosfet controlled by A3)
  //pinMode(BATT_MONITOR_EN, OUTPUT);
  //digitalWrite(BATT_MONITOR_EN, LOW);

  //take several samples, and average
  for (byte i=0; i<5; i++) {
    unsigned int reading=0;
    reading = analogRead(BATT_MONITOR);
    DEBUG("raw ");
    DEBUGln(reading);
    batteryVolts = BATT_FORMULA(reading);
    dtostrf(batteryVolts,3,2, BATstr);
    DEBUGln(BATstr);
    
    readings+=reading;
  }
  
  //disable battery monitor
  pinMode(BATT_MONITOR_EN, INPUT); //highZ mode will allow p-mosfet to be pulled high and disconnect the voltage divider on the weather shield
    
  batteryVolts = BATT_FORMULA(readings / 5.0);
  dtostrf(batteryVolts,3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
  //if (batteryVolts <= BATT_LOW) BATstr = "LOW";
  DEBUG("Average reading: ");
  DEBUGln(BATstr);
}

void Blink(byte PIN, byte DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS/2);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS/2);  
}
