// **********************************************************************************
// SensorIoTNode.cpp
// This is a heavily modified version of Felix Rusu's WeatherShield R2 project
// converted to Platformio and changed to use the RadioHead library. Other changes include
// support for an onboard controllable boost circuit that allows the battery to charge up
// a capacitor which powers the node during its sleep cycle.
// Copyright 2019 Chris Brinton
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
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <SensorIoTNode.h>

void setup(void)
{

#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif

  pinMode(LED, OUTPUT);

  DEBUG("SensorIoT NodeID ");
  DEBUG(NODE_ID);
  DEBUG(" GW: ");
  DEBUG(GATEWAY_ID);
  DEBUG(" SW VER: ");
  DEBUGln(VERSION);
  
  //DEBUGln("Setting up device");

  enablePowerBoost(); //enable the power boost circuit during setup and initialization
  
  secsBetweenXmit = SECSBETWEENXMITS;

  //Set the node status bits
  nodeStatus |= NODESTATUS_FIRSTLOOP; //startup is true

  //initialize weather shield BME280 sensor
  bme280.settings.commInterface = I2C_MODE;
  bme280.settings.I2CAddress = 0x77;
  bme280.settings.runMode = 3; //Normal mode
  bme280.settings.tStandby = 0;
  bme280.settings.filter = 0;
  bme280.settings.tempOverSample = 1;
  bme280.settings.pressOverSample = 1;
  bme280.settings.humidOverSample = 1;

  //DEBUGln("Init and sleep BME280");

  
  bme280.begin();
  P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
  F = bme280.readTempF();
  H = bme280.readFloatHumidity();
  bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280
  
  
  //DEBUGln("Blinking LED");
  
  Blink(LED, 100);Blink(LED, 100);Blink(LED, 100);

  //DEBUGln("Init radio");

  
  if(!radio.init()) {
    //DEBUGln("Radio init failed");
  } else
  {
    //DEBUGln("Radio initialization succeeded");
  }
    
  driver.setFrequency(FREQUENCY);
  _txPower=(MAXTXPOWER-MINTXPOWER)/2+MINTXPOWER; //Start at a midpoint for xmit power.
  //DEBUG("Setting tx power to ");
  //DEBUGln(_txPower);
  #ifdef IS_RF95
    driver.setModemConfig(RF95_MODEM_CONFIG);
    driver.setTxPower(_txPower,false); //Tx power can be set from 5 to 23db
    radio.setRetries(2); //The number of times to retry to contact the GW if the initial send fails.
    radio.setTimeout(1000);
  #endif
  #ifdef IS_RFM69HW
    //driver.setModemConfig(RH_RF69::GFSK_Rb2Fd5);
    driver.setTxPower(_txPower,true); //Tx power can be set from -2 to 20db
    radio.setRetries(5); //The number of times to try to contact the server
    radio.setTimeout(500); //Millis to wait for an ACK. Tune for a particular modem config.
  #endif

  //DEBUG("Transmitting at: ");
  //DEBUGln(FREQUENCY);
  //DEBUGln(" Mhz");

//  radio.setHeaderFrom(NODE_ID);
  SERIALFLUSH();
  driver.sleep();

}

void loop() {
  DEBUGln("Start Loop");
  printMillis();
  enablePowerBoost();
  nodeStatus &= ~NODESTATUS_XMIT_SUCCESS;
  printMillis(); //4
  if((secsSinceXmit >= secsBetweenXmit) || (nodeStatus & NODESTATUS_FIRSTLOOP)) {
    Blink(LED, 8); 
  printMillis(); //8
    readBattery(); 
  printMillis(); //2
    
    //read BME sensor
    bme280.begin();
    P = bme280.readFloatPressure() * 0.0002953; //read Pa and convert to inHg
    F = bme280.readTempF();
    H = bme280.readFloatHumidity();
    bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280
  printMillis(); //37

    //The numbers need to be left justified
    dtostrf(F, -1,2, Fstr);
    dtostrf(H, -1,2, Hstr);
    dtostrf(P, -1,2, Pstr);
    dtostrf(_txPower, -1,0, TxStr);
    dtostrf(nodeStatus, -1,0, STStr);

    sprintf((char*)buffer, "BAT:%sv F:%s H:%s P:%s TX:%s ST:%s", BATstr, (char*)Fstr, (char*)Hstr, (char*)Pstr, (char*)TxStr, (char*)STStr);
//    sprintf((char*)buffer, "BAT:%sv F:%s H:%s P:%s", BATstr, (char*)Fstr, (char*)Hstr, (char*)Pstr);

  printMillis(); //2

    DEBUG("Sending ");
    DEBUGln((char*)buffer);

    DEBUG("loopCounter: ");
    DEBUGln(loopCounter);

    if(loopCounter <= LOOPSWITHACK) {
      sendWithCtrlMsg();//346
      nodeStatus |= NODESTATUS_WAIT_FOR_ACK;
    } else {
      sendWithoutCtrlMsg();//10
      nodeStatus &= ~NODESTATUS_WAIT_FOR_ACK;
    }
  printMillis(); 

      //DEBUGln("sleeping");
    secsSinceXmit = 0;
    delay(3); //Give the power boost a few millis to top off the cap before shutdown.
    driver.sleep();

    //if we have 10 successful xmit loops at the current sleep level, move up to the next level
    if(nodeStatus & NODESTATUS_XMIT_SUCCESS) {
      loopSuccess++;
      loopCounter++;
    }
    if(loopSuccess >= 5) {
      setNextSleepTime();
      loopSuccess = 0;
    }
  } else {
    secsSinceXmit+=getCurrentSleepTime();
  }
  printMillis(); //4

  disablePowerBoost();
  DEBUGln("End Loop");
  SERIALFLUSH();
  LowPower.powerDown(currentSleepTime, ADC_OFF, BOD_OFF);
  nodeStatus &= ~NODESTATUS_FIRSTLOOP; //set status for firstloop to false

}

void sendWithCtrlMsg() {
  sendLen = strlen((char*)buffer);
  // Send a message to radio_server
  DEBUG("sendWithCtrlMsg: ");
  DEBUGln((char*)buffer);

  if (radio.sendtoWait(buffer, sendLen, GATEWAY_ID)) {
//          DEBUG("SendtoWait succeeded with ");
//          DEBUG(radio.retransmissions());
//          DEBUGln(" retransmissions");
  // Now wait for a reply from the server
    uint8_t len = sizeof(buffer);
    uint8_t from;   
    if (radio.recvfromAckTimeout(buffer, &len, 3000, &from))
    {
      DEBUG("Control msg from GW: ");
      DEBUG((char*)buffer);
      DEBUG(" RSSI: ");
      DEBUGln(driver.lastRssi());
      handleControlMsg((char*)buffer);
      nodeStatus |= NODESTATUS_XMIT_SUCCESS;
    }
    else
    {
      DEBUGln("No CtrlMsg");
    }
  }
  else {
    DEBUG("SendtoWait FAILED");
//    DEBUG(radio.retransmissions());
//    DEBUGln(" retransmissions");
//    radio.resetRetransmissions();
    if(_txPower < MAXTXPOWER) {
      _txPower++;
      nodeStatus &= ~NODESTATUS_MAX_POWER;
    } else {
      nodeStatus |= NODESTATUS_MAX_POWER;
    }
    driver.setTxPower(_txPower);
    DEBUG("NOACK Pwr:");
    DEBUGln(_txPower);
  }

}

void sendWithoutCtrlMsg() {
  sendLen = strlen((char*)buffer);
  DEBUG("sendWithoutCtrlMsg: ");
  DEBUGln((char*)buffer);
  //Because we're effectively changing from RHReliableDatagram to RHDatagram
  //when we want to stop waiting for ACKs, we need to start managing some of the
  //header data ourselves. 
  radio.setHeaderFlags(RH_FLAGS_NONE, RH_FLAGS_ACK); // Clear the ACK flag
	radio.setHeaderId(loopSuccess); //Make sure we dont send the same seq num
  if (radio.sendto(buffer, sendLen, GATEWAY_ID)) {
    nodeStatus |= NODESTATUS_XMIT_SUCCESS;
  }
}

uint8_t getCurrentSleepTime() {
  switch((uint8_t)currentSleepTime) {
    case SLEEP_15MS:
      return 0;
    case SLEEP_250MS:
      return 0;
    case SLEEP_1S:
      return 1;
    case SLEEP_2S:
      return 2;
    case SLEEP_4S:
      return 4;
    case SLEEP_8S:
      return 8;
  }

  return 0; //Shouldnt get here :P
}

void setNextSleepTime() {
  switch((uint8_t)currentSleepTime) {
    case SLEEP_15MS:
      currentSleepTime = SLEEP_250MS;
      return;
    case SLEEP_250MS:
      currentSleepTime = SLEEP_1S;
      return;
    case SLEEP_1S:
      currentSleepTime = SLEEP_2S;
      return;
    case SLEEP_2S:
      currentSleepTime = SLEEP_4S;
      return;
    case SLEEP_4S:
      currentSleepTime = SLEEP_8S;
      return;
    case SLEEP_8S:
      currentSleepTime = SLEEP_8S;
      return;
  }
}

void handleControlMsg(char* msg){
  if(strcmp(msg, "TXPWR_UP") == 0){
    if(_txPower < MAXTXPOWER) {
        _txPower++;
        nodeStatus &= ~NODESTATUS_MAX_POWER;
      } else {
        nodeStatus |= NODESTATUS_MAX_POWER;
    }
    //DEBUG("CtrlMsg: TXPWR_UP Pwr:");
    //DEBUGln(_txPower);
  }
  if(strcmp(msg, "TXPWR_DN") == 0){
    if(_txPower > MINTXPOWER) {
      _txPower--;
      nodeStatus &= ~NODESTATUS_MIN_POWER;
    } else {
      nodeStatus |= NODESTATUS_MIN_POWER;
    }
    //DEBUG("CtrlMsg: TXPWR_DN Pwr:");
    //DEBUGln(_txPower);
  }
  #ifdef IS_RF95
    driver.setTxPower(_txPower,false);
  #endif
  #ifdef IS_RFM69HW
    driver.setTxPower(_txPower,true); 
  #endif
}

void enablePowerBoost()
{
  pinMode(POWER_BOOST_EN, OUTPUT);
  digitalWrite(POWER_BOOST_EN, HIGH);
//  DEBUGln("Power boost on");
  delay(3); //hackey atm, but lets try holding the boost on a bit longer and see if the power stabilizes a bit
}

void disablePowerBoost()
{
  digitalWrite(POWER_BOOST_EN, LOW);
  pinMode(POWER_BOOST_EN, INPUT);
//  DEBUGln("Power boost off");
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
    batteryVolts = BATT_FORMULA(reading);
    dtostrf(batteryVolts,3,2, BATstr);
    //DEBUG("Raw ADC reading ");
    //DEBUG(reading);
    //DEBUG(" - adjusted ");
    //DEBUGln(BATstr);
    
    readings+=reading;
  }
  
  batteryVolts = BATT_FORMULA(readings / 5.0);
  dtostrf(batteryVolts,3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
  //if (batteryVolts <= BATT_LOW) BATstr = "LOW";
  //DEBUG("Avg Batt: ");
  //DEBUGln(BATstr);
}

void Blink(byte PIN, byte DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS/2);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS/2);  
}

void printMillis(){
  static int lastMillis=0;
  currentMillis = millis();
  if(lastMillis == 0)
    lastMillis = currentMillis;
  DEBUG("Millis: ");
  DEBUGln(currentMillis - lastMillis);
  lastMillis = currentMillis;
}