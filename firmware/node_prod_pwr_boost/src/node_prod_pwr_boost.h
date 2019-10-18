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
#include <Arduino.h>

void enablePowerBoost();
void disablePowerBoost();
void readBattery();
void Blink(uint8_t PIN, uint8_t DELAY_MS);