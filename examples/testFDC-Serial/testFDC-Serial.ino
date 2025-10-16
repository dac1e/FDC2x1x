/*
  FDC2x1x - Arduino libary for driving the Texas Instruments FDC2112,FDC2114,FDC2212,FDC2214 chips Copyright (c)
  2025 Wolfgang Schmieder.  All right reserved.

  Based on https://github.com/zharijs/FDC2214

  Contributors:
  - Wolfgang Schmieder

  Project home: https://github.com/dac1e/Somo1ELV/

  This library is free software; you can redistribute it and/or modify it
  the terms of the GNU Lesser General Public License as under published
  by the Free Software Foundation; either version 3.0 of the License,
  or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
*/

//
// Supported chips: FDC2112, FDC2114, FDC2212, FDC2214
// Transmits data via serial - use SerialPlot to draw graphs
// 
// FDC2x1x hardware configuration:
// Component value as in default circuit from datasheet. (18uH inductor and 33pF cap)
// 
// SD and ADDR pins tied to GND
// INTB pin not used
// 
// ARDUINO <--> FDC
// A4 <-------> SDA
// A5 <-------> SCL
// 
// !!!!!! Arduinos are mostly 5V. FDC chips are 3.3V, so either use 3.3V version of Arduino, like pro mini, or use level shifter on I2C bus.
//

// ### FDC
#include <Arduino.h>
#include <Wire.h>
#include "FDC2x1x.h"

FDC2x1x capsense(FDC2x1x_I2C_ADDR_0, Wire); // Use FDC2x1x_I2C_ADDR_0

// ### Tell application the maximum of number of channels.
static constexpr size_t CHAN_COUNT_MAX = 4;


// ###
void setup() {
  
  // ### Start I2C 
  Wire.begin();
//  Wire.setClock(400000L);
  
  // ### Start serial
  Serial.begin(115200);
  Serial.println("\nFDC2x1x test");
  
  // ### Start FDC

//  // Start FDC2x1x with at max 2 channels init, external oscillator
//  const FDC2x1x_DEVICE device = capsense.begin(0x3, false, FDC2x1x_DEGLITCH_10Mhz, false, FDC2x1x_GAIN_1); //setup first two channels, don't stay in sleep mode, deglitch at 10MHz, external oscillator, gain=1

//  // Start FDC2x1x with at max 4 channels init, internal oscillator
//  const FDC2x1x_DEVICE device = capsense.begin(0xF, false, FDC2x1x_DEGLITCH_10Mhz, true, FDC2x1x_GAIN_1); //setup all four channels, don't stay in sleep mode, deglitch at 10MHz, internal oscillator, gain=1

  // Start FDC2x1x with at max 4 channels init, external oscillator
  const FDC2x1x_DEVICE device = capsense.begin(0xF, false, FDC2x1x_DEGLITCH_10Mhz, false, FDC2x1x_GAIN_1); //setup all four channels, don't stay in sleep mode, deglitch at 10MHz, external oscillator, gain=1

  if (device != FDC2x1x_DEVICE_INVALID) {
    Serial.println("Sensor OK");
  } else {
    Serial.println("Sensor Fail");
  }
}

// ### 
void loop() {
  unsigned long capa[CHAN_COUNT_MAX]; // variable to store data from FDC
  const size_t n = capsense.getChannelCount();
  for (int i = 0; i < n; i++){ // for each channel
    // ### read 28bit data
    capa[i]= capsense.getReading(i);//
    // ### Transmit data to serial in simple format readable by SerialPlot application.
    Serial.print(capa[i]);  
    if (i < n-1) Serial.print(", ");
    else Serial.println("");
  }
  // No point in sleeping
  //delay(100); 
}


