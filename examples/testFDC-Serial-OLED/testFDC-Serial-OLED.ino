/*
  FDC2x1x - Arduino libary for driving the Texas Instruments FDC2112,FDC2114,FDC2212,FDC2214 chips Copyright (c)
  2025 Wolfgang Schmieder.  All right reserved.

  Based on https://github.com/zharijs/FDC2214

  Contributors:
  - Wolfgang Schmieder

  Project home: https://github.com/dac1e/FDC2x1x/

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
// Uses SSD1306Ascii library to display result on 0.96 inch ebay OLED screen
// Transmits data via serial - use SerialPlot to draw graphs
// 
// FDC2x1x hardware configuration:
// Component value as in default circuit form datasheet. (18uH inductor and 33pF cap)
// 
// SD and ADDR pins tied to GND
// INTB pin not used
// 
// OLED <--> ARDUINO <--> FDC
// SDA <---> A4 <-------> SDA
// SCL <---> A5 <-------> SCL
// 
// !!!!!! Arduinos are mostly 5V. FDC chips are 3.3V, so either use 3.3V version of Arduino, like pro mini, or use level shifter on I2C bus.
// 

// ### OLED.
#include <Arduino.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
constexpr uint8_t I2C_ADDRESS = 0x3C;
constexpr int RST_PIN = -1;
SSD1306AsciiWire oled;

// ### FDC
#include "FDC2x1x.h"
FDC2x1x capsense(FDC2x1x_I2C_ADDR_0, Wire); // Use FDC2x1x_I2C_ADDR_0

// ### Tell application the maximum of number of channels.
static constexpr size_t CHAN_COUNT_MAX = 4;

unsigned long sensorThreshold[2]; // threshold for proximity detection

// ### Local function prototypes 
int iicScan(); // check for attached devices on I2C bus

// ###
void setup() {
  
  // ### Start I2C 
  Wire.begin();
//  Wire.setClock(400000L);
  
  // ### Start serial
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
  
  // ### Start OLED display
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.println("Intelitech ");
  int devices = iicScan();
  oled.print(":");
  oled.print(devices);
  oled.println("devs");

  // ### Start FDC

//  // Start FDC2x1x with at max 2 channels init, external oscillator
//  const FDC2x1x_DEVICE device = capsense.begin(0x3, false, FDC2x1x_DEGLITCH_10Mhz, false, FDC2x1x_GAIN_1); //setup first two channels, don't stay in sleep mode, deglitch at 10MHz, external oscillator

//  // Start FDC2x1x with at max 4 channels init, internal oscillator
//  const FDC2x1x_DEVICE device = capsense.begin(0xF, false, FDC2x1x_DEGLITCH_10Mhz, true, FDC2x1x_GAIN_1); //setup all four channels, don't stay in sleep mode, deglitch at 10MHz, internal oscillator

  // Start FDC2x1x with at max 4 channels init, external oscillator
  const FDC2x1x_DEVICE device = capsense.begin(0xF, false, FDC2x1x_DEGLITCH_10Mhz, false, FDC2x1x_GAIN_1); //setup all four channels, don't stay in sleep mode, deglitch at 10MHz, external oscillator

  if (device != FDC2x1x_DEVICE_INVALID) {
    oled.println("Sensor OK");
  } else {
    oled.println("Sensor Fail");
  }

  sensorThreshold[0] = 14000000+320000;
  sensorThreshold[1] = 320000;

};

// ###
void loop() {
  unsigned long capa[CHAN_COUNT_MAX]; // variable to store data from FDC
  const size_t n = capsense.getChannelCount();
  for (int i = 0; i < n; i++){ // for each channel
    // ### read 28bit data
    capa[i]= capsense.getReading(i);
    // ### jump cursor, set font and display channel data on OLED
    oled.setCursor(0,4+(2*i)); 
    oled.set2X();
    oled.print(capa[i]);
    oled.print("  ");
    oled.set1X();
    // ### Transmit data to serial in simple format readable by SerialPlot application.
    Serial.print(capa[i]);  
    if (i < n-1)
      Serial.print(", ");
    else
      Serial.println("");
    // ### display proximity threshold on OLED 
    oled.setCursor(110,4+(2*i));
    if (capa[i] < sensorThreshold[i]){
      oled.set2X();
      oled.print("*");
      oled.set1X();
    } else {
      oled.set2X();
      oled.print(" ");
      oled.set1X();      
    }
  }
  // No point in sleeping
  //delay(100); 
}

//
// ### scan for I2C devices. This code fragment comes from I2C scan example.
int iicScan (){
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    //
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      oled.print("0x");
      oled.print(address,HEX);
      oled.print(" ");
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  return nDevices;
  }
