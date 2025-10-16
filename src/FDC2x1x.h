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

#pragma once

#ifndef FDC2x1x_API_H_
#define FDC2x1x_API_H_

#include <stdint.h>
#include <Wire.h>

enum FDC2x1x_DEVICE : uint16_t {
  FDC2x1x_DEVICE_INVALID = 0x0000,
  FDC2x1x_DEVICE_FDC211x = 0x3054, // FDC2112 or FDC2114
  FDC2x1x_DEVICE_FDC221x = 0x3055, // FDC2212 or FDC2214
};

// Address is 0x2A (default) or 0x2B (if ADDR is high)
enum FDC2x1x_I2C_ADDR : uint8_t {
  FDC2x1x_I2C_ADDR_0 = 0x2A,
  FDC2x1x_I2C_ADDR_1 = 0x2B,
};

enum FDC2x1x_DEGLITCH : uint8_t {
  FDC2x1x_DEGLITCH_1000Khz = 1,
  FDC2x1x_DEGLITCH_3300Khz = 4,
  FDC2x1x_DEGLITCH_10Mhz   = 5,
  FDC2x1x_DEGLITCH_33Mhz   = 6,
};

enum FDC2x1x_GAIN : uint16_t {
  FDC2x1x_GAIN_1  = 0,
  FDC2x1x_GAIN_4  = 1,
  FDC2x1x_GAIN_8  = 2,
  FDC2x1x_GAIN_16 = 3,
};

// This value is returned by function FDC2x1x::getReading() in case of an error.
static constexpr unsigned long FDC2x1x_INVALID_READING = 0xffffffff;

// Refer to Texas Instruments FDC2212, FDC2214, FDC2112, FDC2114 documentation
class FDC2x1x {
public:
    // Local Alias for FDC2x1x_INVALID_READING.
    static constexpr unsigned long INVALID_READING = FDC2x1x_INVALID_READING;

    // For using a different I2C interface pass Wire2, Wire3... as second parameter
    // if supported by the Arduino board.
    FDC2x1x(FDC2x1x_I2C_ADDR i2caddr, typeof(Wire)& wire = Wire);

    FDC2x1x_DEVICE begin(uint8_t channelMask, bool enableSleepMode = false,
        FDC2x1x_DEGLITCH deglitchValue = FDC2x1x_DEGLITCH_33Mhz,
        bool useInternalOscillator = true, FDC2x1x_GAIN gain = FDC2x1x_GAIN_1);

    const FDC2x1x_DEVICE getDevice() const;

    // Return 2 for FDC2x12 and 4 for FDC1x14
    const size_t getChannelCount() const;

    // Return true on success: Otherwise false.
    bool setFrequencyDivider(uint8_t channel, uint16_t value);

    // Return true on success: Otherwise false.
    // Valid channel = 0..2 on FDC2x12 and 0..3 on FDC1x14
    bool setOffset(uint8_t channel, uint16_t value);

    // Return 1, if sleep mode is enabled. return 0, if sleep mode is disabled. return -1 upon I2C read error.
    int isSleepModeEnabled()const;

    // Return sleep mode before that call or -1 upon I2C read error.
    int enableSleepMode();

    // Return sleep mode before that call or -1 upon I2C read error.
    int disableSleepMode();

    // Gives out the conversion result as 28 bit value for all FDC2x1x devices,
    // in particular also as 28 bit value for FDC212x.
    // Returns FDC2x1x_INVALID_READING in case of an error.
    unsigned long getReading(uint8_t channel, int timeout = 128) const;

    // Return the wire object that is used for I2C communication.
    typeof(Wire)& getWire() {return _wire;}

private:
    unsigned long getReading(uint8_t channel, int timeout, const int addressMSB,
        const int addressLSB) const;

    bool loadSettings(uint8_t chanMask, bool enableSleepMode, uint8_t deglitchValue,
        bool useInternalOscillator, FDC2x1x_GAIN gain);

    bool loadChannelSettings(const uint16_t regOffset);

    const FDC2x1x_DEVICE readDeviceId() const;

    bool readMsbLsb(unsigned long& reading, const int addressMSB, const int addressLSB) const;

    // return -1 if read failed. Otherwise return the read value in the range of 0..0xFFFF
    int32_t read16FDC(int address) const;

    void write16FDC(int address, uint16_t data);

    // return -1 if wait expired. Otherwise the value of the read byte.
    int16_t waitFDC() const;

    FDC2x1x_I2C_ADDR _i2caddr;
    typeof(Wire)& _wire;
    mutable FDC2x1x_DEVICE _device;
};

#endif // #ifndef FDC2x1x_API_H_
