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

#include "FDC2x1x.h"

//bitmasks
static constexpr uint16_t FDC2x1x_CH0_UNREADCONV      = 0x0008;         //denotes unread CH0 reading in STATUS register
static constexpr uint16_t FDC2x1x_CH1_UNREADCONV      = 0x0004;         //denotes unread CH1 reading in STATUS register
static constexpr uint16_t FDC2x1x_CH2_UNREADCONV      = 0x0002;         //denotes unread CH2 reading in STATUS register
static constexpr uint16_t FDC2x1x_CH3_UNREADCONV      = 0x0001;         //denotes unread CH3 reading in STATUS register

static constexpr unsigned FDC2x1x_OSCILLATOR_SHIFT    = 9;
static constexpr uint16_t FDC2x1x_OSCILLATOR_MASK     = 0x1 << FDC2x1x_OSCILLATOR_SHIFT;
static constexpr unsigned FDC2x1x_SLEEP_SHIFT         = 13;
static constexpr uint16_t FDC2x1x_SLEEP_MASK          = 0x1 << FDC2x1x_SLEEP_SHIFT;
static constexpr uint16_t FDC2x1x_CLOCK_DIVIDER_MASK  = 0x3FF;
static constexpr unsigned FDC2x1x_SENSOR_FREQ_SHIFT   = 12;
static constexpr uint16_t FDC2x1x_SENSOR_FREQ_MASK    = 0x3 << FDC2x1x_SENSOR_FREQ_SHIFT;
static constexpr unsigned FDC2x1x_GAIN_SHIFT          = 9;
static constexpr uint16_t FDC2x1x_GAIN_MASK           = 0x3 << FDC2x1x_GAIN_SHIFT;

//registers
static constexpr uint16_t FDC2x1x_DEVICE_ID           = 0x7F;
static constexpr uint16_t FDC2x1x_MUX_CONFIG          = 0x1B;
static constexpr uint16_t FDC2x1x_CONFIG              = 0x1A;
static constexpr uint16_t FDC2x1x_RCOUNT_CH0          = 0x08;
static constexpr uint16_t FDC2x1x_OFFSET_CH0          = 0x0C;
static constexpr uint16_t FDC2x1x_SETTLECOUNT_CH0     = 0x10;
static constexpr uint16_t FDC2x1x_CLOCK_DIVIDERS_CH0  = 0x14;
static constexpr uint16_t FDC2x1x_STATUS              = 0x18;
static constexpr uint16_t FDC2x1x_DATA_CH0_MSB        = 0x00;
static constexpr uint16_t FDC2x1x_DATA_CH0_LSB        = 0x01;
static constexpr uint16_t FDC2x1x_DRIVE_CH0           = 0x1E;
static constexpr uint16_t FDC2x1x_RESET_DEV           = 0x1C;

// mask for 28bit data to filter out flag bits
static constexpr uint16_t FDC2x1x_DATA_CHx_MASK_DATA  = 0x0FFF;

static constexpr uint16_t FREQ_DIVIDER_DEFAULT        = 1;
static constexpr uint16_t SENSOR_FREQ_DEFAULT         = 2;

// index into this array is channel mask minus one
static const uint8_t AUTOSCAN_EN[] = {
  0, // msk=0x0001 // SingleChannel Ch0
  0, // msk=0x0010 // SingleChannel Ch1
  1, // msk=0x0011
  0, // msk=0x0100 // SingleChannel Ch2
  1, // msk=0x0101
  1, // msk=0x0110
  1, // msk=0x0111
  0, // msk=0x1000 // SingleChannel Ch3
  1, // msk=0x1001
  1, // msk=0x1010
  1, // msk=0x1011
  1, // msk=0x1100
  1, // msk=0x1101
  1, // msk=0x1110
  1, // msk=0x1111
};

// index into this array is channel mask minus one
static const uint8_t AUTOSCAN [] = {
  0x00,  // msk=0x0001 SingleChannel Ch0
  0x00,  // msk=0x0010 SingleChannel Ch1
  0x00,  // msk=0x0011 Ch0, Ch1
  0x01,  // msk=0x0100 SingleChannel Ch2
  0x01,  // msk=0x0101 Ch0, Ch2 -> Ch0, Ch1, Ch2
  0x01,  // msk=0x0110 Ch1, Ch2 -> Ch0, Ch1, Ch2
  0x01,  // msk=0x0111 Ch0, Ch1, Ch2
  0x10,  // msk=0x1000 SingleChannel Ch3
  0x10,  // msk=0x1001 Ch0, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1010 Ch1, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1011 Ch0, Ch1, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1100 Ch2, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1101 Ch0, Ch2, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1110 Ch1, Ch2, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1111 Ch0, Ch1, Ch2, Ch3 -> Ch0, Ch1, Ch2, Ch3
};

// index into this array is channel
static uint16_t UNREADCONV[] = {
    FDC2x1x_CH0_UNREADCONV,
    FDC2x1x_CH1_UNREADCONV,
    FDC2x1x_CH2_UNREADCONV,
    FDC2x1x_CH3_UNREADCONV,
};

/**************************************************************************/
/*!
    @file     NelsonsLog_FDC2x1x.cpp
    @author   Chris Nelson
	@license  BSD (see license.txt)

	This is a library for an FDC2x1x capacitive sensor
	----> http://www.nelsonsloxg.com

	@section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
static uint16_t channelMaskToChannel(uint8_t chanMask_) {
  uint16_t result = 0;
  while(chanMask_ & 1) {
    result++;
    chanMask_ >>= 1;
  }
  return result;
}

FDC2x1x::FDC2x1x(FDC2x1x_I2C_ADDR i2caddr, typeof(Wire)& wire) : _i2caddr(i2caddr), _wire(wire), _device(FDC2x1x_DEVICE_INVALID) {
}

const FDC2x1x_DEVICE FDC2x1x::readDeviceId() const {
  const int32_t device = read16FDC(FDC2x1x_DEVICE_ID);
  if(device >= 0) { // if no read error
    switch(device) {
      case FDC2x1x_DEVICE_FDC211x:
      case FDC2x1x_DEVICE_FDC221x:
        return static_cast<FDC2x1x_DEVICE>(device);
    }
  }
  return FDC2x1x_DEVICE_INVALID;
}

const FDC2x1x_DEVICE FDC2x1x::getDevice() const {
  if (_device == FDC2x1x_DEVICE_INVALID) {
    _device = readDeviceId();
  }
  return _device;
}

const size_t FDC2x1x::getChannelCount() const {
  switch (getDevice()) {
    case FDC2x1x_DEVICE_FDC211x:
    return 2;
  case FDC2x1x_DEVICE_FDC221x:
    return 4;
  }
  return 0;
}

FDC2x1x_DEVICE FDC2x1x::begin(uint8_t chanMask, bool enableSleepMode, FDC2x1x_DEGLITCH deglitchValue, bool useInternalOscillator, FDC2x1x_GAIN gain) {
  const FDC2x1x_DEVICE device = getDevice();
  const bool bOk = (device != FDC2x1x_DEVICE_INVALID);
  if (bOk) {
    if(loadSettings(chanMask, enableSleepMode, deglitchValue, useInternalOscillator, gain)) {
      return device;
    }
  }
  return FDC2x1x_DEVICE_INVALID;
}

bool FDC2x1x::loadChannelSettings(const uint16_t regOffset) {
  //settle count maximized, slow application
  write16FDC(FDC2x1x_SETTLECOUNT_CH0 + regOffset, 0x64);
  //rcount maximized for highest accuracy
  write16FDC(FDC2x1x_RCOUNT_CH0 + regOffset, 0xFFFF);
  //no offset
  write16FDC(FDC2x1x_OFFSET_CH0 + regOffset, 0x0000);

  // Set clock dividers
  //  Reserved
  //  | Sensor Frequency Select. b01 = /1 = sensor freq 0.01 to 8.75MHz; b10 = /2 = sensor freq 0.01 to 10 or 5 to 10 MHz
  //  | | Reserved
  //  | | |         Reference divider. Must be > 1. fref = fclk / this register`
  //  | | |         |
  // 00FF00RRRRRRRRRR -> 0010000000000001 -> 0x2001
  int32_t clockRegister = read16FDC(FDC2x1x_CLOCK_DIVIDERS_CH0 + regOffset);
  if(clockRegister >= 0) { // if no read error
    clockRegister &= FDC2x1x_SENSOR_FREQ_MASK;
    clockRegister |= SENSOR_FREQ_DEFAULT << FDC2x1x_SENSOR_FREQ_SHIFT;

    // Set divider to default, if not set yet.
    if((clockRegister & FDC2x1x_CLOCK_DIVIDER_MASK) == 0) {
      clockRegister |= FREQ_DIVIDER_DEFAULT;
    }

    write16FDC(FDC2x1x_CLOCK_DIVIDERS_CH0 + regOffset, clockRegister);

    //set drive register
    write16FDC(FDC2x1x_DRIVE_CH0 + regOffset, 0xF800);

    return true;
  }
  return false;
}

//Internal routine to do actual chip init
bool FDC2x1x::loadSettings(uint8_t chanMask, bool enableSleepMode, uint8_t deglitchValue, bool useInternalOscillator, FDC2x1x_GAIN gain) {

	//Configuration register
	//	Active channel Select: b00 = ch0; b01 = ch1; b10 = ch2; b11 = ch3;
	//  |Sleep Mode: 0 - device active; 1 - device in sleep;
	//  ||Reserved, reserved, set to 1
	//  |||Sensor Activation Mode: 0 - drive sensor with full current. 1 - drive sensor with current set by DRIVE_CURRENT_CHn
	//  ||||Reserved, set to 1
	//  |||||Reference clock: 0 - use internal; 1 - use external clock
	//  ||||||Reserved, set to 0
	//  |||||||Disable interrupt. 0 - interrupt output on INTB pin; 1 - no interrupt output
	//  ||||||||High current sensor mode: 0 - 1.5mA max. 1 - > 1.5mA, not available if Autoscan is enabled
	//  |||||||||Reserved, set to 000001
	//  ||||||||||
	// CCS1A1R0IH000001 -> 0001 1110 1000 0001 -> 0x1E81 	ExtOsc
	// CCS1A1R0IH000001 -> 0001 1100 1000 0001 -> 0x1C81	IntOsc

  const size_t n = getChannelCount();
  if(n > 0) {
    const uint8_t chanAvailableMask = (1 << n) - 1;
    const uint8_t chanMask_(chanMask & chanAvailableMask);
    const uint16_t autoScanEn = AUTOSCAN_EN[chanMask_-1];

    {
      const uint16_t singleScanChan = autoScanEn ? 0 : channelMaskToChannel(chanMask_);
      const uint16_t config = 0x1C81 | singleScanChan |
        (static_cast<uint16_t>(not useInternalOscillator) << FDC2x1x_OSCILLATOR_SHIFT) |
        (static_cast<uint16_t>(enableSleepMode) << FDC2x1x_SLEEP_SHIFT);
      write16FDC(FDC2x1x_CONFIG, config);  //set config
    }

    int32_t resetDev = read16FDC(FDC2x1x_RESET_DEV);
    {
      if(resetDev >= 0) { // if no read error
        resetDev &= ~FDC2x1x_GAIN_MASK;
        resetDev |= static_cast<uint16_t>(gain) << FDC2x1x_GAIN_SHIFT;
        write16FDC(FDC2x1x_RESET_DEV, resetDev);
      }
    }

    bool bOk = (resetDev >= 0);
    for(size_t c = 0; bOk && (c < getChannelCount()); c++) {
      //if channel c selected, init it..
      if (chanMask & (0x01 << c)) {
        bOk = loadChannelSettings(c);
      }
    }

    if(bOk) {
      // Autoscan: 0 = single channel, selected by CONFIG.ACTIVE_CHAN
      // | Autoscan sequence. b00 for chan 1-2, b01 for chan 1-2-3, b10 for chan 1-2-3-4
      // | |         Reserved - must be b0001000001
      // | |         |  Deglitch frequency. b001 for 1 MHz, b100 for 3.3 MHz, b101 for 10 Mhz, b111 for 33 MHz
      // | |         |  |
      // ARR0001000001DDD -> b0000 0010 0000 1000 -> h0208
      const uint16_t muxVal = 0x0208 | (autoScanEn << 15) | (static_cast<uint16_t>(AUTOSCAN[chanMask_-1]) << 13) | deglitchValue;
      //
      write16FDC(FDC2x1x_MUX_CONFIG, muxVal);  //set mux config for channels
    }

    return bOk;
  }

  // no channels available
  return false;
}

// inline because called once.
inline bool FDC2x1x::readMsbLsb(unsigned long& reading, const int addressMSB, const int addressLSB) const {
  const int32_t msb = read16FDC(addressMSB) & FDC2x1x_DATA_CHx_MASK_DATA;
  if(msb >= 0) { // if no read error
    const int32_t lsb = addressLSB < 0 ? 0 : read16FDC(addressLSB);
    if(lsb >= 0) { // if no read error
      reading = (static_cast<unsigned long>(msb)) << 16 | lsb;
      return true;
    }
  }
  return false;
}

// inline because called once
inline unsigned long FDC2x1x::getReading(uint8_t channel, int timeout, const int addressMSB, const int addressLSB) const {
  if(channel < getChannelCount()) {
    const uint16_t bitUnreadConv = UNREADCONV[channel];
    int32_t status = read16FDC(FDC2x1x_STATUS);
    while ((status >= 0) && timeout && !(status & bitUnreadConv)) {
          status = read16FDC(FDC2x1x_STATUS);
          timeout--;
    }

    if(status >= 0) { // if no read error
      if (timeout == 100) {
        // #####################################################################################################
        // There was this weird double read, as "first readout could be stale" in Nelsons file.
        // I have not confirmed the existence of this silicon bug.
        // I suspect that it might be due to crappy breadboard or rats nest wiring or lack of signal integrity for other reason
        //
        // On the other hand, I have done far too little testing to be sure, so I am leaving that bit in for now.
        //
        // #####################################################################################################

        //could be stale grab another //could it really it? ?????
        //read the 28 bit result
        unsigned long reading = FDC2x1x_INVALID_READING;
        if(not readMsbLsb(reading, addressMSB, addressLSB)) {
          status = -1;
        }

        while ((status >= 0) && timeout && !(status & bitUnreadConv)) {
            status = read16FDC(FDC2x1x_STATUS);
            timeout--;
        }
      }

      if(status >= 0) { // if no status read error
        if (timeout) {
          unsigned long reading = FDC2x1x_INVALID_READING;
          readMsbLsb(reading, addressMSB, addressLSB);
          return reading;
        } else {
          // Could not get data, chip readyness flag timeout
        }
      }
    }
  }
  return FDC2x1x_INVALID_READING;
}

// Takes in channel number, gives out the formatted 28 bit reading.
unsigned long FDC2x1x::getReading(uint8_t channel, int timeout) const {
  const int addressMSB = FDC2x1x_DATA_CH0_MSB + 2 * channel;
  const int addressLSB = getDevice() == FDC2x1x_DEVICE_FDC221x ? addressMSB + 1 : -1;
  return getReading(channel, timeout, addressMSB, addressLSB);
}

bool FDC2x1x::setOffset(uint8_t channel, uint16_t value) {
  if(channel < getChannelCount()) {
    write16FDC(FDC2x1x_OFFSET_CH0 + channel, value);
    return true;
  }
  return false;
}

bool FDC2x1x::setFrequencyDivider(uint8_t channel, uint16_t value) {
  if(channel < 4) {
    if(value > 0 && value <= FDC2x1x_CLOCK_DIVIDER_MASK) {
      const int sleepModeAlreadyEnabled = enableSleepMode();
      if(sleepModeAlreadyEnabled >= 0) {
        int32_t clockDivider = read16FDC(FDC2x1x_CLOCK_DIVIDERS_CH0 + channel);
        if(clockDivider >= 0) { // if no read error.
          clockDivider &= ~FDC2x1x_CLOCK_DIVIDER_MASK;
          clockDivider |= value;
          write16FDC(FDC2x1x_CLOCK_DIVIDERS_CH0 + channel, clockDivider);
          if(not sleepModeAlreadyEnabled) {
            return disableSleepMode() >= 0;
          }
        }
      }
    }
  }
  return false;
}

int FDC2x1x::isSleepModeEnabled()const {
  const int32_t config = read16FDC(FDC2x1x_CONFIG);
  if(config >=0) {
    return (config & FDC2x1x_SLEEP_MASK) != 0;
  }
  return config;
}

int FDC2x1x::enableSleepMode() {
  int32_t config = read16FDC(FDC2x1x_CONFIG);
  if(config >=0) {
    if((config & FDC2x1x_SLEEP_MASK) != 0) {
      // Already enabled.
      return true;
    }
    config |= FDC2x1x_SLEEP_MASK;
    write16FDC(FDC2x1x_CONFIG, config);
    return false;
  }
  return config;
}

int FDC2x1x::disableSleepMode() {
  int32_t config = read16FDC(FDC2x1x_CONFIG);
  if(config >= 0) {
    if((config & FDC2x1x_SLEEP_MASK) == 0) {
      // Already disabled.
      return false;
    }
    config &= ~FDC2x1x_SLEEP_MASK;
    write16FDC(FDC2x1x_CONFIG, config);
    return true;
  }
  return config;
}

/**************************************************************************/
/*!
    @brief  I2C low level interfacing
*/
/**************************************************************************/

inline int16_t FDC2x1x::waitFDC() const {
  int16_t data = -1;
  size_t retry = 0xffff;
  while (not _wire.available()) {
    if(not retry--) {
      break;
    }
  }

  if(retry) {
    data = _wire.read();
  }

  return data;
}

// Read 2 byte from the FDC at 'address'
int32_t FDC2x1x::read16FDC(int address) const {
    _wire.beginTransmission(_i2caddr);
    _wire.write(address);
    _wire.endTransmission(false); //restart
    _wire.requestFrom(_i2caddr, 2);
    int32_t data = -1;
    const int16_t dataMsb = waitFDC();

    if(dataMsb >= 0) {
      const int16_t dataLsb = waitFDC();
      if(dataLsb >= 0) {
        data = (dataMsb << 8) + dataLsb;
      }
    }
    _wire.endTransmission(true); //end
    return data;
}

// write 2 bytes to FDC  
void FDC2x1x::write16FDC(int address, uint16_t data) {
    _wire.beginTransmission(_i2caddr);
    _wire.write(address & 0xFF);
    _wire.write(data >> 8);
    _wire.write(data);
    _wire.endTransmission();
}
