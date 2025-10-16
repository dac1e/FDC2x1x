Library to drive FDC2112, FDC2114, FDC2212 and FDC2214

This library is based on based on https://github.com/zharijs/FDC2214 which originally I wanted to re-use for my somo1ELV project. However, the original library was missing some features that I needed. So I decided to create an own library and to add the missing features.
Missing features that were added:

- Selection of a different I2C TwoWire interface, in case the underlying hardware supports more than one. 
Solution: The Wire object to be used can be passed to the FDC2x1x constructor.

- Additional interface functions:
  - const FDC2x1x_DEVICE getDevice() const;
  - const size_t getChannelCount() const;
  - bool setFrequencyDivider(uint8_t channel, uint16_t value);
  - bool setOffset(uint8_t channel, uint16_t value);
  - int isSleepModeEnabled()const;
  - int enableSleepMode();
  - int disableSleepMode();

- Setting the FDC gain register. Solution: Can now be passed to the begin() function.

- The following topics are different compared to the original library.
  - There is no need anymore to pass the autoScan parameter to the begin() function. It will internally be calculated from the channel mask.
  - The 2 functions getReading16() and getReading28() have been collapsed to a single function getReading() that does the right reading automatically.
  - The begin() parameter deglitchValue has become an enumeration type to enhance code readibility.
  - Error handling has been improved. However it does work at best, when the underlying Wire library supports non blocking read and write functions which is unfortunately not always the case.
  - The begin function has got an additional parameter 'enableSleepMode' so that the FDC has not started working after the begin() function has returned. The FDC can be started later by calling disableSleepMode(). 
  
- Finally, code has been refactored to become more lean, to use more contemporary C++ code style like constexpr instead of #define, using static_cast operator and finally using const qualifiers, where possible.
 
