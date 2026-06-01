## v3.0.0

This release introduces a major rework of the firmware architecture. While the underlying 
randomness generation mechanism remains unchanged, the new design improves communication 
reliability through a more formal implementation of a framed binary protocol and enhances 
modularity by separating functionality into the `RAVA_RNG` and `RAVA8_RNG` libraries. 

The `RAVA_RNG` library implements the functionality shared by all hardware implementation of the
RAVA architecture, facilitating the development of RAVA-based devices across different 
microcontroller platforms. The `RAV8_RNG` library provides ATmega32U4-specific implementation and 
replaces the Arduino framework with the LUFA library, enabling lower-level hardware access and 
greater control over communication interfaces.

In addition, the entire project has been migrated from C++ to C, simplifying integration with other 
projects. The Arduino framework, previously used primarily to implement USB CDC communication, has 
been replaced by the LUFA library, providing lower-level hardware access and greater control over 
communication interfaces.

### New
- Introduces the `RAVA_RNG` and `RAVA8_RNG` libraries
- Implements a framed binary communication protocol with byte-wise state parsing
- Added protocol-level error reporting through dedicated `COMM_ERROR_IDS` codes
- Migrated from the Arduino framework to the LUFA library
- Added support for both USB CDC and USART communication interfaces through a unified abstraction 
  layer
- Introduced floating-point number generation using Allen Downey's algorithm

### Changed
- Reorganized the firmware into platform-independent and platform-specific libraries
- Migrated the codebase from C++ to C
- Replaced switch-case command parsing with a lookup-table handler architecture
- Improved code documentation, with descriptive comments added throughout the codebase
- Dedicated Timer 0 exclusively to randomness generation
- Unified EEPROM configuration of PWM boost and RNG sampling size into a single command
- Removed Von Neumann post-processing, which was unnecessarily conservative for the RAVA 
  architecture
- Removed the LED and LAMP modules; these will now be maintained in a separate repository

### Fixed
- Corrected a bug in floating-point number generation where only two random bytes were used instead 
  of the required three
- Improved the efficiency of int8 and int16 random number generation


## v2.0.0

This version improves LED and LAMP functionalities while retaining the RNG behavior unchanged from 
version v1.0.0

- LED upgrades
    - The EEPROM LED variable was updated to represent the quantity of LEDs instead of merely 
      indicating their presence
    - Updated set_color() to incrementally activate sequential LEDs as intensity increases, 
      improving mixed color performance at low intensity levels

- TIMERS upgrades
    - Expanded on interrupt and PWM capabilities of Timer1

- PERIPHERALS upgrades
    - Added the setup_timer3_sound() function to generate sound frequencies through the D3 port    

- LAMP upgrades    
    - Fixed a bug where RNG functions initialize() and finalize() where not being called before and 
      after random byte measurements
    - Now utilize Timer3 to establish a 20 FPS tick clock, replacing the previous reliance on the 
      WDT
    - Moving window algorithm now utilizes a dynamically allocated array
    - Added the option to play a sound melody during color-oscillating events. This feature 
      temporarily utilizes Timer1, restoring it to its standard pulse counting functionality 
      afterward
    - Introduced an option for changing color according to the trial_mag value

- EEPROM upgrades
    - Include the new variables used in the LAMP module

- Other improvements
    - rava_health: LED blue/purple colors were removed during ongoing tests, leaving only a red 
      light to indicate failures
    - rava_tools: array_init() and array_sum() array_size var changed from uint8_t to uint16_t
    - rava_device: Removed temperature functionality as it required calibration for each device
    - Added brackets to all if statements for improved consistency    