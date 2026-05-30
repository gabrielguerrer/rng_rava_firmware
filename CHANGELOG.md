## v3.0.0

This release introduces a major rework of the firmware architecture. While the underlying 
randomness generation mechanism remains unchanged, the new design improves communication 
reliability through a more formal implementation of a framed binary protocol and enhances  
modularity by moving generic functionality into the RAVA_RNG library. 

The RAVA_RNG library provides the features shared across different implementations of the RAVA 
architecture, facilitating the development of RAVA-based devices on other microcontroller 
platforms.

Code specific to the ATmega32U4 implementation is identified with the rava8 prefix, referring to 
the microcontroller’s 8-bit architecture.

Additionally, the firmware now uses the LUFA Library instead of the Arduino framework, providing 
lower-level hardware access and greater control over communication interfaces.

Finally, the project was migrated from C++ to C, facilitating integration into other projects.

### New
- Introduces the RAVA_RNG library, which provides the functionality shared across different 
  hardware implementations of the RAVA architecture
- Implements a framed binary communication protocol with state parser evaluated per byte 
- The protocol now reports different COMM_ERROR_IDS errors that may occur during the communication 
  decoding process
- Replaces the previous Arduino-based implementation with a lower-level architecture based on the 
  LUFA library

### Changed
- Renamed the ATmega32U4 implementation to RAVA8
- Migrated from C++ to C language
- USB CDC and USART communication interfaces implemented through LUFA
- Removed the LED and LAMP modules, which will be maintained in a separate repository
- Command parsing now uses lookup-table handlers instead of switch-case conditions
- Documentation improved, with each function now accompanied by a descriptive comment

- Timer 0 fully dedicated
- ADC operated via LUFA
- device provides temperature and Vcc
- EEPROM: pwm boost and rng sampling size united in one single command
- Removed Von Neumann post-processing (slow and overly conservative)

### Fixed
- Fixed a bug in floating-point number generation where only 2 random bytes were previously used 
  instead of the required 3
- More efficient code for int8 and int16 generation


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