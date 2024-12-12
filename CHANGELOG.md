## v2.0.0

This version improves LED and LAMP functionalities while retaining the RNG behavior unchanged from version v1.0.0

- LED upgrades
    - The EEPROM LED variable was updated to represent the quantity of LEDs instead of merely indicating their presence
    - Updated set_color() to incrementally activate sequential LEDs as intensity increases, improving mixed color performance at low intensity levels

- TIMERS upgrades
    - Expanded on interrupt and PWM capabilities of Timer1

- PERIPHERALS upgrades
    - Added the setup_timer3_sound() function to generate sound frequencies through the D3 port    

- LAMP upgrades    
    - Fixed a bug where RNG functions initialize() and finalize() where not being called before and after random byte measurements
    - Now utilize Timer3 to establish a 20 FPS tick clock, replacing the previous reliance on the WDT
    - Moving window algorithm now utilizes a dynamically allocated array
    - Added the option to play a sound melody during color-oscillating events. This feature temporarily utilizes Timer1, restoring it to its standard pulse counting functionality afterward
    - Introduced an option for changing color according to the trial_mag value

- EEPROM upgrades
    - Include the new variables used in the LAMP module

- Other improvements
    - rava_health: LED blue/purple colors were removed during ongoing tests, leaving only a red light to indicate failures
    - rava_tools: array_init() and array_sum() array_size var changed from uint8_t to uint16_t
    - rava_device: Removed temperature functionality as it required calibration for each device
    - Added brackets to all if statements for improved consistency