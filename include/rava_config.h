/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The configuration file serves three primary purposes by defining:

1. USB parameters with the values used by the operating system to identify the
device.

2. Preprocessor directives enabling specific capabilities. By deactivating
unnecessary functionalities, users can reduce flash memory consumption. The
space optimization allows the users to incorporate additional code addressing
their specific needs.

3. Default EEPROM parameter values. Those values are used to repopulate the
EEPROM in the event of a memory reset.
*/

#ifndef RAVA_CONFIG_H
#define RAVA_CONFIG_H

#include <rava_pwm.h>

/////////////////////////////
// HARDWARE

#define RAVA_HARDWARE_VERSION_1_0 // To be used in the future

/////////////////////////////
// USB CONFIG
// Info accessed by USBCore.cpp sent to the OS in the USB handshake
// VID, PID from https://pid.codes

#define RAVA_USB_PRODUCT "RAVA RNG"
#define RAVA_USB_MANUFACTURER "Gabriel Guerrer"
#define RAVA_USB_VID 0x1209
#define RAVA_USB_PID 0x4884

/////////////////////////////
// FIRMWARE

// VERSION
# define FIRMWARE_VERSION_MAJOR 1
# define FIRMWARE_VERSION_MINOR 0
# define FIRMWARE_VERSION_PATCH 0

// CONFIG
#define FIRMWARE_HEALTH_STARTUP_ENABLED // Enable health startup code
#define FIRMWARE_HEALTH_CONTINUOUS_ENABLED // Enable health continuous code
#define FIRMWARE_LED_ENABLED // Enable LED code
#define FIRMWARE_LAMP_ENABLED // Enable LAMP code
#define FIRMWARE_PERIPHERALS_ENABLED // Enable PERIPHERALS code
// #define FIRMWARE_COMM_SERIAL1_ENABLED // Enable SERIAL1 code

// INTERFACES
// #define FIRMWARE_INTERFACE_DS18B20_ENABLED // Enable DS18B20 INTERFACE code

/////////////////////////////
// RAVA DEFAULT PARAMETERS
// Written on the first boot to the EEPROM. See rava_eeprom.h/cpp

#define DEFAULT_DEVICE_TEMPERATURE_CALIBRATION_SLOPE 397
#define DEFAULT_DEVICE_TEMPERATURE_CALIBRATION_INTERCEPT -280

#define DEFAULT_PWM_FREQ_ID PWM_FREQ_50_KHZ
#define DEFAULT_PWM_DUTY 20

#define DEFAULT_RNG_SAMPLING_INTERVAL_US 10

#define DEFAULT_LAMP_EXP_DURATION_MAX_MS 300000 // 5 min
#define DEFAULT_LAMP_EXP_Z_SIGNIFICANT 3.925
#define DEFAULT_LAMP_FEDBMAG_SMOOTH_NTRIALS 20

#endif