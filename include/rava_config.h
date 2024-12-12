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

#include <rava_pwm_boost.h>

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
# define FIRMWARE_VERSION_MAJOR 2
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
// Written on the first boot to the EEPROM (see rava_eeprom.h/cpp)

#define DEFAULT_PWM_BOOST_FREQ_ID PWM_BOOST_FREQ_50_KHZ
#define DEFAULT_PWM_BOOST_DUTY 20

#define DEFAULT_RNG_SAMPLING_INTERVAL_US 10

#define DEFAULT_LED_N 16

#define DEFAULT_LAMP_EXP_MOVWIN_N_TRIALS 600 // Last 30s
#define DEFAULT_LAMP_EXP_DELTAHITS_SIGEVT 94 // Targeting 6 evt per hour
#define DEFAULT_LAMP_EXP_DURATION_MAX_S 300 // 5min
#define DEFAULT_LAMP_FEDBMAG_SMOOTH_NTRIALS 70
#define DEFAULT_LAMP_FEDBMAG_COLORCHANGE_TRESHOLD 207 // Start changing color when |z|>1.315
#define DEFAULT_LAMP_SOUND_VOLUME 153 // 60%

#endif