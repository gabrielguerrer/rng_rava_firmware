/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

/*
The EEPROM class facilitates the storage and retrieval of data in permanent 
memory. This file specifies the EEPROM addresses where stored information is 
located and provides methods for reading, updating, and transmitting this data.
*/

#ifndef RAVA_EEPROM_H
#define RAVA_EEPROM_H

#include <stdint.h>

enum EEPROM_ADDRESSES {
  EA_DEVICE_RAVA,
  EA_DEVICE_TEMPERATURE_CALIBRATION_SLOPE=4,
  EA_DEVICE_TEMPERATURE_CALIBRATION_INTERCEPT=6,

  EA_FIRMWARE_VERSION_MAJOR=8,
  EA_FIRMWARE_VERSION_MINOR,
  EA_FIRMWARE_VERSION_PATCH,
  EA_FIRMWARE_MODULES,

  EA_PWM_FREQ_ID,
  EA_PWM_DUTY,

  EA_RNG_SAMPLING_INTERVAL,

  EA_LED_ATTACHED,

  EA_LAMP_EXP_DURATION_MAX_MS,
  EA_LAMP_EXP_Z_SIGNIFICANT=21,
  EA_LAMP_FEDBMAG_SMOOTH_NTRIALS=25
};

class EEPROM {
  public:
    EEPROM();
    void update_default();
    void erase();

    void read_device(uint16_t* temp_calib_slope, int16_t* temp_calib_intercept);
    void update_device(uint16_t temp_calib_slope, int16_t temp_calib_intercept);
    void send_device();

    void read_firmware(uint8_t* version_major, uint8_t* version_minor, uint8_t* version_patch, uint8_t* parameters);
    void update_firmware();
    void send_firmware();

    void read_pwm(uint8_t* pwm_freq_id, uint8_t* pwm_duty);
    void update_pwm(uint8_t pwm_freq_id, uint8_t pwm_duty);
    void send_pwm();

    void read_rng(uint8_t* sampling_interval);
    void update_rng(uint8_t sampling_interval);
    void send_rng();

    void read_led(uint8_t* led_attached);
    void update_led(uint8_t led_attached);
    void send_led();

    void read_lamp(uint32_t* exp_dur_max_ms, float* exp_z_significant, uint8_t* exp_mag_smooth_n_trials);
    void update_lamp(uint32_t exp_dur_max_ms, float exp_z_significant, uint8_t exp_mag_smooth_n_trials);
    void send_lamp();
};

#endif