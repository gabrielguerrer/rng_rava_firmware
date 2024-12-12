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

  EA_FIRMWARE_VERSION_MAJOR=5,
  EA_FIRMWARE_VERSION_MINOR,
  EA_FIRMWARE_VERSION_PATCH,
  EA_FIRMWARE_MODULES,

  EA_PWM_BOOST_FREQ_ID=10,
  EA_PWM_BOOST_DUTY,

  EA_RNG_SAMPLING_INTERVAL=15,

  EA_LED_N=20,

  EA_LAMP_EXP_MOVWIN_NTRIALS=25,
  EA_LAMP_EXP_DELTAHITS_SIGEVT=27,
  EA_LAMP_EXP_DURATION_MAX_S=29,
  EA_LAMP_FEDBMAG_SMOOTH_NTRIALS=35,
  EA_LAMP_FEDBMAG_COLORCHANGE_TRESHOLD=36,
  EA_LAMP_SOUND_VOLUME=40
};

class EEPROM {
  public:
    EEPROM();
    void update_default();
    void erase();

    void read_firmware(uint8_t* version_major, uint8_t* version_minor, uint8_t* version_patch, uint8_t* parameters);
    void update_firmware();
    void send_firmware();

    void read_pwm_boost(uint8_t* pwm_freq_id, uint8_t* pwm_duty);
    void update_pwm_boost(uint8_t pwm_freq_id, uint8_t pwm_duty);
    void send_pwm_boost();

    void read_rng(uint8_t* sampling_interval);
    void update_rng(uint8_t sampling_interval);
    void send_rng();

    void read_led(uint8_t* led_n);
    void update_led(uint8_t led_n);
    void send_led();

    void read_lamp(uint16_t* exp_movwin_n_trials, uint16_t* exp_deltahits_sigevt, uint16_t* exp_dur_max_s,
      uint8_t* exp_mag_smooth_n_trials, uint8_t* exp_mag_colorchg_thld, uint8_t* sound_volume);
    void update_lamp(uint16_t exp_movwin_n_trials, uint16_t exp_deltahits_sigevt, uint16_t exp_dur_max_s,
      uint8_t exp_mag_smooth_n_trials, uint8_t exp_mag_colorchg_thld, uint8_t sound_volume);
    void send_lamp();
};

#endif