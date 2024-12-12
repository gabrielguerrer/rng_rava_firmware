/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <avr/eeprom.h>

#include <rava_eeprom.h>
#include <rava_config.h>
#include <rava_comm.h>
#include <rava_device.h>
#include <rava_pwm_boost.h>
#include <rava_rng.h>
#include <rava_lamp.h>
#include <rava_tools.h>

#define EEPROM_SIZE 1024
#define EEPROM_EMPTY_N_TEST 20
#define EEPROM_EMPTY_VALUE 255 // Value assigned to EEPROM bytes after an erase operation

extern COMM* comm;
extern DEVICE* dev;
extern PWM_BOOST* pwm_boost;
extern RNG* rng;
extern LAMP* lamp;

EEPROM::EEPROM()
{
  // If EEPROM is empty (1st boot), write default information to it
  update_default();
}

void EEPROM::update_default()
{
  // Test if the first EEPROM bytes are empty
  bool all_empty = true;
  for (uint8_t i=0; i < EEPROM_EMPTY_N_TEST; i++) {
    uint8_t value = eeprom_read_byte((uint8_t*)i);
    if (value != EEPROM_EMPTY_VALUE) {
      all_empty = false;
      break;
    }
  }

  // Write default information if empty
  if (all_empty) {
    update_pwm_boost(DEFAULT_PWM_BOOST_FREQ_ID, DEFAULT_PWM_BOOST_DUTY);
    update_rng(DEFAULT_RNG_SAMPLING_INTERVAL_US);
    update_led(DEFAULT_LED_N);
    update_lamp(DEFAULT_LAMP_EXP_MOVWIN_N_TRIALS, DEFAULT_LAMP_EXP_DELTAHITS_SIGEVT, DEFAULT_LAMP_EXP_DURATION_MAX_S,
      DEFAULT_LAMP_FEDBMAG_SMOOTH_NTRIALS, DEFAULT_LAMP_FEDBMAG_COLORCHANGE_TRESHOLD, DEFAULT_LAMP_SOUND_VOLUME);
  }

  // Always update firmware
  update_firmware();
}

void EEPROM::erase()
{
  // Erase memory
  uint16_t empty_value = unpack_int(EEPROM_EMPTY_VALUE, EEPROM_EMPTY_VALUE);
  for (uint16_t i=0; i < EEPROM_SIZE/2; i++) {
    eeprom_update_word((uint16_t*)(2*i), empty_value);
  }
}

void EEPROM::read_firmware(uint8_t* version_major, uint8_t* version_minor, uint8_t* version_patch, uint8_t* parameters)
{
  *version_major = eeprom_read_byte((uint8_t*)EA_FIRMWARE_VERSION_MAJOR);
  *version_minor = eeprom_read_byte((uint8_t*)EA_FIRMWARE_VERSION_MINOR);
  *version_patch = eeprom_read_byte((uint8_t*)EA_FIRMWARE_VERSION_PATCH);
  *parameters = eeprom_read_byte((uint8_t*)EA_FIRMWARE_MODULES);
}

void EEPROM::update_firmware()
{
  // Firmware version
  eeprom_update_byte((uint8_t*)EA_FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MAJOR);
  eeprom_update_byte((uint8_t*)EA_FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_MINOR);
  eeprom_update_byte((uint8_t*)EA_FIRMWARE_VERSION_PATCH, FIRMWARE_VERSION_PATCH);

  // Firmware modules
  uint8_t firmw_modules = 0;

  #if defined(FIRMWARE_HEALTH_STARTUP_ENABLED)
  firmw_modules |= 1 << 0;
  #endif
  #if defined(FIRMWARE_HEALTH_CONTINUOUS_ENABLED)
  firmw_modules |= 1 << 1;
  #endif
  #if defined(FIRMWARE_LED_ENABLED)
  firmw_modules |= 1 << 2;
  #endif
  #if defined(FIRMWARE_LAMP_ENABLED)
  firmw_modules |= 1 << 3;
  #endif
  #if defined(FIRMWARE_PERIPHERALS_ENABLED)
  firmw_modules |= 1 << 4;
  #endif
  #if defined(FIRMWARE_COMM_SERIAL1_ENABLED)
  firmw_modules |= 1 << 5;
  #endif

  eeprom_update_byte((uint8_t*)EA_FIRMWARE_MODULES, firmw_modules);
}

void EEPROM::send_firmware()
{
  uint8_t version_major, version_minor, version_patch, parameters;
  read_firmware(&version_major, &version_minor, &version_patch, &parameters);

  comm->write_msg_header(COMM_EEPROM_FIRMWARE, version_major, version_minor, version_patch, parameters);
}

void EEPROM::read_pwm_boost(uint8_t* pwm_freq_id, uint8_t* pwm_duty)
{
  *pwm_freq_id = eeprom_read_byte((uint8_t*)EA_PWM_BOOST_FREQ_ID);
  *pwm_duty = eeprom_read_byte((uint8_t*)EA_PWM_BOOST_DUTY);
}

void EEPROM::update_pwm_boost(uint8_t pwm_freq_id, uint8_t pwm_duty)
{
  if (pwm_boost->validate_setup_pars(pwm_freq_id, pwm_duty)) {
    eeprom_update_byte((uint8_t*)EA_PWM_BOOST_FREQ_ID, pwm_freq_id);
    eeprom_update_byte((uint8_t*)EA_PWM_BOOST_DUTY, pwm_duty);
  }
}

void EEPROM::send_pwm_boost()
{
  uint8_t pwm_freq_id, pwm_duty;
  read_pwm_boost(&pwm_freq_id, &pwm_duty);

  comm->write_msg_header(COMM_EEPROM_PWM_BOOST, pwm_freq_id, pwm_duty);
}

void EEPROM::read_rng(uint8_t* sampling_interval)
{
  *sampling_interval = eeprom_read_byte((uint8_t*)EA_RNG_SAMPLING_INTERVAL);
}

void EEPROM::update_rng(uint8_t sampling_interval)
{
  if (rng->validate_setup_pars(sampling_interval)) {
    eeprom_update_byte((uint8_t*)EA_RNG_SAMPLING_INTERVAL, sampling_interval);
  }
}

void EEPROM::send_rng()
{
  uint8_t sampling_interval_us;
  read_rng(&sampling_interval_us);

  comm->write_msg_header(COMM_EEPROM_RNG, sampling_interval_us);
}

void EEPROM::read_led(uint8_t* led_n)
{
  *led_n = eeprom_read_byte((uint8_t*)EA_LED_N);
}

void EEPROM::update_led(uint8_t led_n)
{
  eeprom_update_byte((uint8_t*)EA_LED_N, led_n);
}

void EEPROM::send_led()
{
  uint8_t led_n;
  read_led(&led_n);

  comm->write_msg_header(COMM_EEPROM_LED, led_n);
}

void EEPROM::read_lamp(uint16_t* exp_movwin_n_trials, uint16_t* exp_deltahits_sigevt, uint16_t* exp_dur_max_s,
  uint8_t* exp_mag_smooth_n_trials, uint8_t* exp_mag_colorchg_thld, uint8_t* sound_volume)
{
  *exp_movwin_n_trials = eeprom_read_word((uint16_t*)EA_LAMP_EXP_MOVWIN_NTRIALS);
  *exp_deltahits_sigevt = eeprom_read_word((uint16_t*)EA_LAMP_EXP_DELTAHITS_SIGEVT);
  *exp_dur_max_s = eeprom_read_word((uint16_t*)EA_LAMP_EXP_DURATION_MAX_S);
  *exp_mag_smooth_n_trials = eeprom_read_byte((uint8_t*)EA_LAMP_FEDBMAG_SMOOTH_NTRIALS);
  *exp_mag_colorchg_thld = eeprom_read_byte((uint8_t*)EA_LAMP_FEDBMAG_COLORCHANGE_TRESHOLD);
  *sound_volume = eeprom_read_byte((uint8_t*)EA_LAMP_SOUND_VOLUME);
}

void EEPROM::update_lamp(uint16_t exp_movwin_n_trials, uint16_t exp_deltahits_sigevt, uint16_t exp_dur_max_s,
  uint8_t exp_mag_smooth_n_trials, uint8_t exp_mag_colorchg_thld, uint8_t sound_volume)
{
  if (lamp->validate_setup_pars(exp_movwin_n_trials, exp_deltahits_sigevt, exp_dur_max_s,
    exp_mag_smooth_n_trials, exp_mag_colorchg_thld, sound_volume)) {

    eeprom_update_word((uint16_t*)EA_LAMP_EXP_MOVWIN_NTRIALS, exp_movwin_n_trials);
    eeprom_update_word((uint16_t*)EA_LAMP_EXP_DELTAHITS_SIGEVT, exp_deltahits_sigevt);
    eeprom_update_word((uint16_t*)EA_LAMP_EXP_DURATION_MAX_S, exp_dur_max_s);
    eeprom_update_byte((uint8_t*)EA_LAMP_FEDBMAG_SMOOTH_NTRIALS, exp_mag_smooth_n_trials);
    eeprom_update_byte((uint8_t*)EA_LAMP_FEDBMAG_COLORCHANGE_TRESHOLD, exp_mag_colorchg_thld);
    eeprom_update_byte((uint8_t*)EA_LAMP_SOUND_VOLUME, sound_volume);
  }
}

void EEPROM::send_lamp()
{
  uint16_t exp_movwin_n_trials, exp_deltahits_sigevt, exp_dur_max_s;
  uint8_t exp_mag_smooth_n_trials, exp_mag_colorchg_thld, sound_volume;
  read_lamp(&exp_movwin_n_trials, &exp_deltahits_sigevt, &exp_dur_max_s, &exp_mag_smooth_n_trials, &exp_mag_colorchg_thld, &sound_volume);

  comm->write_msg_header(COMM_EEPROM_LAMP, exp_mag_smooth_n_trials, exp_mag_colorchg_thld, sound_volume, (uint8_t)6);
  comm->write(exp_movwin_n_trials);
  comm->write(exp_deltahits_sigevt);
  comm->write(exp_dur_max_s);
}