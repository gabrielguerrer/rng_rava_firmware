/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <avr/io.h>
#include <string.h>
#include "rava8_pwm_boost.h"
#include "rava8_eeprom.h"
#include "rava8_timers.h"

static pwm_config_t _pwm_boost_cfg = {0};

/* ===========================
 * RAVA8 PWM_BOOST
 * =========================== */

/*
PWM Boost initialization: configures the module using parameters stored in EEPROM.
*/
bool pwm_boost_init(void)
{
  return pwm_boost_setup_from_eeprom();
}

/*
Validates the PWM boost configuration parameters.
*/
bool pwm_boost_validate_setup_pars(const pwm_boost_config_t *const pwm_boost_cfg)
{
  if (pwm_boost_cfg->freq_id >= PWM_BOOST_FREQ_LAST) {
    return false;
  }
  if (pwm_boost_cfg->duty == 0) {
    return false;
  }
  return true;
}

/*
Configures the PWM BOOST module by setting up Timer 4 according to the selected frequency defined
in the `PWM_BOOST_FREQUENCIES` enum.

For each supported frequency, the corresponding Timer 4 prescaler, TOP value, and maximum allowed
duty-cycle range are selected.
*/
bool pwm_boost_setup(const pwm_boost_config_t *const pwm_boost_cfg)
{
  // Find freq_prescaler, duty_max, and top
  uint8_t duty_max;

  if (pwm_boost_cfg->freq_id == PWM_BOOST_FREQ_30_KHZ) {
    _pwm_boost_cfg.freq_prescaler = TIMER4_CLK_DIV_8;
    _pwm_boost_cfg.top = 199;
    duty_max = 25;
  }
  else if (pwm_boost_cfg->freq_id == PWM_BOOST_FREQ_40_KHZ) {
    _pwm_boost_cfg.freq_prescaler = TIMER4_CLK_DIV_8;
    _pwm_boost_cfg.top = 149;
    duty_max = 25;
  }
  else if (pwm_boost_cfg->freq_id == PWM_BOOST_FREQ_50_KHZ) {
    _pwm_boost_cfg.freq_prescaler = TIMER4_CLK_DIV_4;
    _pwm_boost_cfg.top = 239;
    duty_max = 50;
  }
  else if (pwm_boost_cfg->freq_id == PWM_BOOST_FREQ_60_KHZ) {
    _pwm_boost_cfg.freq_prescaler = TIMER4_CLK_DIV_4;
    _pwm_boost_cfg.top = 199;
    duty_max = 50;
  }
  else if (pwm_boost_cfg->freq_id == PWM_BOOST_FREQ_75_KHZ) {
    _pwm_boost_cfg.freq_prescaler = TIMER4_CLK_DIV_4;
    _pwm_boost_cfg.top = 159;
    duty_max = 50;
  }
  else {
    return false;
  }

  if (pwm_boost_cfg->duty <= duty_max) {
    // Save Variables
    rng_cfg.pwm_boost = *pwm_boost_cfg;
    _pwm_boost_cfg.duty = pwm_boost_cfg->duty;

    // Configure Timer4
    timer4_setup_pwm_pb6(_pwm_boost_cfg.freq_prescaler, _pwm_boost_cfg.top, _pwm_boost_cfg.duty);
    return true;
  }
  else {
    return false;
  }
}

/*
Disables the PWM Boost functionality, restoring Timer 4 to its default inactive configuration.
*/
void pwm_boost_stop(void)
{
  timer4_reset();
}

// RAVA8 PWM_BOOST

/*
Configures the PWM Boost module using the configuration parameters stored in EEPROM.
*/
bool pwm_boost_setup_from_eeprom(void)
{
  // Load parameters from EEPROM
  rng_config_t rng_cfg;
  eeprom_read_rng_cfg(&rng_cfg);

  // Proceed with setup
  return pwm_boost_setup(&rng_cfg.pwm_boost);
}