/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The PWM BOOST module configures Timer4 to generate the PWM signal on port B6 that drives the boost
converter module responsible for increasing the USB 5V input into the higher voltage applied to the
reverse-biased Zeners noise sources.

The PWM configuration is defined by two parameters: the PWM frequency and the PWM duty cycle
(ranging from 0 to 255). During initialization, these values are loaded from EEPROM memory. They
may also be reconfigured during runtime through the COMM_RNG_SET_CONFIG command.
*/

#ifndef RAVA8_PWM_BOOST_H
#define RAVA8_PWM_BOOST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rava_pwm_boost.h>

/* ===========================
 * RAVA8 PWM_BOOST
 * =========================== */

enum PWM_BOOST_FREQUENCIES
{
  PWM_BOOST_FREQ_30_KHZ,  // Timer4 clk/8, 200 ticks
  PWM_BOOST_FREQ_40_KHZ,  // Timer4 clk/8, 150 ticks
  PWM_BOOST_FREQ_50_KHZ,  // Timer4 clk/4, 240 ticks
  PWM_BOOST_FREQ_60_KHZ,  // Timer4 clk/4, 200 ticks
  PWM_BOOST_FREQ_75_KHZ,  // Timer4 clk/4, 160 ticks
  PWM_BOOST_FREQ_LAST
};

// RAVA PWM_BOOST
bool pwm_boost_init(void);
bool pwm_boost_validate_setup_pars(const pwm_boost_config_t *const pwm_boost_cfg);
bool pwm_boost_setup(const pwm_boost_config_t *const pwm_boost_cfg);
void pwm_boost_stop(void);

// RAVA8 PWM_BOOST
bool pwm_boost_setup_from_eeprom(void);

#ifdef __cplusplus
}
#endif

#endif