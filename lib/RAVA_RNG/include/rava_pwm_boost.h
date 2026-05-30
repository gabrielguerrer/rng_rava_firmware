/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
In the RAVA circuit architecture, a PWM boost stage is used to increase the USB 5 V input to the
higher voltage applied to the reverse-biased Zener diodes, enabling the generation of avalanche
noise.

This module defines the methods that must be implemented by any RAVA implementation.
*/

#ifndef RAVA_PWM_BOOST_H
#define RAVA_PWM_BOOST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ===========================
 * RAVA PWM_BOOST
 * =========================== */

typedef struct pwm_boost_config_t
{
  uint8_t freq_id, duty;
} pwm_boost_config_t;

/* ==============================
 * Implemented by the application
 * ============================== */

bool pwm_boost_init(void);
bool pwm_boost_validate_setup_pars(const pwm_boost_config_t *const pwm_boost_cfg);
bool pwm_boost_setup(const pwm_boost_config_t *const pwm_boost_cfg);
void pwm_boost_stop(void);

#ifdef __cplusplus
}
#endif

#endif