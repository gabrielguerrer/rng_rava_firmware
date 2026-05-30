/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
This module enables EEPROM support for persistent storage of RNG configuration parameters.
*/

#ifndef RAVA8_EEPROM_H
#define RAVA8_EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rava_rng.h>
#include <rava_comm.h>

/* ===========================
 * RAVA8 EEPROM
 * =========================== */

// DEFAULT VALUES used by eeprom_reset_to_default()
#define DEFAULT_PWM_BOOST_FREQ_ID         PWM_BOOST_FREQ_50_KHZ
#define DEFAULT_PWM_BOOST_DUTY            20
#define DEFAULT_RNG_SAMPLING_INTERVAL_US  10

enum EEPROM_ADDRESSES
{
  EA_DEVICE_RAVA,

  EA_PWM_BOOST_FREQ_ID = 4,
  EA_PWM_BOOST_DUTY,
  EA_RNG_SAMPLING_INTERVAL,
};

void eeprom_init(void);
void eeprom_read_rng_cfg(rng_config_t *const rng_cfg);

// COMM
void comm_eeprom_reset_to_default(comm_interface_t *const comm);
void comm_eeprom_get_rng_config(comm_interface_t *const comm);
void comm_eeprom_set_rng_config(comm_interface_t *const comm);

#ifdef __cplusplus
}
#endif

#endif