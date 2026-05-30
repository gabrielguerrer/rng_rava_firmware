/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <avr/eeprom.h>
#include <string.h>
#include "rava8_eeprom.h"
#include "rava8_pwm_boost.h"
#include "rava8_rng.h"

#define EEPROM_SIZE 1024
#define EEPROM_EMPTY_N_TEST 20
#define EEPROM_EMPTY_VALUE 255 // Value assigned to EEPROM bytes after an erase operation

static void eeprom_erase(void);
static bool eeprom_is_empty(void);
static void eeprom_reset_to_default(void);
static void eeprom_update_rng_cfg(const rng_config_t *const rng_cfg);

/* ===========================
 * RAVA8 EEPROM
 * =========================== */



/*
Initializes the EEPROM module. If the EEPROM is empty, it is populated with the default
configuration values.
*/
void eeprom_init(void)
{
  // If EEPROM is empty (1st boot), write default information to it
  if (eeprom_is_empty()) {
    eeprom_reset_to_default();
  }
}

/*
Erases EEPROM memory.
*/
static void eeprom_erase(void)
{
  // Erase memory
  uint16_t empty_value = (uint16_t)EEPROM_EMPTY_VALUE | (uint16_t)EEPROM_EMPTY_VALUE << 8;
  for (uint16_t i=0; i < EEPROM_SIZE/2; i++) {
    eeprom_update_word((uint16_t*)(2*i), empty_value);
  }
}

/*
Informs whether the EEPROM is empty.
*/
static bool eeprom_is_empty(void) {
  bool eeprom_empty = true;
  for (uintptr_t i=0; i < EEPROM_EMPTY_N_TEST; i++) {
    uint8_t value = eeprom_read_byte((const uint8_t*)i);
    if (value != EEPROM_EMPTY_VALUE) {
      eeprom_empty = false;
      break;
    }
  }
  return eeprom_empty;
}

/*
Restores the default configuration values in EEPROM.
*/
static void eeprom_reset_to_default(void)
{
  // RAVA String
  eeprom_update_byte((uint8_t*)EA_DEVICE_RAVA+0, 'R');
  eeprom_update_byte((uint8_t*)EA_DEVICE_RAVA+1, 'A');
  eeprom_update_byte((uint8_t*)EA_DEVICE_RAVA+2, 'V');
  eeprom_update_byte((uint8_t*)EA_DEVICE_RAVA+3, 'A');

  // RNG
  rng_config_t rng_cfg = {
    .pwm_boost = {.freq_id = DEFAULT_PWM_BOOST_FREQ_ID, .duty = DEFAULT_PWM_BOOST_DUTY},
    .sampling_interval = DEFAULT_RNG_SAMPLING_INTERVAL_US
  };

  eeprom_update_rng_cfg(&rng_cfg);
}

/*
Reads the RNG configuration parameters from EEPROM.
*/
void eeprom_read_rng_cfg(rng_config_t *const rng_cfg)
{
  rng_cfg->pwm_boost.freq_id = eeprom_read_byte((uint8_t*)EA_PWM_BOOST_FREQ_ID);
  rng_cfg->pwm_boost.duty = eeprom_read_byte((uint8_t*)EA_PWM_BOOST_DUTY);
  rng_cfg->sampling_interval = eeprom_read_byte((uint8_t*)EA_RNG_SAMPLING_INTERVAL);
}

/*
Writes the RNG configuration parameters to EEPROM.
*/
static void eeprom_update_rng_cfg(const rng_config_t *const rng_cfg)
{
  eeprom_update_byte((uint8_t*)EA_PWM_BOOST_FREQ_ID, rng_cfg->pwm_boost.freq_id);
  eeprom_update_byte((uint8_t*)EA_PWM_BOOST_DUTY, rng_cfg->pwm_boost.duty);
  eeprom_update_byte((uint8_t*)EA_RNG_SAMPLING_INTERVAL, rng_cfg->sampling_interval);
}

/* ===========================
 * COMM
 * =========================== */

/*
Processes the request to restore the default EEPROM configuration values.
*/
void comm_eeprom_reset_to_default(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {bool success;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  // Erase and Update
  eeprom_erase();
  eeprom_reset_to_default();

  // Force PWM and RNG to load EEPROM parameters
  data_out.success = true;
  data_out.success &= pwm_boost_setup_from_eeprom();
  data_out.success &= rng_setup_from_eeprom();

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/*
Processes the request to send the stored RNG configuration parameters.
*/
void comm_eeprom_get_rng_config(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {rng_config_t rng_cfg;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  eeprom_read_rng_cfg(&data_out.rng_cfg);

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/*
Processes the request to store the provided RNG configuration parameters in EEPROM.
*/
void comm_eeprom_set_rng_config(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {rng_config_t rng_cfg;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

// Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  if (!(pwm_boost_validate_setup_pars(&data_in.rng_cfg.pwm_boost) &&
        rng_validate_setup_pars(data_in.rng_cfg.sampling_interval) &&
        pwm_boost_setup(&data_in.rng_cfg.pwm_boost) &&  // Setup PWM
        rng_setup(data_in.rng_cfg.sampling_interval)    // Setup RNG
        )) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Update eeprom
  eeprom_update_rng_cfg(&data_in.rng_cfg);

  // Process Output
  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}