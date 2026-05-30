/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Defines the RAVA8 communication command identifiers and their associated message handler functions,
adding support for ATmega32U4 EEPROM and peripheral functionality.
*/

#ifndef RAVA8_COMM_HANDLERS_H
#define RAVA8_COMM_HANDLERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rava_comm.h>
#include <rava_comm_handlers.h>
#include "rava8_config.h"

/* ===========================
 * RAVA8 COMM HANDLERS
 * =========================== */

#ifdef PERIPHERALS_ENABLED
#define RAVA8_COMM_HANDLERS_COUNT 35
#else
#define RAVA8_COMM_HANDLERS_COUNT 10
#endif

enum RAVA8_COMM_IDS
{
  COMM_EEPROM_RESET_TO_DEFAULT = 35,
  COMM_EEPROM_RNG_GET_CONFIG,
  COMM_EEPROM_RNG_SET_CONFIG,

  COMM_PERIPH_DIGI = 45,

  COMM_PERIPH_D1_DIGI_FAST = 49,
  COMM_PERIPH_D1_TRIGGER_INPUT,
  COMM_PERIPH_D1_COMPARATOR,
  COMM_PERIPH_D1_DELAY_US_TEST,

  COMM_PERIPH_D2_TIMER3_INPUT_CAPTURE = 55,

  COMM_PERIPH_D3_TIMER3_TRIGGER_OUTPUT = 58,
  COMM_PERIPH_D3_TIMER3_PWM,
  COMM_PERIPH_D3_TIMER3_SOUND,

  COMM_PERIPH_D4_PIN_CHANGE = 64,

  COMM_PERIPH_D5_ADC = 67,
};

void comm_eeprom_reset_to_default(comm_interface_t *const comm);
void comm_eeprom_get_rng_config(comm_interface_t *const comm);
void comm_eeprom_set_rng_config(comm_interface_t *const comm);

#ifdef PERIPHERALS_ENABLED
void comm_periph_digi(comm_interface_t *const comm);
void comm_periph_d1_digi_fast(comm_interface_t *const comm);
void comm_periph_d1_trigger_input(comm_interface_t *const comm);
void comm_periph_d1_comparator(comm_interface_t *const comm);
void comm_periph_d1_device_delay_us_test(comm_interface_t *const comm);
void comm_periph_d2_timer3_input_capture(comm_interface_t *const comm);
void comm_periph_d3_timer3_periodic_trigger_output(comm_interface_t *const comm);
void comm_periph_d3_timer3_pwm(comm_interface_t *const comm);
void comm_periph_d3_timer3_sound(comm_interface_t *const comm);
void comm_periph_d4_pin_change(comm_interface_t *const comm);
void comm_periph_d5_adc(comm_interface_t *const comm);
#endif

extern const comm_handler_t rava8_comm_handlers[RAVA8_COMM_HANDLERS_COUNT];

#ifdef __cplusplus
}
#endif

#endif