/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include "rava8_comm_handlers.h"
#include "rava8_config.h"

/* ===========================
 * RAVA8 COMM HANDLERS
 * =========================== */

const comm_handler_t rava8_comm_handlers[RAVA8_COMM_HANDLERS_COUNT] = {
  comm_eeprom_reset_to_default,         // 35
  comm_eeprom_get_rng_config,           // 36
  comm_eeprom_set_rng_config,           // 37
  NULL,                                 // 38
  NULL,                                 // 39
  NULL,                                 // 40
  NULL,                                 // 41
  NULL,                                 // 42
  NULL,                                 // 43
  NULL,                                 // 44
  #ifdef PERIPHERALS_ENABLED
  comm_periph_digi,                     // 45
  NULL,                                 // 46
  NULL,                                 // 47
  NULL,                                 // 48
  comm_periph_d1_digi_fast,             // 49
  comm_periph_d1_trigger_input,         // 50
  comm_periph_d1_comparator,            // 51
  comm_periph_d1_device_delay_us_test,  // 52
  NULL,                                 // 53
  NULL,                                 // 54
  comm_periph_d2_timer3_input_capture,  // 55
  NULL,                                 // 56
  NULL,                                 // 57
  comm_periph_d3_timer3_periodic_trigger_output, // 58
  comm_periph_d3_timer3_pwm,            // 59
  comm_periph_d3_timer3_sound,          // 60
  NULL,                                 // 61
  NULL,                                 // 62
  NULL,                                 // 63
  comm_periph_d4_pin_change,            // 64
  NULL,                                 // 65
  NULL,                                 // 66
  comm_periph_d5_adc,                   // 67
  NULL,                                 // 68
  NULL,                                 // 69
  #endif
};