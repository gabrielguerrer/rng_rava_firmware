/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include "rava_comm_handlers.h"

/* ===========================
 * RAVA8 COMM HANDLERS
 * =========================== */

const comm_handler_t rava_comm_handlers[RAVA_COMM_HANDLERS_COUNT] = {
  comm_device_ping,                     // 0
  comm_device_get_info,                 // 1
  comm_device_get_usage,                // 2
  comm_device_get_free_ram,             // 3
  comm_device_get_temperature,          // 4
  comm_device_get_vcc,                  // 5
  NULL,                                 // 6
  NULL,                                 // 7
  NULL,                                 // 8
  NULL,                                 // 9
  comm_rng_get_config,                  // 10
  comm_rng_set_config,                  // 11
  comm_rng_set_timing_debug,            // 12
  comm_rng_gen_pulse_counts,            // 13
  comm_rng_gen_bit,                     // 14
  comm_rng_gen_bytes,                   // 15
  comm_rng_gen_int8s,                   // 16
  comm_rng_gen_int16s,                  // 17
  comm_rng_gen_floats,                  // 18
  comm_rng_gen_floats_downey,           // 19
  NULL,                                 // 20
  NULL,                                 // 21
  NULL,                                 // 22
  NULL,                                 // 23
  NULL,                                 // 24
  comm_rng_start_byte_stream,           // 25
  comm_rng_stop_byte_stream,            // 26
  comm_rng_get_status_byte_stream,      // 27
  NULL,                                 // 28
  NULL,                                 // 29
  comm_health_startup_run,              // 30
  comm_health_startup_get_results,      // 31
  comm_health_continuous_get_errors,    // 32
  NULL,                                 // 33
  NULL,                                 // 34
};