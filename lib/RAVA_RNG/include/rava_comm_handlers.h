/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Defines the RAVA communication command identifiers and their associated message handler functions.

When a valid protocol message is received and successfully parsed, the handler associated with the
received command identifier is invoked.

By default, the comm_handlers lookup table is stored in both Flash and RAM memory. To reduce RAM
usage, the table may instead be stored in Flash memory only using PROGMEM. In this case, handler
function pointers must be retrieved using:
`comm_handler_t comm_handler_fn = (comm_handler_t)pgm_read_ptr(&comm_handlers[comm_msg.comm_id]);`
*/

#ifndef RAVA_COMM_HANDLERS_H
#define RAVA_COMM_HANDLERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include "rava_comm.h"

/* ===========================
 * RAVA COMM HANDLERS
 * =========================== */

#define RAVA_COMM_HANDLERS_COUNT 35

enum RAVA_COMM_IDS
{
  COMM_DEVICE_PING,
  COMM_DEVICE_GET_INFO,
  COMM_DEVICE_GET_USAGE,
  COMM_DEVICE_GET_FREE_RAM,
  COMM_DEVICE_GET_TEMPERATURE,
  COMM_DEVICE_GET_VCC,

  COMM_RNG_GET_CONFIG=10,
  COMM_RNG_SET_CONFIG,
  COMM_RNG_SET_TIMING_DEBUG,
  COMM_RNG_GEN_PULSE_COUNTS,
  COMM_RNG_GEN_BIT,
  COMM_RNG_GEN_BYTES,

  COMM_RNG_GEN_INT8S,
  COMM_RNG_GEN_INT16S,
  COMM_RNG_GEN_FLOATS,
  COMM_RNG_GEN_FLOATS_DOWNEY,

  COMM_RNG_START_BYTE_STREAM = 25,
  COMM_RNG_STOP_BYTE_STREAM,
  COMM_RNG_GET_STATUS_BYTE_STREAM,

  COMM_HEALTH_STARTUP_RUN = 30,
  COMM_HEALTH_STARTUP_GET_RESULTS,
  COMM_HEALTH_CONTINUOUS_GET_ERRORS,
};

typedef void (*comm_handler_t)(comm_interface_t *const comm);

extern const comm_handler_t rava_comm_handlers[RAVA_COMM_HANDLERS_COUNT];

void comm_device_ping(comm_interface_t *const comm);
void comm_device_get_info(comm_interface_t *const comm);
void comm_device_get_usage(comm_interface_t *const comm);
void comm_device_get_free_ram(comm_interface_t *const comm);
void comm_device_get_temperature(comm_interface_t *const comm);
void comm_device_get_vcc(comm_interface_t *const comm);

void comm_rng_get_config(comm_interface_t *const comm);
void comm_rng_set_config(comm_interface_t *const comm);
void comm_rng_set_timing_debug(comm_interface_t *const comm);
void comm_rng_gen_pulse_counts(comm_interface_t *const comm);
void comm_rng_gen_bit(comm_interface_t *const comm);
void comm_rng_gen_bytes(comm_interface_t *const comm);
void comm_rng_gen_int8s(comm_interface_t *const comm);
void comm_rng_gen_int16s(comm_interface_t *const comm);
void comm_rng_gen_floats(comm_interface_t *const comm);
void comm_rng_gen_floats_downey(comm_interface_t *const comm);
void comm_rng_start_byte_stream(comm_interface_t *const comm);
void comm_rng_stop_byte_stream(comm_interface_t *const comm);
void comm_rng_get_status_byte_stream(comm_interface_t *const comm);

void comm_health_startup_run(comm_interface_t *const comm);
void comm_health_startup_get_results(comm_interface_t *const comm);
void comm_health_continuous_get_errors(comm_interface_t *const comm);

/* ==============================
 * Implemented by the application
 * ============================== */

void comm_device_get_info(comm_interface_t *const comm);
void comm_device_get_free_ram(comm_interface_t *const comm);
void comm_device_get_temperature(comm_interface_t *const comm);
void comm_device_get_vcc(comm_interface_t *const comm);

#ifdef __cplusplus
}
#endif

#endif