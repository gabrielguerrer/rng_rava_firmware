/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Enables the retrieval of information related to the RAVA device.

Implementation-specific functionality must provide the hooks declared in the "Implemented by the
application" section.
*/

#ifndef RAVA_DEVICE_H
#define RAVA_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "rava_comm.h"

/* ===========================
 * RAVA DEVICE
 * =========================== */

#define SERIAL_NUMBER_N_BYTES 10

extern uint16_t device_request_count;
extern uint32_t device_gen_bytes_count;

typedef struct device_info_t
{
  uint8_t mcu, model;
  uint8_t firmw_ver_major, firmw_ver_minor, firmw_ver_patch, firmw_modules;
  uint16_t rng_gen_max_nbytes_per_core;
  uint8_t serial_number[SERIAL_NUMBER_N_BYTES];
} device_info_t;

// COMM
void comm_device_ping(comm_interface_t *const comm);
void comm_device_get_usage(comm_interface_t *const comm);

/* ==============================
 * Implemented by the application
 * ============================== */

// COMM
void comm_device_get_info(comm_interface_t *const comm);
void comm_device_get_free_ram(comm_interface_t *const comm);
void comm_device_get_temperature(comm_interface_t *const comm);
void comm_device_get_vcc(comm_interface_t *const comm);

#ifdef __cplusplus
}
#endif

#endif