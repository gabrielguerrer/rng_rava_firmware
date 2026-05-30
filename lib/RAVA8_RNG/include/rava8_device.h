/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Enables the retrieval of information related to the RAVA8 device.
*/

#ifndef RAVA8_DEVICE_H
#define RAVA8_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rava_comm.h>
#include <rava_device.h>

/* ===========================
 * RAVA8 DEVICE
 * =========================== */

// DEVICE
#define RAVA_MCU_ATMEGA32U4 1
#define RAVA_MODEL_RNG      1

#define RAVA_MCU    RAVA_MCU_ATMEGA32U4
#define RAVA_MODEL  RAVA_MODEL_RNG

// USB
#define RAVA_USB_VID          0x1209 // VID, PID from https://pid.codes
#define RAVA_USB_PID          0x4884
#define RAVA_USB_PRODUCT      L"RAVA8 RNG ∿"
#define RAVA_USB_MANUFACTURER L"Gabriel Guerrer"

// FIRMWARE VERSION
# define RAVA_FIRMWARE_VERSION_MAJOR 3
# define RAVA_FIRMWARE_VERSION_MINOR 0
# define RAVA_FIRMWARE_VERSION_PATCH 0

void device_calc_serial_number(void);
void device_delay_us(uint16_t us);

// COMM
void comm_device_get_info(comm_interface_t *const comm);
void comm_device_get_free_ram(comm_interface_t *const comm);
void comm_device_get_temperature(comm_interface_t *const comm);
void comm_device_get_vcc(comm_interface_t *const comm);

#ifdef __cplusplus
}
#endif

#endif