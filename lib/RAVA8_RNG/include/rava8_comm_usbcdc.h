/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Defines the USB CDC transport implementation for RAVA8, based on the LUFA library.
*/

#ifndef RAVA8_COMM_USBCDC_H
#define RAVA8_COMM_USBCDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <LUFA/Drivers/USB/USB.h>
#include <rava_comm.h>

/* ===========================
 * RAVA8 COMM USBCDC
 * =========================== */

extern USB_ClassInfo_CDC_Device_t lufa_usbcdc_if;
extern comm_interface_t comm_usbcdc_if;

void usbcdc_init(void);

#ifdef __cplusplus
}
#endif

#endif