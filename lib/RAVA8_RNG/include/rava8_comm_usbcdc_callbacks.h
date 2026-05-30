/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Defines the USB and CDC event callback handlers required by the LUFA library.
*/

#ifndef RAVA8_COMM_USBCDC_CALLBACKS_H
#define RAVA8_COMM_USBCDC_CALLBACKS_H

#include <LUFA/Drivers/USB/USB.h>

/* ===========================
 * RAVA8 COMM USB CDC CALLBACKS
 * =========================== */

#ifdef __cplusplus
extern "C" {
#endif

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);
void EVENT_USB_Device_Suspend(void);
void EVENT_USB_Device_WakeUp(void);
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo);

#ifdef __cplusplus
}
#endif

#endif