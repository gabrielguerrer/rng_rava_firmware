/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */


#include <avr/io.h>
#include "rava8_comm_usbcdc.h"
#include "rava8_comm_usbcdc_descriptors.h"

static bool usbcdc_read(uint8_t *const b);
static void usbcdc_write(uint8_t b);
static void usbcdc_write_buf(const uint8_t *const buffer, uint8_t len);
static void usbcdc_flush(void);

USB_ClassInfo_CDC_Device_t lufa_usbcdc_if =	{
  .Config = {
    .ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
    .DataINEndpoint           = {
      .Address          = CDC_TX_EPADDR,
      .Size             = CDC_TXRX_EPSIZE,
      .Banks            = 2,
    },
    .DataOUTEndpoint = {
      .Address          = CDC_RX_EPADDR,
      .Size             = CDC_TXRX_EPSIZE,
      .Banks            = 2,
    },
    .NotificationEndpoint = {
      .Address          = CDC_NOTIFICATION_EPADDR,
      .Size             = CDC_NOTIFICATION_EPSIZE,
      .Banks            = 1,
    },
  },
};

comm_interface_t comm_usbcdc_if = {
  .read = usbcdc_read,
  .write = usbcdc_write,
  .write_buf = usbcdc_write_buf,
  .flush = usbcdc_flush,
};

/* ===========================
 * RAVA8 COMM USBCDC
 * =========================== */

/*
Initializes the USB CDC communication interface.

Attaches the USB device, and waits for an optional USB setup handshake before proceeding.
*/
void usbcdc_init(void)
{
  USB_Init();
}

/*
Reads one byte from the USB CDC receive buffer.

If no byte is available, returns false. Otherwise, stores the received byte in `b` and returns true.
*/
static bool usbcdc_read(uint8_t *const b)
{
  int16_t recv = CDC_Device_ReceiveByte(&lufa_usbcdc_if);
  if (recv >= 0) {
    *b = (uint8_t)recv;
    return true;
  }
  return false;
}

/*
Writes a single byte through the USB CDC interface.
*/
static void usbcdc_write(uint8_t b)
{
  CDC_Device_SendByte(&lufa_usbcdc_if, b);
}

/*
Writes a buffer of bytes through the USB CDC interface.
*/
static void usbcdc_write_buf(const uint8_t *const buffer, uint8_t len)
{
  CDC_Device_SendData(&lufa_usbcdc_if, buffer, len);
}

/*
Flushes pending USB CDC transmission data.
*/
static void usbcdc_flush(void)
{
  CDC_Device_Flush(&lufa_usbcdc_if);
}