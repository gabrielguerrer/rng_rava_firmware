/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Defines the USB device, configuration, and string descriptors presented to the host during the USB
enumeration process.

The serial number descriptor contents are later updated by device_calc_serial_number(), which
should be called before initializing the USB interface.
*/

#ifndef RAVA8_COMM_USBCDC_DESCRIPTORS_H
#define RAVA8_COMM_USBCDC_DESCRIPTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <LUFA/Drivers/USB/USB.h>

/* ===========================
 * RAVA8 COMM USBCDC DESCRIPTORS
 * =========================== */

// Endpoint address of the CDC device-to-host notification IN endpoint
#define CDC_NOTIFICATION_EPADDR        (ENDPOINT_DIR_IN  | 2)

// Endpoint address of the CDC device-to-host data IN endpoint
#define CDC_TX_EPADDR                  (ENDPOINT_DIR_IN  | 3)

// Endpoint address of the CDC host-to-device data OUT endpoint
#define CDC_RX_EPADDR                  (ENDPOINT_DIR_OUT | 4)

// Size in bytes of the CDC device-to-host notification IN endpoint
#define CDC_NOTIFICATION_EPSIZE        8

// Size in bytes of the CDC data IN and OUT endpoints
#define CDC_TXRX_EPSIZE                64

typedef struct USB_Descriptor_Configuration_t
{
  USB_Descriptor_Configuration_Header_t    Config;

  // CDC Control Interface
  USB_Descriptor_Interface_t               CDC_CCI_Interface;
  USB_CDC_Descriptor_FunctionalHeader_t    CDC_Functional_Header;
  USB_CDC_Descriptor_FunctionalACM_t       CDC_Functional_ACM;
  USB_CDC_Descriptor_FunctionalUnion_t     CDC_Functional_Union;
  USB_Descriptor_Endpoint_t                CDC_NotificationEndpoint;

  // CDC Data Interface
  USB_Descriptor_Interface_t               CDC_DCI_Interface;
  USB_Descriptor_Endpoint_t                CDC_DataOutEndpoint;
  USB_Descriptor_Endpoint_t                CDC_DataInEndpoint;
} USB_Descriptor_Configuration_t;

enum InterfaceDescriptors_t
{
  INTERFACE_ID_CDC_CCI = 0, // CDC CCI interface descriptor ID
  INTERFACE_ID_CDC_DCI = 1, // CDC DCI interface descriptor ID
};

enum StringDescriptors_t
{
  STRING_ID_Language     = 0,
  STRING_ID_Manufacturer = 1,
  STRING_ID_Product      = 2,
  STRING_ID_SerialNumber = 3,
};

extern USB_Descriptor_String_t SerialNumberDescriptor;

uint16_t CALLBACK_USB_GetDescriptor(
  const uint16_t wValue, const uint16_t wIndex,
  const void **const DescriptorAddress, uint8_t *const DescriptorMemorySpace)
  ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);

#ifdef __cplusplus
}
#endif

#endif