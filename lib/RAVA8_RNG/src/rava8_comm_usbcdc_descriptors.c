/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <avr/pgmspace.h>
#include "rava8_comm_usbcdc_descriptors.h"
#include "rava8_device.h"

/* ===========================
 * RAVA8 COMM USBCDC DESCRIPTORS
 * =========================== */

const USB_Descriptor_String_t PROGMEM LanguageDescriptor = USB_STRING_DESCRIPTOR_ARRAY(LANGUAGE_ID_ENG);

const USB_Descriptor_String_t PROGMEM ManufacturerDescriptor = USB_STRING_DESCRIPTOR(RAVA_USB_MANUFACTURER);

const USB_Descriptor_String_t PROGMEM ProductDescriptor = USB_STRING_DESCRIPTOR(RAVA_USB_PRODUCT);

// The serial number descriptor contents are later updated by device_calc_serial_number(), which
// should be called before initializing the USB interface.
USB_Descriptor_String_t SerialNumberDescriptor = USB_STRING_DESCRIPTOR(L"0123456789");

/*
Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
device characteristics, including the supported USB version, control endpoint size and the
number of device configurations. The descriptor is read out by the USB host when the enumeration
process begins.
*/
const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
  .Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},
  .USBSpecification       = VERSION_BCD(1,1,0),
  .Class                  = CDC_CSCP_CDCClass,
  .SubClass               = CDC_CSCP_NoSpecificSubclass,
  .Protocol               = CDC_CSCP_NoSpecificProtocol,
  .Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,
  .VendorID               = RAVA_USB_VID,
  .ProductID              = RAVA_USB_PID,
  .ReleaseNumber          = VERSION_BCD(0,0,1),
  .ManufacturerStrIndex   = STRING_ID_Manufacturer,
  .ProductStrIndex        = STRING_ID_Product,
  .SerialNumStrIndex      = STRING_ID_SerialNumber,
  .NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/*
Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
of the device in one of its supported configurations, including information about any device
interfaces and endpoints. The descriptor is read out by the USB host during the enumeration process
when selecting a configuration so that the host may correctly communicate with the USB device.
*/
const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor = {
  .Config = {
    .Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},
    .TotalConfigurationSize = sizeof(USB_Descriptor_Configuration_t),
    .TotalInterfaces        = 2,
    .ConfigurationNumber    = 1,
    .ConfigurationStrIndex  = NO_DESCRIPTOR,
    .ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),
    .MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
    },
  .CDC_CCI_Interface = {
    .Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
    .InterfaceNumber        = INTERFACE_ID_CDC_CCI,
    .AlternateSetting       = 0,
    .TotalEndpoints         = 1,
    .Class                  = CDC_CSCP_CDCClass,
    .SubClass               = CDC_CSCP_ACMSubclass,
    .Protocol               = CDC_CSCP_ATCommandProtocol,
    .InterfaceStrIndex      = NO_DESCRIPTOR
    },
  .CDC_Functional_Header = {
    .Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t), .Type = DTYPE_CSInterface},
    .Subtype                = CDC_DSUBTYPE_CSInterface_Header,
    .CDCSpecification       = VERSION_BCD(1,1,0),
    },
  .CDC_Functional_ACM = {
    .Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t), .Type = DTYPE_CSInterface},
    .Subtype                = CDC_DSUBTYPE_CSInterface_ACM,
    .Capabilities           = 0x06,
    },
  .CDC_Functional_Union = {
    .Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t), .Type = DTYPE_CSInterface},
    .Subtype                = CDC_DSUBTYPE_CSInterface_Union,
    .MasterInterfaceNumber  = INTERFACE_ID_CDC_CCI,
    .SlaveInterfaceNumber   = INTERFACE_ID_CDC_DCI,
    },
  .CDC_NotificationEndpoint = {
    .Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
    .EndpointAddress        = CDC_NOTIFICATION_EPADDR,
    .Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
    .EndpointSize           = CDC_NOTIFICATION_EPSIZE,
    .PollingIntervalMS      = 0xFF
    },
  .CDC_DCI_Interface = {
    .Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
    .InterfaceNumber        = INTERFACE_ID_CDC_DCI,
    .AlternateSetting       = 0,
    .TotalEndpoints         = 2,
    .Class                  = CDC_CSCP_CDCDataClass,
    .SubClass               = CDC_CSCP_NoDataSubclass,
    .Protocol               = CDC_CSCP_NoDataProtocol,
    .InterfaceStrIndex      = NO_DESCRIPTOR
    },
  .CDC_DataOutEndpoint = {
    .Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
    .EndpointAddress        = CDC_RX_EPADDR,
    .Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
    .EndpointSize           = CDC_TXRX_EPSIZE,
    .PollingIntervalMS      = 0x05
    },
  .CDC_DataInEndpoint = {
    .Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
    .EndpointAddress        = CDC_TX_EPADDR,
    .Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
    .EndpointSize           = CDC_TXRX_EPSIZE,
    .PollingIntervalMS      = 0x05
    }
};

/*
When the device receives a Get Descriptor request on the control endpoint, this function is called
so that the descriptor details can be passed back and the appropriate descriptor sent back to the
USB host.
*/
uint16_t CALLBACK_USB_GetDescriptor(
  const uint16_t wValue, const uint16_t wIndex,
  const void **const DescriptorAddress, uint8_t *const DescriptorMemorySpace)
{
  const uint8_t  DescriptorType   = (wValue >> 8);
  const uint8_t  DescriptorNumber = (wValue & 0xFF);

  const void *Address = NULL;
  uint16_t    Size    = NO_DESCRIPTOR;
  *DescriptorMemorySpace = MEMSPACE_FLASH;

  switch (DescriptorType)
  {
    case DTYPE_Device:
      Address = &DeviceDescriptor;
      Size    = sizeof(USB_Descriptor_Device_t);
      break;
    case DTYPE_Configuration:
      Address = &ConfigurationDescriptor;
      Size    = sizeof(USB_Descriptor_Configuration_t);
      break;
    case DTYPE_String:
      switch (DescriptorNumber)
      {
        case STRING_ID_Language:
          Address = &LanguageDescriptor;
          Size    = pgm_read_byte(&LanguageDescriptor.Header.Size);
          break;
        case STRING_ID_Manufacturer:
          Address = &ManufacturerDescriptor;
          Size    = pgm_read_byte(&ManufacturerDescriptor.Header.Size);
          break;
        case STRING_ID_Product:
          Address = &ProductDescriptor;
          Size    = pgm_read_byte(&ProductDescriptor.Header.Size);
          break;
        case STRING_ID_SerialNumber:
          Address = &SerialNumberDescriptor;
          Size    = SerialNumberDescriptor.Header.Size;
          *DescriptorMemorySpace = MEMSPACE_RAM;
          break;
      }
      break;
  }

  *DescriptorAddress = Address;
  return Size;
}