/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <rava_tools.h>
#include "rava8_device.h"
#include "rava8_adc.h"
#include "rava8_comm_usbcdc_descriptors.h"
#include "rava8_config.h"

static uint8_t device_get_firmware_modules(void);
static uint16_t device_get_free_ram(void);
static uint16_t device_get_temperature(void);
static uint16_t device_get_vcc(void);

/* ===========================
 * RAVA8 DEVICE
 * =========================== */

/*
Computes the device serial number and stores it in the descriptor used during the USB enumeration
process. This function must be called before initializing the USB interface.

The ATmega32U4 appears to provide unique boot signature bytes from addresses 14 to 23, with the
exception of byte 20, which seems to be constant (0x15).
*/
void device_calc_serial_number(void)
{
  uint8_t b, k=0;
  for (uint8_t i = 14; i <= 22; i += 2) {
    b = xor_dichtl(boot_signature_byte_get(i), boot_signature_byte_get(i+1));
    SerialNumberDescriptor.UnicodeString[k++] = nibble_to_hex(b >> 4);
    SerialNumberDescriptor.UnicodeString[k++] = nibble_to_hex(b & 0x0F);
  }
}

/*
Sleeps for a specified number of microseconds on the 16 MHz MCU using a busy-wait delay.

Maximum delay is 16383 us, as larger values overflows the uint16_t.
*/
void device_delay_us(uint16_t us)
{
  if (us <= 1) return;

  // Same as multiplying by 4 (Four iterations = 1us)
  us <<= 2;

  // Discount ~20 cycles overhead
  us -= 5;

  // Busy Wait; Four cycles per iteration
  __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t"
    "brne 1b" : "=w" (us) : "0" (us)
  );
}

/*
Returns a bitmask representing the optional firmware capabilities enabled through preprocessor
directives.
*/
static uint8_t device_get_firmware_modules(void)
{
  uint8_t firmw_modules = 0;

  #if defined(HEALTH_STARTUP_ENABLED)
  sbi(firmw_modules, 0);
  #endif
  #if defined(HEALTH_CONTINUOUS_ENABLED)
  sbi(firmw_modules, 1);
  #endif
  #if defined(COMM_USART_ENABLED)
  sbi(firmw_modules, 2);
  #endif
  #if defined(PERIPHERALS_ENABLED)
  sbi(firmw_modules, 3);
  #endif
  #if defined(RNG_TIMING_DEBUG_ENABLED)
  sbi(firmw_modules, 4);
  #endif

  return firmw_modules;
}

/*
Estimates the amount of free RAM available.
*/
static uint16_t device_get_free_ram(void)
{
  extern int16_t __heap_start, *__brkval;
  int16_t v, res;
  res = (int16_t)&v - (__brkval == 0 ? (int16_t)&__heap_start : (int16_t) __brkval);
  return (uint16_t)res;
}

/*
Measures the device internal temperature sensor.

Returns the raw ADC measurement without temperature conversion, which requires proper
device-specific calibration.
*/
static uint16_t device_get_temperature(void)
{
  // Select internal 2.56V reference voltage  and internal temperature sensor input channel.
  ADMUX =
      (1 << REFS1) |  // REFS1:0 = 11: Internal 2.56V reference
      (1 << REFS0) |
      0x7;            // MUX[4:0] = 0x7: Internal temperature sensor channel

  // Set MUX5 to complete channel selection.
  ADCSRB |= (1 << MUX5);

  // Enable ADC and configure prescaler = 128.
  ADCSRA =
      (1 << ADEN) |
      (1 << ADPS2) |  // With F_CPU = 16 MHz: ADC clock = 125 kHz
      (1 << ADPS1) |
      (1 << ADPS0);

  // Wait for reference voltage and sensor mux to stabilize.
  _delay_ms(2);

  // Perform dummy conversion.
  ADCSRA |= (1 << ADSC);

  // Wait until conversion completes.
  while (ADCSRA & (1 << ADSC));

  // Start real conversion.
  ADCSRA |= (1 << ADSC);

  // Wait until conversion completes.
  while (ADCSRA & (1 << ADSC));

  // Read raw 10-bit ADC result.
  uint16_t adc_measurement =  ADC;

  // Reset ADC
  adc_reset();

  // Return ADC
  return adc_measurement;
}

/*
Measures the device supply voltage Vcc in mV.
*/
static uint16_t device_get_vcc(void)
{
  // Select ADC reference as AVcc (the MCU supply voltage),
  // and select the internal 1.1V bandgap as ADC input channel
  ADMUX =
      (1 << REFS0) |  //   ADC reference voltage = AVcc
      0x1E;           //   Select internal 1.1V bandgap channel

  // Clear MUX5 to complete internal channel selection
  ADCSRB &= ~(1 << MUX5);

  // Configure and enable the ADC
  ADCSRA =
      (1 << ADEN) |   // ADEN  = Enable ADC peripheral
      (1 << ADPS2) |  // ADPS2:0 = 111, ADC prescaler = 128
      (1 << ADPS1) |  // With F_CPU = 16 MHz: ADC clock = 16 MHz / 128 = 125 kHz
      (1 << ADPS0);

  // Wait for the voltage reference and input mux to stabilize after changing ADC configuration
  _delay_ms(2);

  // Start a dummy ADC conversion. The first conversion after changing reference or mux
  // is often inaccurate and should be discarded
  ADCSRA |= (1 << ADSC);

  // Wait until conversion completes.
  while (ADCSRA & (1 << ADSC));

  // Start the real ADC conversion.
  ADCSRA |= (1 << ADSC);

  // Wait until conversion completes.
  while (ADCSRA & (1 << ADSC));

  // Read the 10-bit ADC result.
  uint16_t adc = ADC;

  // Reseat ADC
  adc_reset();

  // Compute Vcc in millivolts: ADC = (Vbg / Vcc) * 1023
  // Rearranging: Vcc = (Vbg * 1023) / ADC
  // Assuming: Vbg = 1100 mV
  return (1100UL * 1023UL) / adc;
}

/* ===========================
 * COMM
 * =========================== */

/*
Processes the request to send the device identification, firmware version, and enabled module data.
*/
void comm_device_get_info(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {device_info_t devinfo;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  data_out.devinfo.mcu = RAVA_MCU;
  data_out.devinfo.model = RAVA_MODEL;
  data_out.devinfo.firmw_ver_major = RAVA_FIRMWARE_VERSION_MAJOR;
  data_out.devinfo.firmw_ver_minor = RAVA_FIRMWARE_VERSION_MINOR;
  data_out.devinfo.firmw_ver_patch = RAVA_FIRMWARE_VERSION_PATCH;
  data_out.devinfo.rng_gen_max_nbytes_per_core = RNG_GEN_MAX_NBYTES_PER_CORE;
  data_out.devinfo.firmw_modules = device_get_firmware_modules();

  for (uint8_t i = 0; i < SERIAL_NUMBER_N_BYTES; i++) {
    data_out.devinfo.serial_number[i] = (uint8_t)SerialNumberDescriptor.UnicodeString[i];
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/*
Processes the request to send the currently available RAM amount.
*/
void comm_device_get_free_ram(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {uint16_t free_ram;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  data_out.free_ram = device_get_free_ram();

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/*
Processes the request to send the measured device temperature.
*/
void comm_device_get_temperature(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {uint16_t temp;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  data_out.temp = device_get_temperature();

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/*
Processes the request to send the measured device supply voltage.
*/
void comm_device_get_vcc(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {uint16_t vcc;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  data_out.vcc = device_get_vcc();

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}