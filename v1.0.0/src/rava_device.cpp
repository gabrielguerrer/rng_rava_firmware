/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <avr/boot.h>

#include <rava_device.h>
#include <rava_comm.h>
#include <rava_eeprom.h>
#include <rava_adc.h>
#include <rava_tools.h>

extern COMM* comm;
extern EEPROM* eeprom;
extern ADC_COMP* adc_comp;

void DEVICE::get_serial_number(uint8_t* const sn)
{
  uint8_t b = 0;

  // Set prefix byte
  // Convert 6 first bytes to a single hashed one (7th byte is always the same = 15)
  for(uint8_t i = 14; i < 20; i++) {
    b ^= boot_signature_byte_get(i);
    }
  sn[0] = nibble_to_hex(b >> 4);
  sn[1] = nibble_to_hex(b & 0xF);

  // Set last 3 bytes
  b = boot_signature_byte_get(21);
  sn[2] = nibble_to_hex(b >> 4);
  sn[3] = nibble_to_hex(b & 0xF);

  b = boot_signature_byte_get(22);
  sn[4] = nibble_to_hex(b >> 4);
  sn[5] = nibble_to_hex(b & 0xF);

  b = boot_signature_byte_get(23);
  sn[6] = nibble_to_hex(b >> 4);
  sn[7] = nibble_to_hex(b & 0xF);
}

void DEVICE::send_serial_number()
{
    // Get SN
    uint8_t sn[8];
    get_serial_number(sn);

    // Send header and SN
    comm->write_msg_header(COMM_DEVICE_SERIAL_NUMBER, (uint8_t)8);
    comm->write(sn, 8);
}

float DEVICE::get_temperature()
{
  // Get internal temperature reading in millivolts
  float temp_v = adc_comp->read_adc_chtemp_volts();

  // Read conversion parameters stored in the EEPROM
  uint16_t temp_slope;
  int16_t temp_intercept;
  eeprom->read_device(&temp_slope, &temp_intercept);

  // Convert V to Celcius degrees
  float temp = temp_v * temp_slope + temp_intercept;
  return temp;
}

void DEVICE::send_temperature()
{
  float temp = get_temperature();
  comm->write_msg_header(COMM_DEVICE_TEMPERATURE, temp);
}

int16_t DEVICE::get_free_ram()
{
  extern int16_t __heap_start, *__brkval;
  int16_t v;
  return (int16_t)&v - (__brkval == 0 ? (int16_t)&__heap_start : (int16_t) __brkval);
}

void DEVICE::send_free_ram()
{
  int16_t free_ram = get_free_ram();
  comm->write_msg_header(COMM_DEVICE_FREE_RAM, (uint16_t)free_ram);
}