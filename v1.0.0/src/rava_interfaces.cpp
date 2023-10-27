/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <rava_interfaces.h>
#include <rava_comm.h>

extern COMM* comm;

// FIRMWARE_INTERFACE_DS18B20
#if defined (FIRMWARE_INTERFACE_DS18B20_ENABLED)

INTERF_DS18B20::INTERF_DS18B20(uint8_t pin, uint8_t resolution)
{
  one_wire = new OneWire(pin);
  sensor = new DS18B20(one_wire, resolution);
}

void INTERF_DS18B20::set_resolution(uint8_t resolution)
{
  sensor->setResolution(resolution);
}

float INTERF_DS18B20::read()
{
  sensor->requestTemperatures();
  while (!sensor->isConversionComplete());
  return sensor->getTempC();
}

void INTERF_DS18B20::send_read()
{
  float temp = read();
  comm->write_msg_header(COMM_INTERFACE_DS18B20, temp);
}
#endif