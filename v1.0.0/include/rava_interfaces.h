/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

/*
This module contains the code for interfacing the RAVA device with external 
components, which users should customize to meet their specific requirements.

As an example, the code for using a DS18B20 digital thermometer is provided. 
To activate this functionality, users should enable the inclusion of the 
robtillaart/DS18B20 library in the platformio.ini file and uncomment the line 
"#define FIRMWARE_INTERFACE_DS18B20_ENABLED" in the rava_config.h file.
*/

#ifndef RAVA_INTERFACES_H
#define RAVA_INTERFACES_H

#include <rava_config.h>

// FIRMWARE_INTERFACE_DS18B20
#if defined (FIRMWARE_INTERFACE_DS18B20_ENABLED)

#include <OneWire.h>
#include <DS18B20.h>

#define DS18B20_PIN 9
#define DS18B20_RESOLUTION 12

class INTERF_DS18B20
{
  public:
    INTERF_DS18B20(uint8_t pin, uint8_t resolution);    
    void set_resolution(uint8_t resolution);
    float read();
    void send_read();

    OneWire* one_wire;
    DS18B20* sensor;
};
#endif

#endif