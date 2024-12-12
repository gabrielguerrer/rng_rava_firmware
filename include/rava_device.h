/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The DEVICE class enables the retrieval and transmission of information including
the device's serial number and the available RAM memory.
*/

#ifndef RAVA_DEVICE_H
#define RAVA_DEVICE_H

#include <stdint.h>

class DEVICE {
  public:
    void get_serial_number(uint8_t* const sn);
    void send_serial_number();

    int16_t get_free_ram();
    void send_free_ram();
};

#endif