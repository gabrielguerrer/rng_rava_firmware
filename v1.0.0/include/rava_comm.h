/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The comm module defines the communication protocol employed for controlling the
RAVA device. It implements a leader/follower design, wherein a leader device
initiates communication by issuing requests that prompt the RAVA device to
respond accordingly.

Since the primary task of the device is to transmit random bytes within the
range of 0 to 255, the use of message-ending characters is impractical. Instead,
the communication protocol relies on fixed-size messages, including information
about the quantity of additional bytes when required.

All exchanges start with an 8-byte message. The first byte holds the character
'$' (00100100), signifying the message start. The second byte encodes the
command's identification code, while the subsequent bytes house the command's
specific data. It's important to note that data transmission follows the
little-endian format.

The device continuously scans its communication interfaces using the
read_msg_header() function. When a byte matching '$' is encountered, the next 7
bytes are promptly read resulting in the command id and the data payload.
Subsequently, the task_serial_read() function within the main loop takes action
based on the received command identifier. Following the execution of the
required task, the device may transmit information back using the
write_msg_header() function.

Additionally, this file includes all available command identifiers within the
RAVA_COMMAND_IDS enum. It also defines the COMM classes responsible for
exchanginginformation through the established serial port, which includes both
the USB connection (class COMM_USB) and the USART interface (class COMM_SERIAL).
*/

#ifndef RAVA_COMM_H
#define RAVA_COMM_H

#include <stdint.h>

#define COMM_MSG_START '$'
#define COMM_MSG_LEN 8

enum RAVA_COMMAND_IDS {
  COMM_DEVICE_SERIAL_NUMBER=1,
  COMM_DEVICE_TEMPERATURE,
  COMM_DEVICE_FREE_RAM,
  COMM_DEVICE_REBOOT,
  COMM_DEVICE_DEBUG,

  COMM_EEPROM_RESET_TO_DEFAULT=10,
  COMM_EEPROM_DEVICE,
  COMM_EEPROM_FIRMWARE,
  COMM_EEPROM_PWM,
  COMM_EEPROM_RNG,
  COMM_EEPROM_LED,
  COMM_EEPROM_LAMP,

  COMM_PWM_SETUP = 30,

  COMM_RNG_SETUP=40,
  COMM_RNG_PULSE_COUNTS,
  COMM_RNG_BITS,
  COMM_RNG_BYTES,
  COMM_RNG_TIMING_DEBUG_D1,

  COMM_RNG_INT8S = 50,
  COMM_RNG_INT16S,
  COMM_RNG_FLOATS,

  COMM_RNG_STREAM_START = 60,
  COMM_RNG_STREAM_STOP,
  COMM_RNG_STREAM_BYTES,
  COMM_RNG_STREAM_STATUS,

  COMM_HEALTH_STARTUP_RUN = 70,
  COMM_HEALTH_STARTUP_RESULTS,
  COMM_HEALTH_CONTINUOUS_ERRORS,

  COMM_LED_COLOR = 80,
  COMM_LED_COLOR_FADE,
  COMM_LED_COLOR_OSCILLATE,
  COMM_LED_INTENSITY,
  COMM_LED_INTENSITY_FADE,
  COMM_LED_FADE_STOP,
  COMM_LED_STATUS,

  COMM_LAMP_MODE = 90,
  COMM_LAMP_STATISTICS,
  COMM_LAMP_DEBUG,

  COMM_PERIPH_MODE=100,
  COMM_PERIPH_READ,
  COMM_PERIPH_WRITE,
  COMM_PERIPH_PULSE,

  COMM_PERIPH_D1_TRIGGER_INPUT=110,
  COMM_PERIPH_D1_COMPARATOR,
  COMM_PERIPH_D1_DELAY_US_TEST,
  COMM_PERIPH_D2_TIMER3_INPUT_CAPTURE,
  COMM_PERIPH_D3_TIMER3_TRIGGER_OUTPUT,
  COMM_PERIPH_D3_TIMER3_PWM,
  COMM_PERIPH_D4_PIN_CHANGE,
  COMM_PERIPH_D5_ADC,

  COMM_INTERFACE_DS18B20=130
};

class COMM
{
  public:
    virtual uint16_t available();
    virtual uint8_t read();
    virtual uint8_t read(uint8_t* buffer, uint8_t length);
    virtual void write(uint8_t value);
    virtual void write(const uint8_t* buffer, uint8_t length);
    void write(uint16_t value);
    void write(uint32_t value);
    void write(float value);

    uint8_t read_msg_header(uint8_t* msg_bytes);
    void write_msg_header(uint8_t command_id, uint8_t par1=0, uint8_t par2=0, uint8_t par3=0, uint8_t par4=0,
                          uint8_t par5=0, uint8_t par6=0);
    void write_msg_header(uint8_t command_id, uint16_t par1, uint8_t par2=0, uint8_t par3=0, uint8_t par4=0,
                          uint8_t par5=0);
    void write_msg_header(uint8_t command_id, uint16_t par1, uint16_t par2, uint8_t par3=0, uint8_t par4=0);
    void write_msg_header(uint8_t command_id, uint32_t par1, uint8_t par2=0, uint8_t par3=0);
    void write_msg_header(uint8_t command_id, float par1, uint8_t par2=0, uint8_t par3=0);

    bool get_contacted();
    void set_contacted(bool contact=false);

  private:
    bool is_contacted = false;
};

class COMM_USB : public COMM
{
  public:
    COMM_USB();
    uint16_t available();
    uint8_t read();
    uint8_t read(uint8_t* buffer, uint8_t length);
    void write(uint8_t value);
    void write(const uint8_t* buffer, uint8_t length);

    void wait_usb_setup(uint32_t ticks_timeout);
};

class COMM_SERIAL : public COMM
{
  public:
    COMM_SERIAL();
    uint16_t available();
    uint8_t read();
    uint8_t read(uint8_t* buffer, uint8_t length);
    void write(uint8_t value);
    void write(const uint8_t* buffer, uint8_t length);
};

#endif