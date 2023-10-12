/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

#include <USBAPI.h>

#include <rava_comm.h>
#include <rava_tools.h>

#define SERIAL_BAUD 115200
#define SERIAL_READ_TIMEOUT_MS 100

#define USB_WAIT_SETUP_TIMEOUT_TICKS 250000

void COMM::write(uint16_t value)
{
  int_union value_u;
  value_u.i = value;
  write(value_u.b, 2);
}

void COMM::write(uint32_t value)
{
  long_union value_u;
  value_u.i = value;
  write(value_u.b, 4);
}

void COMM::write(float value)
{
  float_union value_u;
  value_u.f = value;
  write(value_u.b, 4);
}

uint8_t COMM::read_msg_header(uint8_t* msg_bytes)
{
  // Has the device been contacted?
  if (available()) {

    // Search for msg start
    if (read() == COMM_MSG_START) {

      // Read remaining bytes. Timeout after SERIAL_READ_TIMEOUT_MS
      uint8_t n_read = read(msg_bytes, COMM_MSG_LEN-1);

      // Incomplete messages are filtered by this condition
      if (n_read == COMM_MSG_LEN-1) {

        // Return message command
        uint8_t command_id = msg_bytes[0];
        return command_id;
      }
    }
  }

  // Otherwise returns 0
  return 0;
}

void COMM::write_msg_header(uint8_t command_id, uint8_t par1, uint8_t par2, uint8_t par3, uint8_t par4, uint8_t par5, 
                            uint8_t par6)
{
  uint8_t msg_header[COMM_MSG_LEN];

  msg_header[0] = COMM_MSG_START;
  msg_header[1] = command_id;
  msg_header[2] = par1;
  msg_header[3] = par2;
  msg_header[4] = par3;
  msg_header[5] = par4;
  msg_header[6] = par5;  
  msg_header[7] = par6;  

  write(msg_header, COMM_MSG_LEN);
}

void COMM::write_msg_header(uint8_t command_id, uint16_t par1, uint8_t par2, uint8_t par3, uint8_t par4, uint8_t par5)
{
  write_msg_header(command_id, par1 & 255, par1 >> 8 & 255, par2, par3, par4, par5);
}

void COMM::write_msg_header(uint8_t command_id, uint16_t par1, uint16_t par2, uint8_t par3, uint8_t par4)
{
  write_msg_header(command_id, par1 & 255, par1 >> 8 & 255, par2 & 255, par2 >> 8 & 255, par3, par4);
}

void COMM::write_msg_header(uint8_t command_id, uint32_t par1, uint8_t par2, uint8_t par3)
{
  write_msg_header(command_id, par1 & 255, par1 >> 8 & 255, par1 >> 16 & 255, par1 >> 24 & 255, par2, par3);
}

void COMM::write_msg_header(uint8_t command_id, float par1, uint8_t par2, uint8_t par3)
{
  float_union par_union;
  par_union.f = par1;
  write_msg_header(command_id, par_union.b[0], par_union.b[1], par_union.b[2], par_union.b[3], par2, par3);
}

bool COMM::get_contacted()
{
  return is_contacted;
}

void COMM::set_contacted(bool contact)
{
  is_contacted = contact;
}

COMM_USB::COMM_USB()
{
  // The baud value is irrelevant for the 32u4
  Serial.begin(SERIAL_BAUD);  

  // Timeout allows receiving incomplete messages without blocking the code flow
  Serial.setTimeout(SERIAL_READ_TIMEOUT_MS); 

  // Tries waiting for USB handshake before entering health startup
  wait_usb_setup(USB_WAIT_SETUP_TIMEOUT_TICKS);
}

uint16_t COMM_USB::available()
{
  return Serial.available();
}
  
uint8_t COMM_USB::read()
{
  return Serial.read();
}

uint8_t COMM_USB::read(uint8_t* buffer, uint8_t length)
{
  return Serial.readBytes(buffer, length);
}

void COMM_USB::write(uint8_t value)
{
  Serial.write(value);
}

void COMM_USB::write(const uint8_t* buffer, uint8_t length)
{
  Serial.write(buffer, length);
}

/**
 * Without this wait function, the device proceeds directly to the startup 
 * health tests. These tests place a considerable load on the CPU, delaying the 
 * response to USB setup interactions with the operating system.
 * 
 * A timeout is necessary as not all boots events involve new USB 
 * communications, as when the device is connected to a USB interface delivering 
 * power only.
 */
void COMM_USB::wait_usb_setup(uint32_t ticks_timeout)
{
  uint32_t i=0;

  // If not waking from a WDT reboot
  if (!(MCUSR & _BV(WDRF))) {

    // Received USB setup?
    while (!(UEINTX & _BV(RXSTPI))) {
      i++;

      // Break loop if timeout is reach
      if (i > ticks_timeout)
        break;
    }
  }
}

COMM_SERIAL::COMM_SERIAL()
{
  // The baud value is relevant for the Hardware serial
  Serial1.begin(SERIAL_BAUD);  
  
  // Timeout allows receiving incomplete messages without blocking the code flow
  Serial1.setTimeout(SERIAL_READ_TIMEOUT_MS); 
}

uint16_t COMM_SERIAL::available()
{
  return Serial1.available();
}
  
uint8_t COMM_SERIAL::read()
{
  return Serial1.read();
}

uint8_t COMM_SERIAL::read(uint8_t* buffer, uint8_t length)
{
  return Serial1.readBytes(buffer, length);
}

void COMM_SERIAL::write(uint8_t value)
{
  Serial1.write(value);
}

void COMM_SERIAL::write(const uint8_t* buffer, uint8_t length)
{
  Serial1.write(buffer, length);
}