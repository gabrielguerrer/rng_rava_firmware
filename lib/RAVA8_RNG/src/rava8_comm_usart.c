/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <stdint.h>
#include <avr/power.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include "rava8_comm_usart.h"

static bool usart_read(uint8_t *const b);
static void usart_write(uint8_t b);
static void usart_write_buf(const uint8_t *const buffer, uint8_t len);
static void usart_flush(void);

comm_interface_t comm_usart_if = {
  .read = usart_read,
  .write = usart_write,
  .write_buf = usart_write_buf,
  .flush = usart_flush
};

/* ===========================
 * RAVA8 COMM USART
 * =========================== */

/*
Initializes the USART interface with the configured baud rate.
*/
void usart_init(void)
{
  power_usart1_enable();

  Serial_Init(USART_BAUD, false);
}

/*
Reads and returns one byte from the USART receive buffer.
*/
static bool usart_read(uint8_t *const b)
{
  int16_t recv = Serial_ReceiveByte();
  if (recv >= 0) {
    *b = (uint8_t)recv;
    return true;
  }
  return false;
}

/*
Writes a single byte through the USART interface.
*/
static void usart_write(uint8_t b)
{
  Serial_SendByte(b);
}

/*
Writes a buffer of bytes through the USART interface.
*/
static void usart_write_buf(const uint8_t *const buffer, uint8_t len)
{
  Serial_SendData(buffer, len);
}

/*
Empty, as no flush operation is required for USART.
*/
static void usart_flush(void)
{
}