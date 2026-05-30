/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <string.h>
#include "rava_tools.h"

/* ===========================
 * RAVA TOOLS
 * =========================== */

/*
Deserializes a 2-byte little-endian buffer into a uint16_t value.
*/
uint16_t unpack_int(const uint8_t *bytes)
{
  uint16_t res;
  memcpy(&res, bytes, 2);
  return res;
}

/*
Deserializes a 4-byte little-endian buffer into a uint32_t value.
*/
uint32_t unpack_long(const uint8_t *bytes)
{
  uint32_t res;
  memcpy(&res, bytes, 4);
  return res;
}

/*
Deserializes a 4-byte little-endian buffer into a float value.
*/
float unpack_float(const uint8_t *bytes)
{
  float res;
  memcpy(&res, bytes, 4);
  return res;
}

/*
Returns the Hamming weight of an 8-bit value, corresponding to the number of bits set to 1 in the 
input byte.
*/
uint8_t hamming_weight_8(uint8_t byte) {
  uint8_t n;
  uint8_t tmp = byte;
  for (n = 0; tmp; n++) {
    tmp &= tmp - 1; // clear the least significant bit set
  }
  return n;
}

/*
Generates the minimum 8-bit mask capable of representing values in the interval [0, int_delta].

The returned mask has all bits set up to the most significant bit of int_delta and is typically 
used for rejection-sampling of uniformly distributed integers.
*/
uint8_t gen_int8s_mask(uint8_t int_delta)
{
  if (int_delta == 0) {
    return 0;
  }
  uint8_t bits = 0;
  uint8_t tmp = int_delta;
  while (tmp) {
    tmp >>= 1;
    bits++;
  }
  return (1 << bits) - 1;
}

/*
Generates the minimum 16-bit mask capable of representing values in the interval [0, int_delta].

The returned mask has all bits set up to the most significant bit of int_delta and is typically 
used for rejection-sampling of uniformly distributed integers.
*/
uint16_t gen_int16s_mask(uint16_t int_delta)
{
  if (int_delta == 0) {
    return 0;
  }
  else if (int_delta == 0xFFFF) {
    return 0xFFFF;
  }
  uint16_t tmp = int_delta;
  uint8_t bits = 0;
  while (tmp) {
    tmp >>= 1;
    bits++;
  }
  return ((uint16_t)1 << bits) - 1;
}