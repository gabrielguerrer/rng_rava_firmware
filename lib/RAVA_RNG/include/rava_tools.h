/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Tools and utilities library.
*/

#ifndef RAVA_TOOLS_H
#define RAVA_TOOLS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===========================
 * RAVA TOOLS
 * =========================== */

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))

typedef union
{
  uint16_t i;
  uint8_t b[2];
} uint16_u;

typedef union
{
  uint32_t i;
  uint8_t b[4];
} uint32_u;

typedef union
{
  float f;
  uint8_t b[4];
  uint32_t i;
} float_u;

uint16_t unpack_int(const uint8_t *bytes);
uint32_t unpack_long(const uint8_t *bytes);
float unpack_float(const uint8_t *bytes);

static inline uint8_t nibble_to_hex(uint8_t n) {return (n <= 9) ? '0' + n : 'a' + (n - 10);};
static inline uint8_t byte_rol(uint8_t b, uint8_t n) {return (b << n) | (b >> (8 - n));};
static inline uint8_t xor_dichtl(uint8_t rnd_a, uint8_t rnd_b) {return (rnd_a ^ byte_rol(rnd_a, 1)) ^ rnd_b;};
uint8_t hamming_weight_8(uint8_t byte);

uint8_t gen_int8s_mask(uint8_t int_delta);
uint16_t gen_int16s_mask(uint16_t int_delta);

#ifdef __cplusplus
}
#endif

#endif