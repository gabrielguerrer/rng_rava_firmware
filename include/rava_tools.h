/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Tools and utilities library.
*/

#ifndef RAVA_TOOLS_H
#define RAVA_TOOLS_H

#include <stdint.h>

typedef union {
  uint16_t i;
  uint8_t b[2];
} int_union;

typedef union {
  uint32_t i;
  uint8_t b[4];
} long_union;

typedef union {
  float f;
  uint8_t b[4];
  uint32_t i;
} float_union;

uint16_t unpack_int(uint8_t b0, uint8_t b1);
uint32_t unpack_long(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
float unpack_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);

uint8_t nibble_to_hex(uint8_t n);
uint8_t byte_rol(uint8_t b, uint8_t n);
uint8_t xor_dichtl(uint8_t &rnd_a, uint8_t &rnd_b);
uint8_t hamming_weight_8(uint8_t b);
uint32_t bit_mask_1s(uint8_t n1s);

void array_init(uint16_t* a, uint16_t a_size, uint16_t value);
void array_init(uint8_t* a, uint16_t a_size, uint8_t value);
uint32_t array_sum(uint8_t* a, uint16_t a_size);

float normal_sf(float x);

#endif