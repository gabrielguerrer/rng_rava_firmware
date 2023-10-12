/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

#include <math.h>
#include <rava_tools.h>

uint16_t unpack_int(uint8_t b0, uint8_t b1)
{
  uint16_t res = b0 | (uint16_t)b1 << 8;
  return res;
}

int16_t unpack_int_signed(uint8_t b0, uint8_t b1)
{
  int16_t res = b0 | (uint16_t)b1 << 8;
  return res;
}

uint32_t unpack_long(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  uint32_t res = b0 | (uint16_t)b1 << 8 | (uint32_t)b2 << 16 | (uint32_t)b3 << 24;
  return res;
}

float unpack_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  float_union res;
  res.b[0] = b0;
  res.b[1] = b1;
  res.b[2] = b2;
  res.b[3] = b3;
  return res.f;
}

uint8_t nibble_to_hex(uint8_t n)
{
    if (n <= 9)
      return '0' + n; 
    else 
      return 'a' + (n - 10); 
}

uint8_t byte_rol(uint8_t b, uint8_t n) {
  return (b << n) | (b >> (8 - n));
}

uint8_t hamming_weight_8(uint8_t b) {
  // Count the ammount of 1 bits in the byte b
  uint8_t n;
  for (n = 0; b; n++)
    b &= b - 1; // clear the least significant bit set  
  return n;
} 

uint32_t bit_mask_1s(uint8_t n1s)
{
  uint32_t res = 0;
  for (uint8_t i=0; i < n1s; i++) {
    res |= 1 << i;
  }
  return res;
}

void array_init(uint16_t* a, uint8_t a_size, uint16_t value) {  
  for (uint8_t i = 0; i < a_size; i++) {
    a[i] = value;
  }  
}

void array_init(uint8_t* a, uint8_t a_size, uint8_t value) {  
  for (uint8_t i = 0; i < a_size; i++) {
    a[i] = value;
  }  
}

uint32_t array_sum(uint8_t* a, uint8_t a_size) {
  uint32_t res = 0;
  for (uint8_t i = 0; i < a_size ; i++) {
    res += a[i];
  }
  return res;
}

float normal_sf(float z) {
  // Normal curve survival function : sf(z) = 1 - cdf(z)
  float zsq = z * z;
  float p = 0.5 * sqrt( 1 - 1. / 30 * ( 7 * exp(-zsq / 2) + 16 * exp(-zsq * (2 - sqrt(2))) 
            + ( 7 + 1. / 4 * 3.141592653589793 * zsq ) * exp(-zsq) ) );
  if (z < 0)
    p = 0.5 + p;
  else
    p = 0.5 - p;
  return p;
}