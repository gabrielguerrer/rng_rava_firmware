/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The ADC methods enable analog-to-digital conversion and comparator functionality exposed through
the D5 peripheral.
*/

#ifndef RAVA8_ADC_H
#define RAVA8_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===========================
 * RAVA8 ADC
 * =========================== */

#define ADC_SETTLE_DELAY_US 1000

enum ADC_PRESCALERS
{
  ADC_CLK_DIV_2=1,
  ADC_CLK_DIV_4,
  ADC_CLK_DIV_8,
  ADC_CLK_DIV_16,
  ADC_CLK_DIV_32,
  ADC_CLK_DIV_64,
  ADC_CLK_DIV_128,
  ADC_CLK_ENUM_LAST
};

void adc_reset(void);
void adc_setup_ch12(uint8_t ref_5v, uint8_t clk_prescaler);
void adc_comp_setup(uint8_t neg_to_adc12);
float adc_read_ch12_volts(uint8_t ref_5v, uint8_t clk_prescaler, uint8_t oversampling_n_bits);

#ifdef __cplusplus
}
#endif

#endif