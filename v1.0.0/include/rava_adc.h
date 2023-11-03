/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The ADC_COMP class empowers the functionality of analog-to-digital conversions
and comparator features that are exposed throgh the D5 peripheral. For more
details refer to rava_peripherals.
*/

#ifndef RAVA_ADC_H
#define RAVA_ADC_H

#include <stdint.h>

enum ADC_PRESCALERS{
  ADC_CLK_DIV_2=1,
  ADC_CLK_DIV_4,
  ADC_CLK_DIV_8,
  ADC_CLK_DIV_16,
  ADC_CLK_DIV_32,
  ADC_CLK_DIV_64,
  ADC_CLK_DIV_128
};

class ADC_COMP
{
  public:
    void reset();

    void setup_adc_ch12(uint8_t ref_5v, uint8_t clk_prescaler);    
    void setup_adc_chtemp();
    void setup_comparator(uint8_t neg_to_adc12);

    float read_adc_volts(uint8_t ref_5v, uint8_t oversampling_n_bits);

    float read_adc_ch12_volts(uint8_t ref_5v, uint8_t clk_prescaler, uint8_t oversampling_n_bits);
    float read_adc_chtemp_volts();
};

#endif