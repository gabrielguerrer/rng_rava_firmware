/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <Arduino.h>
#include <avr/io.h>

#include <rava_adc.h>
#include <rava_tools.h>

#define ADC_SETTLE_DELAY_US 1000

void ADC_COMP::reset()
{
  ADMUX = 0;

  // Prescaler clk/2 ; Auto Trigger Disabled
  ADCSRA = 0;

  // High speed = No
  ADCSRB = 0;

  ACSR = 0;
}

void ADC_COMP::setup_adc_ch12(uint8_t ref_5v, uint8_t clk_prescaler)
{
  // Reset
  reset();

  // Reference
  if (ref_5v) {
    // AVCC (5V)
    ADMUX |= _BV(REFS0);
  }
  else {
    // Internal 2.56V Voltage Reference
    ADMUX |= _BV(REFS1) | _BV(REFS0);
  }

  // CLK PRESCALER
  ADCSRA = clk_prescaler;

  // Analog Channel Selection: ADC12
  ADMUX |= _BV(MUX2);
  ADCSRB |= _BV(MUX5);

  // ADC Enable
  ADCSRA |= _BV(ADEN);

  // Wait for internal reference to settle
  delayMicroseconds(ADC_SETTLE_DELAY_US);
}

void ADC_COMP::setup_adc_chtemp()
{
  // Reset
  reset();

  // Internal 2.56V Voltage Reference
  ADMUX |= _BV(REFS1) | _BV(REFS0);

  // Analog Channel Selection: Temperature Sensor
  ADMUX |= _BV(MUX2) | _BV(MUX1) | _BV(MUX0);
  ADCSRB |= _BV(MUX5);

  // CLK PRESCALER
  ADCSRA = ADC_CLK_DIV_64;

  // ADC Enable
  ADCSRA |= _BV(ADEN);

  // Wait for internal reference to settle
  delayMicroseconds(ADC_SETTLE_DELAY_US);
}

void ADC_COMP::setup_comparator(uint8_t neg_to_adc12)
{
  // Reset
  reset();

  // Define AIN-
  if (neg_to_adc12) {
    // Analog Comparator Multiplexer Enable
    ADCSRB |= _BV(ACME);
    // AIN- to ADC12
    ADMUX |= _BV(MUX2);
    ADCSRB |= _BV(MUX5);
  }
  // else: AIN- to 1.1V

  // Analog Comparator Interrupt Enable
  ACSR |= _BV(ACIE);

  // Wait for internal reference to settle
  delayMicroseconds(ADC_SETTLE_DELAY_US);

  // Analog Comparator Interrupt Mode Select : Comparator Interrupt on Rising Output Edge
  ACSR |= _BV(ACIS1) | _BV(ACIS0); // The interrupt function is impĺemented by ISR(ANALOG_COMP_vect)
}

float ADC_COMP::read_adc_volts(uint8_t ref_5v, uint8_t oversampling_n_bits)
{
  uint8_t low, high;

  // Oversampling: increase bit resolution by oversampling_n_bits
  uint16_t n_iter = 1U << (2*oversampling_n_bits); // 2^(2*oversampling_n_bits)
  uint16_t adc_read_single = 0;
  uint32_t adc_read_over = 0;

  // Oversampling loop
  for (uint16_t i=0; i < n_iter; i++) {

    // ADC Start conversionf
    ADCSRA |= _BV(ADSC);

    // Wait for the conversion
    while (ADCSRA & _BV(ADSC));

    // Read result
    low = ADCL;
    high = ADCH;
    adc_read_single = unpack_int(low, high);
    adc_read_over += adc_read_single;
  }

  // Oversampling result
  uint16_t adc_read = (adc_read_over >> oversampling_n_bits) & 0xffff;

  // Reference
  float ref_volts;
  if (ref_5v)
    ref_volts = 4.84; // USB 5V after the fuse
  else
    ref_volts = 2.56;

  // Convert to Volts and return
  float adc_read_v = (float)adc_read * ref_volts;
  adc_read_v /= (1UL << (10 + oversampling_n_bits));
  return adc_read_v;
}

float ADC_COMP::read_adc_ch12_volts(uint8_t ref_5v, uint8_t clk_prescaler, uint8_t oversampling_n_bits)
{
  // Setup
  setup_adc_ch12(ref_5v, clk_prescaler);

  // Discard fist measurement
  ADCSRA |= _BV(ADSC); // ADC Start conversion
  while (ADCSRA & _BV(ADSC)); // Wait for the conversion

  // Oversampling, increase bit resolution by oversampling_n_bits
  return read_adc_volts(ref_5v, oversampling_n_bits);
}

float ADC_COMP::read_adc_chtemp_volts()
{
  // Setup
  setup_adc_chtemp();

  // Discard fist measurement as described in the 32u4 datasheed
  ADCSRA |= _BV(ADSC); // ADC Start conversion
  while (ADCSRA & _BV(ADSC)); // Wait for the conversion

  // Oversample 5 extra bits; 2^(2*5) iterations
  return read_adc_volts(false, 5);
}