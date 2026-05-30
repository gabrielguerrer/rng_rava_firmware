/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */


#include <avr/io.h>
#include "rava8_adc.h"
#include "rava8_device.h"

static float adc_read_volts(uint8_t ref_5v, uint8_t oversampling_n_bits);

/* ===========================
 * RAVA8 ADC
 * =========================== */

/*
Resets the ADC and analog comparator configuration registers to their default state.
*/
void adc_reset(void)
{
  ADMUX = 0;

  // Prescaler clk/2 ; Auto Trigger Disabled
  ADCSRA = 0;

  // High speed = No
  ADCSRB = 0;

  ACSR = 0;
}

/*
Configures the ADC for single-ended measurements on channel ADC12.

The reference voltage can be selected between AVCC (5V rail) and the internal 2.56V reference.
The ADC clock prescaler is configured through `clk_prescaler`.
*/
void adc_setup_ch12(uint8_t ref_5v, uint8_t clk_prescaler)
{
  // Reset
  adc_reset();

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
  device_delay_us(ADC_SETTLE_DELAY_US);
}

/*
Configures the analog comparator.

The negative comparator input can be selected between:
- ADC12, through the ADC multiplexer
- Internal 1.1V bandgap reference

Comparator interrupts are configured to trigger on the rising output edge.
*/
void adc_comp_setup(uint8_t neg_to_adc12)
{
  // Reset
  adc_reset();

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
  device_delay_us(ADC_SETTLE_DELAY_US);

  // Analog Comparator Interrupt Mode Select : Comparator Interrupt on Rising Output Edge
  ACSR |= _BV(ACIS1) | _BV(ACIS0); // The interrupt function is impĺemented by ISR(ANALOG_COMP_vect)
}

/*
Performs ADC measurements and returns the converted voltage value.

Supports oversampling to increase effective ADC resolution by `oversampling_n_bits`.
The ADC reference voltage is selected through `ref_5v`.
*/
static float adc_read_volts(uint8_t ref_5v, uint8_t oversampling_n_bits)
{
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
    adc_read_single = ADC;
    adc_read_over += adc_read_single;
  }

  // Oversampling result
  uint16_t adc_read = (adc_read_over >> oversampling_n_bits) & 0xffff;

  // Reference
  float ref_volts;
  if (ref_5v) {
    ref_volts = 4.84; // USB 5V after the fuse
  }
  else {
    ref_volts = 2.56;
  }

  // Convert to Volts and return
  float adc_read_v = (float)adc_read * ref_volts;
  adc_read_v /= (1UL << (10 + oversampling_n_bits));
  return adc_read_v;
}

/*
Performs a single-ended voltage measurement on ADC12.
*/
float adc_read_ch12_volts(uint8_t ref_5v, uint8_t clk_prescaler, uint8_t oversampling_n_bits)
{
  // Setup
  adc_setup_ch12(ref_5v, clk_prescaler);

  // Discard fist measurement
  ADCSRA |= _BV(ADSC); // ADC Start conversion
  while (ADCSRA & _BV(ADSC)); // Wait for the conversion

  // Oversampling, increase bit resolution by oversampling_n_bits
  float adc_measurement =  adc_read_volts(ref_5v, oversampling_n_bits);

  // Reset ADC
  adc_reset();

  return adc_measurement;
}