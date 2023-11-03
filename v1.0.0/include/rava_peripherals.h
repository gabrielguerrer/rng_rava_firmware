/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The peripherals module encompasses the functionalities of the five exposed ports
Di, which can be utilized for both input and output of digital signals.
Furthermore, these ports encompass a variety of alternate functions and
specialized applications, which are described below.

// D1 (PE6)

PE6 alternate funcions are:
 * INT6 - External Interrupt
 * AIN0 - Analog Comparator Positive input

Port D1 is specially used in the RAVA device for:
 * Implementing fast read and write operations. All bits of PORTE can be changed
 simultaneously, as no individual bit set or clear operations are required. This
 is facilitated by the fact that PORTE contains only one additional channel,
 PE2, which is linked to GND -- functions read_fast(), write_lo_fast(),  and
 write_hi_fast().
 * Receiving external triggers -- setup_trigger_input() function.
 * Implemementing the negative input of analog comparator applications
 -- setup_comparator() function.

// D2 (PC7)

PC7 alternate funcions are:
 * ICP3 - Input Capture Timer 3
 * CLK0 - System Clock
 * OC4A - Timer 4 Output Compare A

Port D2 is specially used in the RAVA device for:
  * Measure the time interval between digital pulses -- functions
  setup_timer3_input_capture() and send_timer3_input_capture_count().
  * Output the system clock (must enable CKOUT fuse bit)

// D3 (PC6)

PC6 alternate funcions are:
 * OC3A - Timer 3 Output Compare A
 * OC4A - Timer 4 Output Compare A

Port D3 is specially used in the RAVA device for:
  * Exposing Output compare ports of Timer3 -- setup_timer3_trigger_output() and
  setup_timer3_pwm() functions. MCU Timers 0, 1 and 4 are exclusive to the
  entropy generation, hence only Timer3 applications are exposed on D3.

// D4 (PB7)

PB7 alternate funcions are:
 * OC0A - Timer 0 Output Compare A
 * OC1C - Timer 1 Output Compare C
 * PCINT7 - Pin Change Interrupt

Port D4 is specially used in the RAVA device for:
  * Pin change applications -- function setup_pin_change().

// D5 (PB5)

PB5 alternate funcions are:
 * OC1A - Timer 1 Output Compare A
 * PCINT5 - Pin Change Interrupt source
 * OC4B - Timer 4 Output Compare B
 * ADC12 - Analog to Digital Converter

Port D5 is specially used in the RAVA device for:
  * Implementing ADC applications -- function read_adc().
  * Implemementing the positive input of analog comparator applications enabled
  by D1::setup_comparator(true).
*/

#ifndef RAVA_PERIPHERALS_H
#define RAVA_PERIPHERALS_H

#include <stdint.h>

enum PERIPHERALS_MODES {
  PERIPH_INPUT,
  PERIPH_OUTPUT
};

class PERIPH
{
  public:
    PERIPH(uint8_t* port_cfg);

    void mode_output();
    void mode_input();
    void write_lo();
    void write_hi();

    bool validate_duration(uint16_t duration_us);
    void write_pulse(uint16_t duration_us);

    uint8_t read();
    void send_digi_state();

  private:
    uint8_t m_port_addr, m_ddr_addr, m_pin_addr, m_port_i;
};

class D1 : public PERIPH
{
  public:
    D1();

    void write_lo_fast();
    void write_hi_fast();
    uint8_t read_fast();

    void reset_trigger_input();
    void setup_trigger_input();

    void reset_comparator();
    void setup_comparator(uint8_t neg_to_d5);

    bool validate_delay(uint8_t delay_us);
    void delay_us_test(uint8_t delay_us);
};

class D2 : public PERIPH
{
  public:
    D2();

    void reset_timer3_input_capture();
    void setup_timer3_input_capture();
    void send_timer3_input_capture_interval();

    volatile uint16_t timer3_overflow_n = 0;
};

class D3 : public PERIPH
{
  public:
    D3();

    bool validate_interval(uint16_t interval_ms);
    void reset_timer3_trigger_output();
    void setup_timer3_trigger_output(uint16_t interval_ms);

    bool validate_pwm_pars(uint8_t freq_prescaler, uint16_t top, uint16_t duty);
    void reset_timer3_pwm();
    void setup_timer3_pwm(uint8_t freq_prescaler, uint16_t top, uint16_t duty);
};

class D4 : public PERIPH
{
  public:
    D4();

    void reset_pin_change();
    void setup_pin_change();
};

class D5 : public PERIPH
{
  public:
    D5();

    void reset_adc();
    bool validate_adc_pars(uint8_t clk_prescaler, uint8_t oversampling_n_bits);
    void send_adc_reading(uint8_t ref_5v, uint8_t clk_prescaler, uint8_t oversampling_n_bits);
};

#endif