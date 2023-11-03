/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <Arduino.h>
#include <avr/io.h>

#include <rava_peripherals.h>
#include <rava_adc.h>
#include <rava_timers.h>
#include <rava_comm.h>

extern COMM* comm;

uint8_t D1_CFG[4] = {0x0E, 0x0D, 0x0C, 6}; // PE6
uint8_t D2_CFG[4] = {0x08, 0x07, 0x06, 7}; // PC7
uint8_t D3_CFG[4] = {0x08, 0x07, 0x06, 6}; // PC6
uint8_t D4_CFG[4] = {0x05, 0x04, 0x03, 7}; // PB7
uint8_t D5_CFG[4] = {0x05, 0x04, 0x03, 5}; // PB5

// OLD model
// uint8_t D2_CFG[4] = {0x08, 0x07, 0x06, 6}; // PC6
// uint8_t D3_CFG[4] = {0x05, 0x04, 0x03, 5}; // PB5
// uint8_t D5_CFG[4] = {0x0B, 0x0A, 0x09, 4}; // PD4

extern ADC_COMP* adc_comp;
extern TIMER3* timer3;
extern COMM* comm;

PERIPH::PERIPH(uint8_t* port_cfg):
  m_port_addr(port_cfg[0]),
  m_ddr_addr(port_cfg[1]),
  m_pin_addr(port_cfg[2]),
  m_port_i(port_cfg[3])
{}

void PERIPH::mode_output()
{
  _SFR_IO8(m_port_addr) &= ~_BV(m_port_i); // Low state
  _SFR_IO8(m_ddr_addr) |= _BV(m_port_i); // Output mode
}

void PERIPH::mode_input()
{
  _SFR_IO8(m_port_addr) &= ~_BV(m_port_i); // Low state
  _SFR_IO8(m_ddr_addr) &= ~_BV(m_port_i); // Input mode
}

void PERIPH::write_lo()
{
  _SFR_IO8(m_port_addr) &= ~_BV(m_port_i);
}

void PERIPH::write_hi()
{
  _SFR_IO8(m_port_addr) |= _BV(m_port_i);
}

bool PERIPH::validate_duration(uint16_t duration_us)
{
  if (duration_us == 0)
    return false;
  return true;
}

void PERIPH::write_pulse(uint16_t duration_us)
{
  if (!validate_duration(duration_us))
    return;

  write_hi();
  delayMicroseconds(duration_us);
  write_lo();
}

uint8_t PERIPH::read()
{
  return ((_SFR_IO8(m_pin_addr) & _BV(m_port_i)) != 0);
}

void PERIPH::send_digi_state()
{
  uint8_t mode = (_SFR_IO8(m_ddr_addr) & _BV(m_port_i)) != 0;
  uint8_t digi_state = read();

  comm->write_msg_header(COMM_PERIPH_READ, digi_state, mode);
}

D1::D1():
  PERIPH(D1_CFG)
{
}

void D1::write_lo_fast()
{
  PORTE = 0;
}

void D1::write_hi_fast()
{
  PORTE = 0b01000000;
}

uint8_t D1::read_fast()
{
  return (PORTE & 0b01000000) != 0;
}

void D1::reset_trigger_input()
{
  EICRB = 0;
  EIMSK = 0; // Disable interrupt
}

void D1::setup_trigger_input()
{
  mode_input();
  reset_trigger_input();

  // Rising edge between two samples of INT6 generates an interrupt
  EICRB = _BV(ISC61) | _BV(ISC60);
  // Enable interrupt. The interrupt function is impĺemented by ISR(INT6_vect)
  EIMSK |= _BV(INT6);
}

void D1::reset_comparator()
{
  adc_comp->reset();
}

void D1::setup_comparator(uint8_t neg_to_d5)
{
  adc_comp->setup_comparator(neg_to_d5);
}

bool D1::validate_delay(uint8_t delay_us)
{
  if (delay_us == 0)
    return false;
  return true;
}

void D1::delay_us_test(uint8_t delay_us)
{
  if (!validate_delay(delay_us))
    return;

  cli();
  write_hi_fast();
  delayMicroseconds(delay_us);
  write_lo_fast();
  sei();
}

D2::D2():
  PERIPH(D2_CFG)
{
}

void D2::reset_timer3_input_capture()
{
  timer3->reset();
}

void D2::setup_timer3_input_capture()
{
  mode_input();
  timer3->setup_input_capture();
}

void D2::send_timer3_input_capture_interval()
{
  // Measure and send counts
  uint16_t timer3_count = ICR3;
  float interval_s = timer3_count * 0.000016 + timer3_overflow_n;
  comm->write_msg_header(COMM_PERIPH_D2_TIMER3_INPUT_CAPTURE, interval_s);

  // Restart timer3 counter
  TCNT3 = 0;

  // Restart overflows
  timer3_overflow_n = 0;
}

D3::D3():
  PERIPH(D3_CFG)
{
}

bool D3::validate_interval(uint16_t interval_ms)
{
  if ((interval_ms == 0) or (interval_ms > TIMER3_MAXIMUM_DELAY_MS))
    return false;
  return true;
}

void D3::reset_timer3_trigger_output()
{
  timer3->reset();
}

void D3::setup_timer3_trigger_output(uint16_t interval_ms)
{
  if (!validate_interval(interval_ms))
    return;

  timer3->setup_trigger_output(interval_ms);
}

bool D3::validate_pwm_pars(uint8_t freq_prescaler, uint16_t top, uint16_t duty)
{
  if ((freq_prescaler == 0) || (freq_prescaler > 5))
    return false;
  if (top == 0)
    return false;
  if (duty == 0)
    return false;
  return true;
}

void D3::reset_timer3_pwm()
{
  timer3->reset();
}

void D3::setup_timer3_pwm(uint8_t freq_prescaler, uint16_t top, uint16_t duty)
{
  if (!validate_pwm_pars(freq_prescaler, top, duty))
    return;

  timer3->setup_pwm(freq_prescaler, top, duty);
}

D4::D4():
  PERIPH(D4_CFG)
{
}

void D4::reset_pin_change()
{
  PCICR = 0;
  PCMSK0 = 0;
}

void D4::setup_pin_change()
{
  mode_input();
  reset_pin_change();

  // Pin Change Interrupt Enable
  PCICR = _BV(PCIE0);
  // Pin Change Enable Mask ; Enable PCINT7
  PCMSK0 |= _BV(PCINT7);
}

D5::D5():
  PERIPH(D5_CFG)
{
}

void D5::reset_adc()
{
  adc_comp->reset();
}

bool D5::validate_adc_pars(uint8_t clk_prescaler, uint8_t oversampling_n_bits)
{
  if ((clk_prescaler == 0) || (clk_prescaler > 7))
    return false;
  if (oversampling_n_bits > 6)
    return false;
  return true;
}

void D5::send_adc_reading(uint8_t ref_5v, uint8_t clk_prescaler, uint8_t oversampling_n_bits)
{
  if (!validate_adc_pars(clk_prescaler, oversampling_n_bits))
    return;

  // Measure and send adc voltage
  float adc_reading = adc_comp->read_adc_ch12_volts(ref_5v, clk_prescaler, oversampling_n_bits);
  comm->write_msg_header(COMM_PERIPH_D5_ADC, adc_reading);
}