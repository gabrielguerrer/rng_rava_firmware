/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <avr/io.h>

#include <rava_pwm_boost.h>
#include <rava_config.h>
#include <rava_comm.h>
#include <rava_eeprom.h>
#include <rava_timers.h>

extern COMM* comm;
extern EEPROM* eeprom;
extern TIMER4* timer4;

PWM_BOOST::PWM_BOOST()
{
  setup(true);
}

bool PWM_BOOST::validate_setup_pars(uint8_t freq_id, uint8_t duty)
{
  if ((freq_id == 0) || (freq_id > 5)) {
    return false;
  }
  if (duty == 0) {
    return false;
  }
  return true;
}

void PWM_BOOST::setup(bool eeprom_values, uint8_t freq_id, uint8_t duty)
{
  // Load parameters from EEPROM?
  if (eeprom_values) {
    eeprom->read_pwm_boost(&freq_id, &duty);
  }

  // Validate pars
  if (!validate_setup_pars(freq_id, duty)) {
    return;
  }

  // Find freq_prescaler, duty_max, and top
  uint8_t freq_prescaler, top, duty_max;

  if (freq_id == PWM_BOOST_FREQ_30_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_8;
    duty_max = 25;
    top = 199;
  }
  else if (freq_id == PWM_BOOST_FREQ_40_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_8;
    duty_max = 25;
    top = 149;
  }
  else if (freq_id == PWM_BOOST_FREQ_50_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_4;
    duty_max = 50;
    top = 239;
  }
  else if (freq_id == PWM_BOOST_FREQ_60_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_4;
    duty_max = 50;
    top = 199;
  }
  else if (freq_id == PWM_BOOST_FREQ_75_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_4;
    duty_max = 50;
    top = 159;
  }

  // Configure Timer4
  if (duty <= duty_max) {
    configured = true;
    _freq_id = freq_id;
    _duty = duty;
    _freq_prescaler = freq_prescaler;
    _top = top;

    timer4->setup_pwm_pb6(freq_prescaler, top, duty);
  }
}

void PWM_BOOST::setup_resume()
{
  if (configured) {
    timer4->setup_pwm_pb6(_freq_prescaler, _top, _duty);
  }
}

void PWM_BOOST::stop()
{
  timer4->reset();
}

void PWM_BOOST::send_setup()
{
  comm->write_msg_header(COMM_PWM_BOOST_SETUP, _freq_id, _duty);
}