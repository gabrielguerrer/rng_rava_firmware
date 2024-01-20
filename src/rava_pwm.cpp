/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <rava_pwm.h>
#include <rava_config.h>
#include <rava_comm.h>
#include <rava_eeprom.h>
#include <rava_timers.h>

extern COMM* comm;
extern EEPROM* eeprom;
extern TIMER4* timer4;

PWM::PWM()
{
  setup(true);
}

bool PWM::validate_setup_pars(uint8_t freq_id, uint8_t duty)
{
  if ((freq_id == 0) || (freq_id > 5)) {
    return false;
  }
  if (duty == 0) {
    return false;
  }
  return true;
}

void PWM::setup(bool eeprom_values, uint8_t freq_id, uint8_t duty)
{
  // Load parameters from EEPROM?
  if (eeprom_values) {
    eeprom->read_pwm(&freq_id, &duty);
  }

  // Validate pars
  if (!validate_setup_pars(freq_id, duty))
    return;

  // Find freq_prescaler, duty_max, and top
  uint8_t freq_prescaler, top, duty_max;

  if (freq_id == PWM_FREQ_30_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_8;
    duty_max = 25;
    top = 199;
  }
  else if (freq_id == PWM_FREQ_40_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_8;
    duty_max = 25;
    top = 149;
  }
  else if (freq_id == PWM_FREQ_50_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_4;
    duty_max = 50;
    top = 239;
  }
  else if (freq_id == PWM_FREQ_60_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_4;
    duty_max = 50;
    top = 199;
  }
  else if (freq_id == PWM_FREQ_75_KHZ) {
    freq_prescaler = TIMER4_CLK_DIV_4;
    duty_max = 50;
    top = 159;
  }

  // Configure Timer4
  if (duty <= duty_max) {
    m_freq_id = freq_id;
    m_duty = duty;

    timer4->setup_pwm(freq_prescaler, top, m_duty);
  }
}

void PWM::send_setup()
{
  comm->write_msg_header(COMM_PWM_SETUP, m_freq_id, m_duty);
}