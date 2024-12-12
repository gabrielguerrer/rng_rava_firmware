/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The PWM BOOST module configures Timer4 to produce the PWM signal on port B6 fed
to the Boost converter, the electronic module responsible for increasing the
USB 5V input into a higher voltage applied to the noise sources.

The PWM configuration accepts two parameters: the PWM frequency and the PWM duty
cycle (ranging from 0 to 255). During initialization, these values are loaded
from EEPROM memory, but it is also possible to reconfigure them during runtime
without affecting the EEPROM values.
*/

#ifndef RAVA_PWM_BOOST_H
#define RAVA_PWM_BOOST_H

#include <stdint.h>

enum PWM_BOOST_FREQUENCIES {
  PWM_BOOST_FREQ_30_KHZ=1,  // Timer4 clk/8, 200 ticks
  PWM_BOOST_FREQ_40_KHZ,    // Timer4 clk/8, 150 ticks
  PWM_BOOST_FREQ_50_KHZ,    // Timer4 clk/4, 240 ticks
  PWM_BOOST_FREQ_60_KHZ,    // Timer4 clk/4, 200 ticks
  PWM_BOOST_FREQ_75_KHZ     // Timer4 clk/4, 160 ticks
};

class PWM_BOOST
{
  public:
    PWM_BOOST();

    bool validate_setup_pars(uint8_t freq_id, uint8_t duty);
    void setup(bool eeprom_values, uint8_t freq_id=0, uint8_t duty=0);
    void setup_resume();
    void stop();
    void send_setup();

  private:
    bool configured = false;
    uint8_t _freq_prescaler, _top;
    uint8_t _freq_id, _duty;
};

#endif