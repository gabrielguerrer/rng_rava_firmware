/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

/*
The ATmega32u4 microcontroller features four counter/timers (0, 1, 3, and 4) and 
a watchdog timer. The entropy generation within the RAVA circuit relies on 
timers 0 and 1 for pulse counting and on timer 4 for the PWM signal. Timer 3 can 
be optionally employed to generate random bytes at predefined intervals or 
allocating its functionality to another application related to rava_peripherals. 
The watchdog timer functions as a 60Hz clock, supporting the operations of the 
LED and LAMP modules.

Next, a more detailed description of each timer class.

// TIMER0
TIMER0 is employed to count digital pulses originating from the PD7 port 
connected to CMP2. This operation constitutes the foundation for generating 
entropy within the randomness channel designated as "B".

TIMER0 is also employed the Arduino library to provide functions such as 
micros(), millis(), and delay(). The Timer0 setup is defined in the init() 
function of the wiring.c file as follows:

TCCR0A |= _BV(WGM01) | _BV(WGM00); // Fast PWM, TOP at 0xff  
TCCR0B |= _BV(CS01) | _BV(CS00); // clk_IO / 64
TIMSK0 |= _BV(TOIE0); // Ovflow int. enable; Occurs every 256/(16M/64)s=1.024 ms

To broaden the circuit's application range, this functionality remains partially 
undisturbed: During the random bit generation process, the timer 0 clock 
configuration is temporarily switched to an external source, being reverted to 
its original configuration upon task completion. 

// TIMER1
TIMER1 is dedicated to count digital pulses originating from the PD6 port 
connected to CMP1. This operation constitutes the foundation for generating 
entropy within the randomness channel designated as "A".

// TIMER3
TIMER3 may be employed by the RNG module to trigger the production and 
transmission of random bytes in a regular interval, or it can be exposed as a 
peripheral for custom applications as PWM, trigger output, and interrupt at a 
given interval -- see rava_peripherals for more details.

The maximum interval that can be achieved is 4194 ms, given by 2**16/(16M/1024).

// TIMER4
TIMER4 is dedicated to provide the PWM signal used by the boost converter module 
which converts the 5V USB input into a higher voltage used to create the 
avalanche noise -- see rava_pwm for more details.

It configures the PLL Postcaler Factor to run clk_USB and clk_TMR at 48MHz.

// WDT
WDT functions as a 60Hz clock -- see rava_led and rava_lamp for more details. 
*/

#ifndef RAVA_TIMERS_H
#define RAVA_TIMERS_H

#include <stdint.h>

#define TIMER3_MAXIMUM_DELAY_MS 4194

enum TIMERS013_CLOCK {
  TIMER013_CLK_OFF,
  TIMER013_CLK_DIV_1,
  TIMER013_CLK_DIV_8,
  TIMER013_CLK_DIV_64,
  TIMER013_CLK_DIV_256,
  TIMER013_CLK_DIV_1024,
  TIMER013_CLK_EXTERN_FALLING_EDGE,
  TIMER013_CLK_EXTERN_RISING_EDGE,
};

enum TIMER4_CLOCK {
  TIMER4_CLK_OFF,
  TIMER4_CLK_DIV_1,
  TIMER4_CLK_DIV_2,
  TIMER4_CLK_DIV_4,
  TIMER4_CLK_DIV_8,
  TIMER4_CLK_DIV_16
};

enum WDT_CLOCK {
  WDT_INTERVAL_16MS,
  WDT_INTERVAL_32MS,
  WDT_INTERVAL_64MS,
  WDT_INTERVAL_125MS,
  WDT_INTERVAL_250MS,
  WDT_INTERVAL_500MS,
  WDT_INTERVAL_1S,
  WDT_INTERVAL_2S,
  WDT_INTERVAL_4S = 32,
  WDT_INTERVAL_8S,
};

class TIMER0
{
  public:
    void reset();    
    void setup_arduino_and_rng();
    void clock_internal();
    void clock_external();

    void reset_counter();
    uint8_t read_counter();
};

class TIMER1
{
  public:
    void reset();
    void setup_rng();
    
    void reset_counter();
    uint8_t read_counter();
};

class TIMER3
{
  public:
    void reset();
    uint16_t setup_clock(uint16_t delay_ms);
    void setup_rng_interrupt(uint16_t delay_ms);
    void setup_trigger_output(uint16_t delay_ms);
    void setup_pwm(uint8_t freq_prescaler, uint16_t top, uint16_t duty);
    void setup_input_capture();
};

class TIMER4
{
  public:
    void reset();
    void setup_pll();
    void setup_pwm(uint8_t freq_prescaler, uint8_t top, uint8_t duty);
};

class WDT
{
  public:
    WDT();
    void reset();
    void setup_interrupt(uint8_t freq_prescaler);

    void reboot_device();
};

#endif