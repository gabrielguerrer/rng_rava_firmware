/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The ATmega32U4 features four hardware Timers/Counters (0, 1, 3, and 4), in addition to a watchdog
timer (WDT). The entropy generation relies on Timers 0 and 1 for pulse counting, while Timer 4 is
dedicated to generating the PWM signal required by the boost converter stage. Timer 3 may
optionally be used to periodically generate and transmit random bytes, or alternatively allocated
to applications implemented through `rava8_peripherals`.

The following sections provide a more detailed description of each timer class.

## TIMER0
Timer 0 is dedicated to counting digital pulses arriving at pin PD7, connected to comparator CMP2.
This operation forms the basis of entropy generation for randomness channel “B”.

## TIMER1
Timer 1 is dedicated to counting digital pulses arriving at pin PD6, connected to comparator CMP1.
This operation forms the basis of entropy generation for randomness channel “A”.

## TIMER3
Timer 3 may be employed by the RNG module to periodically trigger the generation and transmission
of random bytes. Using a clock prescaler of 1024, the maximum interval is approximately 4194 ms,
given by: 2**16/(16MHz/1024).

Additionally, Timer 3 can be utilized by the peripherals module to operate as a general-purpose
chronometer capable of generating interrupts at configurable intervals. It may also support PWM
generation, pediodic trigger outputs, and sound synthesis.

## TIMER4
Timer 4 is dedicated to generating the PWM signal that drives the boost converter module
responsible for converting the 5V USB input into the higher voltage required by the reverse-biased
Zener diodes to produce avalanche noise.

For proper operation, the PLL must be configured beforehand using `pll_setup_48mhz()`.

## WDT
The watchdog timer may be employed as an auxiliary timing source operating in parallel with the
main timers.
*/

#ifndef RAVA8_TIMERS_H
#define RAVA8_TIMERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <avr/io.h>

/* ===========================
 * RAVA8 TIMERS
 * =========================== */

#define TIMER3_MAXIMUM_DELAY_MS 4194
#define TIMER13_SOUND_VOLUME_DUTYC_MAX 100.

typedef struct pwm_config_t
{
  uint8_t freq_prescaler, top, duty;
} pwm_config_t;

enum TIMERS013_CLOCK
{
  TIMER013_CLK_OFF,
  TIMER013_CLK_DIV_1,
  TIMER013_CLK_DIV_8,
  TIMER013_CLK_DIV_64,
  TIMER013_CLK_DIV_256,
  TIMER013_CLK_DIV_1024,
  TIMER013_CLK_EXTERN_FALLING_EDGE,
  TIMER013_CLK_EXTERN_RISING_EDGE,
  TIMER013_CLK_ENUM_LAST,
};

enum TIMER4_CLOCK
{
  TIMER4_CLK_OFF,
  TIMER4_CLK_DIV_1,
  TIMER4_CLK_DIV_2,
  TIMER4_CLK_DIV_4,
  TIMER4_CLK_DIV_8,
  TIMER4_CLK_DIV_16,
  TIMER4_CLK_DIV_32,
  TIMER4_CLK_DIV_64,
  TIMER4_CLK_DIV_128,
  TIMER4_CLK_DIV_256,
  TIMER4_CLK_DIV_512,
  TIMER4_CLK_DIV_1024,
  TIMER4_CLK_ENUM_LAST
};

enum WDT_CLOCK
{
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

// PLL
void pll_setup_48mhz(void);

// TIMER0
void timer0_reset(void);
void timer0_setup_rng(void);

static inline void timer0_reset_counter(void) {TCNT0 = 0;};      // Clears the counter register
static inline uint8_t timer0_read_counter(void) {return TCNT0;}; // Returns the current counter value

// TIMER1
void timer1_reset(void);
void timer1_setup_rng(void);

static inline void timer1_reset_counter(void) {TCNT1L = 0;};      // Clears the counter register
static inline uint8_t timer1_read_counter(void) {return TCNT1L;}; // Returns the current counter value

// TIMER3
void timer3_reset(void);
void timer3_start_chronometer(void);
float timer3_stop_chronometer(void);

uint16_t timer3_setup_clock(uint16_t interval_ms);
void timer3_setup_interrupt(uint16_t interval_ms, uint8_t interrupt_ch);

void timer3_setup_pwm_pc6(uint8_t freq_prescaler, uint16_t top, uint16_t duty);
void timer3_setup_trigger_output_pc6(uint16_t interval_ms);
void timer3_setup_input_capture_pc7(void);

extern volatile uint16_t timer3_overflow_n;

// TIMER4
void timer4_reset(void);
void timer4_setup_pwm_pb6(uint8_t freq_prescaler, uint8_t top, uint8_t duty);

// WDT
void wdt_init(void);
void wdt_reset_(void);
void wdt_setup_interrupt(uint8_t freq_prescaler);
void wdt_reboot_device(void);

#ifdef __cplusplus
}
#endif

#endif