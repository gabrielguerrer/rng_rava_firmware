/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <math.h>

#include <rava_timers.h>

void TIMER0::reset()
{
  // Normal mode, TOP at 0xff
  TCCR0A = 0;

  // No clock source
  TCCR0B = 0;

  // Interrupts disabled
  TIMSK0 = 0;
}

void TIMER0::setup_arduino_and_rng()
{
  // Reset
  reset();

  // clk_IO / 64
  TCCR0B = TIMER013_CLK_DIV_64;

  // Overflow Interrupt Enable; occurs every 256 / (16000000 / 64) s = 1.024 ms
  TIMSK0 |= _BV(TOIE0);
}

void TIMER0::clock_internal()
{
  // clk_IO / 64. Enables millis(), micros(), and delay()
  TCCR0B = TIMER013_CLK_DIV_64;
}

void TIMER0::clock_external()
{
  // External clock source on T0 pin, clock on rising edge. Enables rng read_byte()
  TCCR0B = TIMER013_CLK_EXTERN_RISING_EDGE;
}

void TIMER0::reset_counter()
{
  TCNT0 = 0;
}

uint8_t TIMER0::read_counter()
{
  return TCNT0;
}

void TIMER1::reset()
{
  // Normal mode, TOP at 0xffff
  TCCR1A = 0;

  // No clock source
  TCCR1B = 0;
  TCCR1C = 0;

  // Interrupts disabled
  TIMSK1 = 0;
}

void TIMER1::setup_rng()
{
  // Reset
  reset();

  // External clock source on T1 pin, clock on rising edge
  TCCR1B = TIMER013_CLK_EXTERN_RISING_EDGE;
}

void TIMER1::reset_counter()
{
  TCNT1L = 0;
}

uint8_t TIMER1::read_counter()
{
  return TCNT1L;
}

void TIMER3::reset()
{
  // Normal mode, TOP at 0xffff
  TCCR3A = 0;

  // No clock source
  TCCR3B = 0;
  TCCR3C = 0;
  TIMSK3 = 0;

  // PC6 as input
  DDRC &= ~_BV(6);
}

uint16_t TIMER3::setup_clock(uint16_t interval_ms)
{
  // Assumes a 16MHz external clock
  float ticks_per_ms;

  // Find the correct prescaler and the associated number of ticks per ms
  if (interval_ms <= 32) {  // 2**16/(16MHz/8)
    TCCR3B = TIMER013_CLK_DIV_8; // clk/8
    ticks_per_ms = 2000.;  // (16MHz/8)/1000
  }
  else if (interval_ms <= 262) {  // 2**16/(16MHz/64)
    TCCR3B = TIMER013_CLK_DIV_64; // clk/64
    ticks_per_ms = 250.;  // (16MHz/64)/1000
  }
  else if (interval_ms <= 1048) {  // 2**16/(16MHz/256)
    TCCR3B = TIMER013_CLK_DIV_256; // clk/256
    ticks_per_ms = 62.5;  // (16MHz/256)/1000
  }
  else if (interval_ms <= 4194) {  // 2**16/(16MHz/1024)
    TCCR3B = TIMER013_CLK_DIV_1024; // clk/1024
    ticks_per_ms = 15.625;  // (16MHz/1024)/1000
  }
  else {
    return 0;
  }

  // Return delay ticks, the quantity of clock ticks for interval_ms
  uint16_t delay_ticks = round(float(interval_ms) * ticks_per_ms);
  delay_ticks -= 1;
  return delay_ticks;
}

void TIMER3::setup_rng_interrupt(uint16_t interval_ms)
{
  // Reset
  reset();

  // Setup clock
  OCR3A = setup_clock(interval_ms);

  // CTC mode, TOP at OCR3A
  TCCR3B |= _BV(WGM32);

  // Output Compare A Match Interrupt Enable
  TIMSK3 |= _BV(OCIE3A);  // Activates ISR(TIMER3_COMPA_vect)

  // Restart counter
  TCNT3 = 0;
}

void TIMER3::setup_trigger_output(uint16_t interval_ms)
{
  // Find 100us duty. Assumes a 16MHz external clock
  uint16_t duty_100us = 0;
  if (interval_ms <= 32) { // 2**16/(16MHz/8)
    duty_100us = 200;  // (16MHz/8)/1000 / 10
  }
  else if (interval_ms <= 262) {  // 2**16/(16MHz/64)
    duty_100us = 25;  // (16MHz/64)/1000 / 10
  }
  else if (interval_ms <= 1048) {  // 2**16/(16MHz/256)
    duty_100us = 6;  // (16MHz/256)/1000 / 10
  }
  else if (interval_ms <= 4194) {  // 2**16/(16MHz/1024)
    duty_100us = 2;  // (16MHz/1024)/1000 / 10
  }

  // Reset
  reset();

  // Setup prescaler (CS3i) and set the TOP value
  ICR3 = setup_clock(interval_ms);

  // Fast PWM, TOP at ICR3
  TCCR3A |= _BV(WGM31);
  TCCR3B |= _BV(WGM33) | _BV(WGM32);

  // Clear OC3A on compare match ; PWM output to port C6
  TCCR3A |= _BV(COM3A1);

  // Port C6: Output mode
  PORTC &= ~_BV(6);
  DDRC |= _BV(6);

  // Setup duty
  OCR3A = duty_100us;

  // Restart counter
  TCNT3 = 0;
}

void TIMER3::setup_pwm(uint8_t freq_prescaler, uint16_t top, uint16_t duty)
{
  // Reset
  reset();

  // Frequency prescaler
  TCCR3B = freq_prescaler % 6;

  // Fast PWM, TOP at ICR3
  TCCR3A |= _BV(WGM31);
  TCCR3B |= _BV(WGM33) | _BV(WGM32);

  // Clear OC3A on compare match ; PWM output to port C6
  TCCR3A |= _BV(COM3A1);

  // Port C6: Output mode
  PORTC &= ~_BV(6);
  DDRC |= _BV(6);

  // Setup top and duty
  ICR3 = top;
  OCR3A = duty;

  // Restart counter
  TCNT3 = 0;
}

void TIMER3::setup_input_capture()
{
  // Reset
  reset();

  // Setup prescaler
  TCCR3B = TIMER013_CLK_DIV_256;

  // Fast PWM mode, TOP at OCR3A, OVERFLOW at TOP
  TCCR3A |= _BV(WGM31) | _BV(WGM30);
  TCCR3B |= _BV(WGM33) | _BV(WGM32);

  // Setup prescaler
  OCR3A = 62499; // overflow every 1s ; 62500 / (16MHz/256)

  // Enable Input Capture Noise Canceler
  TCCR3B |= _BV(ICNC3);

  // Input Capture Edge Select
  TCCR3B |= _BV(ICES3); // Rising edge

  // Clear any pending Input capture flag
  TIFR3 |= _BV(ICF3);

  // Input Capture Interrupt Enable
  TIMSK3 |= _BV(ICIE3);  // Running ISR(TIMER3_CAPT_vect) defined on rava_interrupts.h

  // Overflow Interrupt Enable
  TIMSK3 |= _BV(TOIE3);  // Running ISR(TIMER3_OVF_vect) defined on rava_interrupts.h

  // Restart counter
  TCNT3 = 0;
}

void TIMER4::reset()
{
  // Normal mode, TOP at OCR4C
  TCCR4A = 0;

  // No clock source
  TCCR4B = 0;
  TCCR4C = 0;
  TCCR4D = 0;
  TCCR4E = 0;

  // B6 as input
  DDRB &= ~_BV(6);
}

void TIMER4::setup_pll()
{
  // 16MHz clock
  #if F_CPU == 16000000UL

    // PLL Postcaler Factor
    PLLCSR = 0;
    PLLCSR |= _BV(PINDIV); // Disable PLL, 16 MHz clock source

    PLLFRQ = 0;
    PLLFRQ |= _BV(PDIV3) | _BV(PDIV1);  // CLK_PLL 96MHz
    PLLFRQ |= _BV(PLLTM1) | _BV(PLLTM0); // PLLTM / 2
    PLLFRQ |= _BV(PLLUSB); // PLLUSB / 2

    PLLCSR |= _BV(PLLE); // Enable PLL

  // 8MHz clock
  #elif F_CPU == 8000000UL

    // PLL Postcaler Factor
    PLLCSR = 0; // Disable PLL, 8 MHz clock source

    PLLFRQ = 0; // PLLUSB / 1
    PLLFRQ |= _BV(PDIV2) // CLK_PLL 48MHz
    PLLFRQ |= _BV(PLLTM0); // PLLTM / 1

    PLLCSR |= _BV(PLLE); // Enable PLL

  #endif
}

void TIMER4::setup_pwm(uint8_t freq_prescaler, uint8_t top, uint8_t duty)
{
  // Reset Timer4
  reset();

  // Configure the PLL Postcaler Factor
  setup_pll();

  // Prescaling factor
  TCCR4B = freq_prescaler % 16;

  // PWM mode based on OCR4B, TOP at OCR4C
  TCCR4A |= _BV(PWM4B);

  // Clear OC4B on Compare Match ; PWM output to port B6
  TCCR4A |= _BV(COM4B1);

  // Port B6: Output mode
  PORTB &= ~_BV(6);
  DDRB |= _BV(6);

  // Define TOP value ; OC4B frequency of (48MHz / prescaling factor) / OCR4C
  OCR4C = top;

  // Set duty value
  OCR4B = duty;

  // Reset counter
  TCNT4 = 0;
}

WDT::WDT()
{
  // Clear Watchdog Reset Flag
  // To clear WDE, WDRF must be cleared first
  MCUSR &= ~_BV(WDRF);
}

void WDT::reset()
{
  // Disable Interrupts
  cli();

  // Watchdog Change Enable
  WDTCSR = _BV(WDCE) | _BV(WDE);

  // Change register
  WDTCSR = 0;

  // Enable Interrupts
  sei();
}

void WDT::setup_interrupt(uint8_t freq_prescaler)
{
  // Disable Interrupts
  cli();

  // Watchdog Change Enable
  WDTCSR = _BV(WDCE) | _BV(WDE);

  // Set timeout interval with WDIE enabled (see enum WDT_CLOCK)
  WDTCSR = _BV(WDIE) | freq_prescaler;

  // Enable Interrupts
  sei();
}

void WDT::reboot_device()
{
  // Disable Interrupts
  cli();

  // Reset the WDT timer
  wdt_reset();

  // Watchdog Change Enable
  WDTCSR = _BV(WDCE) | _BV(WDE);

	// System Reset Enable
  WDTCSR = _BV(WDE);

  // Enable Interrupts
	sei();
}