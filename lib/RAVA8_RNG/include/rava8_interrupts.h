/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Defines the interrupt service routines (ISRs) used throughout the firmware.

Users may customize the peripheral-related interrupt handlers to implement application-specific
behavior.
*/

#ifndef RAVA8_INTERRUPTS_H
#define RAVA8_INTERRUPTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/interrupt.h>
#include <rava_rng.h>
#include "rava8_config.h"
#include "rava8_peripherals.h"

/* ===========================
 * RAVA8 INTERRUPTS
 * =========================== */

// WDT

ISR (WDT_vect)
{
}

// Timer1

ISR (TIMER1_COMPA_vect)
{
}

ISR (TIMER1_COMPB_vect)
{
}

ISR (TIMER1_COMPC_vect)
{
}

// Timer3

/*
Enabled by rng_start_byte_stream().

Triggered periodically according to the sampling interval configured for the RNG byte stream
functionality.
*/
ISR (TIMER3_COMPA_vect)
{
  rng_byte_stream_cfg.triggered = true;
}

ISR (TIMER3_COMPB_vect)
{
}

ISR (TIMER3_COMPC_vect)
{
}

// PERIPHERALS
#if defined(PERIPHERALS_ENABLED)

/*
Enabled by d2_setup_timer3_input_capture()

Triggered whenever a rising-edge pulse is detected on D2.
*/
ISR (TIMER3_CAPT_vect)
{
  // d2_write_timer3_input_capture_interval();
}

/*
Enabled by d2_setup_timer3_input_capture()

Triggered whenever 1 second elapses between rising-edge pulses on D2.
*/
ISR (TIMER3_OVF_vect)
{
  // timer3_overflow_n += 1;
}

/*
Enabled by d1_setup_trigger_input().

Defines the action to be taken when rising-edge transitions are detected on D1.
*/
ISR (INT6_vect)
{
  // Ex.:
  periph_write_pulse(5, 100);
}

/*
Enabled by d1_setup_comparator().

Defines the action to be taken when the positive comparator input voltage exceeds the negative
input voltage.
*/
ISR (ANALOG_COMP_vect)
{
  // Ex.:
  periph_write_pulse(5, 100);
}

/*
Enabled by d4_setup_pin_change().

Defines the action to be taken when logic-level transitions are detected on D4.
*/
ISR (PCINT0_vect)
{
  // Ex.:
  periph_write_pulse(5, 100);
}
#endif

#ifdef __cplusplus
}
#endif

#endif