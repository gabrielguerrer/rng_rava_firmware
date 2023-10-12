/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

/*
This file contains the definitions of all the interrupt functions utilized in 
the firmware code.

The users should modify the functions associated with peripheral actions to 
implement their specific needs.
*/

#ifndef RAVA_INTERRUPTS_H
#define RAVA_INTERRUPTS_H

#include <avr/io.h>

#include <rava_config.h>
#include <rava_rng.h>
#include <rava_led.h>
#include <rava_lamp.h>
#include <rava_peripherals.h>

extern RNG* rng;
extern LED* led;
extern LAMP* lamp;
extern D1* d1;
extern D2* d2;
extern D3* d3;
extern D4* d4;
extern D5* d5;

// Used by RNG bytes stream
ISR (TIMER3_COMPA_vect) 
{
  if (rng->stream_cfg.ready)
    rng->send_bytes_stream();
}

// Used by LED and LAMP tick functions
ISR (WDT_vect) 
{
  #if defined(FIRMWARE_LED_ENABLED)
  led->tick_increment();
  #endif

  #if defined(FIRMWARE_LED_ENABLED) && defined(FIRMWARE_LAMP_ENABLED)
  lamp->tick_increment();
  #endif
}

/////////////////////////////
// PERIPHERALS

#if defined(FIRMWARE_PERIPHERALS_ENABLED)

// D2::setup_timer3_input_capture()
ISR (TIMER3_CAPT_vect) 
{
  d2->send_timer3_input_capture_count();
}

// D2::setup_timer3_input_capture()
ISR (TIMER3_OVF_vect) 
{
  d2->timer3_overflow_n += 1;
}

// D1::setup_trigger_input()
ISR (INT6_vect)
{
  // Define the action to be taken on a trigger detection. Example:
  // d2->write_pulse(100);
}

// D1::setup_comparator()
ISR (ANALOG_COMP_vect)
{
  // Define the action to be taken on a comparator event
}

// D4::setup_pin_change()
ISR (PCINT0_vect)
{
  // Define the action to be taken o pin change detection
}
#endif

#endif