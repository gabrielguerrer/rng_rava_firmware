/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <avr/interrupt.h>
#include <rava_health_continuous.h>
#include "rava8_rng.h"
#include "rava8_config.h"
#include "rava8_device.h"
#include "rava8_timers.h"
#include "rava8_eeprom.h"

static bool timing_debug_d1 = false;

// Bit mask used for left-shift bit extraction and manipulation
static const uint8_t bit_lshift_mask[8] = {1, 2, 4, 8, 16, 32, 64, 128};

/* ===========================
 * RAVA8 RNG
 * =========================== */

/*
Initializes the RAVA8 RNG module by setting up Timers 0 and 1 for entropy acquisition, loading the
sampling interval configuration from EEPROM memory, and defining the maximum supported interval for
periodic random byte streaming.
*/
void rng_init(void)
{
  // Setup RNG timers
  timer0_setup_rng();
  timer1_setup_rng();

  // Configure sampling interval. Read from EEPROM
  rng_setup_from_eeprom();

  // Configure byte stream max interval
  rng_byte_stream_cfg.interval_ms_max = TIMER3_MAXIMUM_DELAY_MS;
}

/*
Validates the sampling interval configuration parameter.
*/
bool rng_validate_setup_pars(uint8_t sampling_interval)
{
  if (sampling_interval > 0) {
    return true;
  }
  else {
    return false;
  }
}

/*
Configures the RNG module using the configuration parameters stored in EEPROM.
*/
bool rng_setup_from_eeprom(void)
{
  // Read from EEPROM
  rng_config_t rng_cfg;
  eeprom_read_rng_cfg(&rng_cfg);

  // Validate pars and config
  if (rng_validate_setup_pars(rng_cfg.sampling_interval)) {
    return rng_setup(rng_cfg.sampling_interval);
  }
  else {
    return rng_setup(DEFAULT_RNG_SAMPLING_INTERVAL_US);
  }
}

/*
Initialization routine executed prior to entropy acquisition and random data generation.

MCU interrupts are temporarily disabled during measurements.
*/
void rng_read_initialize(void) {
  // Disable MCU interrupts during measurements
  cli();
};

/*
Finalization routine executed after entropy acquisition and random data generation.

MCU interrupts are re-enabled.
*/
void rng_read_finalize(void) {
  // Re-enable MCU interrupts
  sei();
};

/*
Enables or disables random-generation timing debugging on pin PE6 / D1.

When enabled, `rng_timing_debug_on()` and `rng_timing_debug_off()` generate a digital HIGH/LOW
signal on D1 before and after entropy acquisition, respectively. This allows the random generation
timing to be inspected using an oscilloscope or logic analyzer.
*/
void rng_setup_timing_debug(bool on)
{
  #ifdef RNG_TIMING_DEBUG_ENABLED
  if (on) {
    // D1 mode = output
    PORTE &= ~_BV(6); // Output = low
    DDRE |= _BV(6); // PE6 as output
    timing_debug_d1 = true;
  }
  else {
    // D1 mode = input
    DDRE &= ~_BV(6); // PE6 as input
    timing_debug_d1 = false;
  }
  #endif
}

/*
Sets D1 (PE6) output to HIGH state.

This functionality is conditionally compiled through the `RNG_TIMING_DEBUG_ENABLED` preprocessor
directive defined in `rava8.c`. When disabled, the associated code is completely removed at compile
time, avoiding additional CPU-cycle overhead.
*/
void rng_timing_debug_on(void)
{
  #ifdef RNG_TIMING_DEBUG_ENABLED
  if (timing_debug_d1) {
    // PE6 (D1) output = HI
    PORTE = 0b01000000;
  }
  #endif
};

/*
Sets D1 (PE6) output to LOW state.

This functionality is conditionally compiled through the `RNG_TIMING_DEBUG_ENABLED` preprocessor
directive defined in `rava8.c`. When disabled, the associated code is completely removed at compile
time, avoiding additional CPU-cycle overhead.
*/
void rng_timing_debug_off(void)
{
  #ifdef RNG_TIMING_DEBUG_ENABLED
  if (timing_debug_d1) {
    // PE6 (D1) output = LO
    PORTE = 0;
  }
  #endif
};

/*
Generates pulse-count measurements from RNG channels A and B.

The counters associated with Timers 0 and 1 are first reset, after which the MCU performs a busy
wait for the configured sampling interval. Once the interval expires, the pulse counts accumulated
from the avalanche-noise comparator outputs are read and returned.
*/
void rng_gen_pulse_count(uint8_t *const pc_a, uint8_t *const pc_b)
{
  // Reset counters
  timer1_reset_counter();
  timer0_reset_counter();

  // Wait for the sampling interval
  device_delay_us(rng_cfg.sampling_interval);

  // Measure counters
  *pc_a = timer1_read_counter();
  *pc_b = timer0_read_counter();

  // Update usage counters
  device_gen_bytes_count += 1;
}

/*
Generates random bits from RNG channels A and B.

The counters associated with Timers 0 and 1 are first reset, after which the MCU performs a busy
wait for the configured sampling interval. Once the interval expires, the pulse counts accumulated
from the avalanche-noise comparator outputs are read.

The resulting bit values are derived from the parity of the pulse counts: even counts produce
bit 0, while odd counts produce bit 1.
*/
void rng_gen_bit(uint8_t *const bit_a, uint8_t *const bit_b)
{
  // Reset counters
  timer1_reset_counter();
  timer0_reset_counter();

  // Wait for the sampling interval
  device_delay_us(rng_cfg.sampling_interval);

  // Measure counters
  *bit_a = timer1_read_counter();
  *bit_b = timer0_read_counter();

  // Bit value derives from count parity
  *bit_a &= 1;
  *bit_b &= 1;

  // Update usage counters
  device_gen_bytes_count += 1;
}

/*
Generates random bytes from RNG channels A and B.

The counters associated with Timers 0 and 1 are first reset, after which the MCU performs a busy
wait for the configured sampling interval. Once the interval expires, the pulse counts accumulated
from the avalanche-noise comparator outputs are read.

The resulting bit values are derived from the parity of the pulse counts: even counts produce
bit 0, while odd counts produce bit 1. Eight successive bits are generated and assembled into the
output bytes according to their bit positions.
*/
void rng_gen_byte(uint8_t *const byte_a, uint8_t *const byte_b)
{
  // Clear output bytes
  *byte_a = 0;
  *byte_b = 0;

  // Bits loop
  uint8_t pc_a, pc_b;
  for (uint8_t i = 0; i < 8; i++)
  {
    // Reset counters
    timer1_reset_counter();
    timer0_reset_counter();

    // Wait for the sampling interval
    device_delay_us(rng_cfg.sampling_interval);

    // Measure counters
    pc_a = timer1_read_counter();
    pc_b = timer0_read_counter();

    // Bit value derives from count parity
    // If odd, enable the corresponding bit in the byte output
    if (pc_a & 1) {
      *byte_a |= bit_lshift_mask[i]; // Equivalent to *byte_a |= 1 << i; but faster
    }
    if (pc_b & 1) {
      *byte_b |= bit_lshift_mask[i];
    }
  }

  // Update usage counters
  device_gen_bytes_count += 2;
}

/*
Runs continuous health-monitoring tests on the generated pair of random bytes.

This functionality is conditionally compiled through the `HEALTH_CONTINUOUS_ENABLED` preprocessor
directive defined in `rava8.c`. When disabled, the associated code is completely removed at compile
time, avoiding additional CPU-cycle overhead.
*/
void rng_health_monitoring(const uint8_t *const byte_a, const uint8_t *const byte_b)
{
  #ifdef HEALTH_CONTINUOUS_ENABLED
  health_continuous_run_tests(byte_a, byte_b);
  #endif
}

/*
Starts periodic random-byte streaming.

Timer 3 is configured to periodically trigger byte generation through the `TIMER3_COMPA_vect`
interrupt service routine.
*/
void rng_start_byte_stream(comm_interface_t *const comm, uint16_t n_bytes, uint16_t interval_ms,
  uint8_t rng_cores, uint8_t postproc_id)
{
  // Configure
  rng_byte_stream_cfg.req_id = comm->msg.req_id;
  rng_byte_stream_cfg.comm = comm;
  rng_byte_stream_cfg.n_bytes = n_bytes;
  rng_byte_stream_cfg.interval_ms = interval_ms;
  rng_byte_stream_cfg.postproc_id = postproc_id;
  rng_byte_stream_cfg.rng_cores = rng_cores;
  rng_byte_stream_cfg.streaming = true;

  // Stream trigger
  if (interval_ms == 0) {
    // Always on
    rng_byte_stream_cfg.triggered = true;
  }
  else {
    // Enable Timer3 interrupts
    timer3_setup_interrupt(interval_ms, 0);

    // Wait for next TIMER3_COMPA_vect interrupt
    rng_byte_stream_cfg.triggered = false;
  }
}

/*
Stops periodic random-byte streaming and resets the associated Timer 3 configuration.
*/
void rng_stop_byte_stream(void)
{
  rng_byte_stream_cfg.streaming = false;
  timer3_reset();
}