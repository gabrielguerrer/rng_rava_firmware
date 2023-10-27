/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <Arduino.h>

#include <rava_rng.h>
#include <rava_timers.h>
#include <rava_comm.h>
#include <rava_config.h>
#include <rava_eeprom.h>
#include <rava_health.h>
#include <rava_tools.h>

extern TIMER0* timer0;
extern TIMER1* timer1;
extern TIMER3* timer3;
extern COMM* comm;
extern EEPROM* eeprom;
extern HEALTH_CONTINUOUS* rng_health_continuous;

uint8_t bit_lshift_mask[8] = {1, 2, 4, 8, 16, 32, 64, 128};

RNG::RNG()
{
  // Setup RNG timers
  timer0->setup_arduino_and_rng();
  timer1->setup_rng();

  // Initialize post processing byte functions address array
  random_byte_func_addrs[0] = &RNG::read_byte;
  random_byte_func_addrs[1] = &RNG::read_byte_pp_xor;
  random_byte_func_addrs[2] = &RNG::read_byte_pp_xor_dichtl;
  random_byte_func_addrs[3] = &RNG::read_byte_pp_von_neumann;

  // Configure sampling interval. Read from EEPROM
  setup(true);
}

bool RNG::validate_sampling_interval(uint8_t sampling_interval_us)
{
  if (sampling_interval_us == 0)
    return false;
  return true;
}

bool RNG::validate_bit_source(uint8_t bit_source)
{
  if ((bit_source == 0) || (bit_source > 5))
    return false;
  return true;
}

bool RNG::validate_postproc_id(uint8_t postproc_id)
{
  if (postproc_id > 3)
    return false;
  return true;
}

bool RNG::validate_byte_stream_delay(uint16_t interval_ms)
{
  if (interval_ms > TIMER3_MAXIMUM_DELAY_MS)
    return false;
  return true;
}

bool RNG::validate_int_delta(uint8_t int_delta)
{
  if (int_delta < 2)
    return false;
  return true;
}

bool RNG::validate_int_delta(uint16_t int_delta)
{
  if (int_delta < 2)
    return false;
  return true;
}

void RNG::setup(bool eeprom_value, uint8_t sampling_interval_us)
{
  // Read from EEPROM
  if (eeprom_value)
    eeprom->read_rng(&sampling_interval_us);

  // Validate pars and config
  if (validate_sampling_interval(sampling_interval_us))
    m_sampling_interval_us = sampling_interval_us;
}

uint8_t RNG::get_sampling_interval()
{
  return m_sampling_interval_us;
}

void RNG::send_setup()
{
  comm->write_msg_header(COMM_RNG_SETUP, m_sampling_interval_us);
}

void RNG::setup_timing_debug_d1(bool on)
{
  if (on) {
    // D1 mode = output
    PORTE &= ~_BV(6); // Output = low
    DDRE |= _BV(6); // PE6 as output
    d1_timing_debug = true;
  }
  else {
    // D1 mode = input
    DDRE &= ~_BV(6); // PE6 as input
    d1_timing_debug = false;
  }
}

void RNG::read_initialize()
{
  // Disable MCU interrupts during measurements
  cli();

  // Timer0 to external clock during measurements
  timer0->clock_external();
}

void RNG::read_finalize()
{
  // Re-enable MCU interrupts
  sei();

  // Timer0 to internal clock
  timer0->clock_internal();
}

void RNG::read_pulse_count(uint8_t* rng_a, uint8_t* rng_b)
{
  // Reset counters
  timer1->reset_counter();
  timer0->reset_counter();

  // Wait for the sampling interval
  delayMicroseconds(m_sampling_interval_us);

  // Measure counters
  *rng_a = timer1->read_counter();
  *rng_b = timer0->read_counter();
}

void RNG::send_pulse_counts(uint32_t n_counts)
{
  // Send header
  comm->write_msg_header(COMM_RNG_PULSE_COUNTS, n_counts);

  // Initialize
  read_initialize();

  // Loop, measure, and write pulse counts
  uint8_t pc[2];
  for (uint32_t i = 0; i < n_counts; i++) {

    // Timing debug
    if (d1_timing_debug)
      PORTE = 0b01000000; // PE6 (D1) output = HI

    // Read pulse counts
    read_pulse_count(&pc[0], &pc[1]);

    // Timing debug
    if (d1_timing_debug)
      PORTE = 0; // PE6 (D1) output = LO

    // Write pulse counts
    comm->write(pc, 2);
  }

  // Finalize
  read_finalize();
}

void RNG::read_bit(uint8_t* rng_a, uint8_t* rng_b, uint8_t &bit_source)
{
  uint8_t rnd[2];

  // BIT_SRC_AB_RND
  if (bit_source == BIT_SRC_AB_RND) {

    // Generate one bit from each channel
    uint8_t bit_source_temp = BIT_SRC_AB;
    read_bit(&rnd[0], &rnd[1], bit_source_temp);

    // XOR the result and define the channel
    if (rnd[0] ^ rnd[1])
      bit_source = BIT_SRC_B;
    else
      bit_source = BIT_SRC_A;
  }

  // BIT_SRC_A
  if (bit_source == BIT_SRC_A)
  {
    // Reset counters
    timer1->reset_counter();

    // Wait for the sampling interval
    delayMicroseconds(m_sampling_interval_us);

    // Measure counters
    rnd[0] = timer1->read_counter();

    // Bit value derives from count parity
    *rng_a = rnd[0] & 1;
  }

  // BIT_SRC_B
  else if (bit_source == BIT_SRC_B)
  {
    // Reset counters
    timer0->reset_counter();

    // Wait for the sampling interval
    delayMicroseconds(m_sampling_interval_us);

    // Measure counters
    rnd[0] = timer0->read_counter();

    // Bit value derives from count parity
    *rng_a = rnd[0] & 1;
  }

  // BIT_SRC_AB, BIT_SRC_AB_XOR
  else
  {
    // Reset counters
    timer1->reset_counter();
    timer0->reset_counter();

    // Wait for the sampling interval
    delayMicroseconds(m_sampling_interval_us);

    // Measure counters
    rnd[0] = timer1->read_counter();
    rnd[1] = timer0->read_counter();

    // Bit value derives from count parity
    *rng_a = rnd[0] & 1;
    *rng_b = rnd[1] & 1;

    if (bit_source == BIT_SRC_AB_XOR)
      *rng_a ^= *rng_b;
  }
}

void RNG::send_bits(uint8_t bit_source)
{
  // Validate bit_source
  if (!validate_bit_source(bit_source))
    return;

  // Initialize
  read_initialize();

  // Timing debug
  if (d1_timing_debug)
    PORTE = 0b01000000; // PE6 (D1) output = HI

  // Read random bit
  uint8_t rnd[2];
  read_bit(&rnd[0], &rnd[1], bit_source);

  // Timing debug
  if (d1_timing_debug)
    PORTE = 0; // PE6 (D1) output = LO

  // Send bits
  comm->write_msg_header(COMM_RNG_BITS, bit_source, rnd[0], rnd[1]);

  // Finalize
  read_finalize();
}

void RNG::read_byte(uint8_t* rng_a, uint8_t* rng_b)
{
  // Clear output bytes
  *rng_a = 0;
  *rng_b = 0;

  // Bits loop
  uint8_t rnd[2];
  for (uint8_t i = 0; i < 8; i++)
  {
    // Reset counters
    timer1->reset_counter();
    timer0->reset_counter();

    // Wait for the sampling interval
    delayMicroseconds(m_sampling_interval_us);

    // Measure counters
    rnd[0] = timer1->read_counter();
    rnd[1] = timer0->read_counter();

    // Bit value derives from count parity
    // If odd, enable the corresponding bit in the byte output
    if (rnd[0] & 1)
      *rng_a |= bit_lshift_mask[i]; // Equivalent to *rng_a |= 1 << i; but faster
    if (rnd[1] & 1)
      *rng_b |= bit_lshift_mask[i];
  }

  // Health monitoring
  #if defined(FIRMWARE_HEALTH_CONTINUOUS_ENABLED)
  rng_health_continuous->run_tests(rng_a, rng_b);
  #endif
}

void RNG::read_byte_pp_xor(uint8_t* rng_a, uint8_t* rng_b)
{
  // Measure bytes
  uint8_t rnd1[2], rnd2[2];
  read_byte(&rnd1[0], &rnd1[1]);
  read_byte(&rnd2[0], &rnd2[1]);

  // XOR bytes
  *rng_a = rnd1[0] ^ rnd2[0];
  *rng_b = rnd1[1] ^ rnd2[1];
}

void RNG::read_byte_pp_xor_dichtl(uint8_t* rng_a, uint8_t* rng_b)
{
  // Measure bytes
  uint8_t rnd1[2], rnd2[2];
  read_byte(&rnd1[0], &rnd1[1]);
  read_byte(&rnd2[0], &rnd2[1]);

  // XOR bytes a la Dichtl
  *rng_a = (rnd1[0] ^ byte_rol(rnd1[0], 1)) ^ rnd2[0];
  *rng_b = (rnd1[1] ^ byte_rol(rnd1[1], 1)) ^ rnd2[1];
}

void RNG::read_byte_pp_von_neumann(uint8_t* rng_a, uint8_t* rng_b)
{
  // Reset output bytes
  *rng_a = 0;
  *rng_b = 0;

  // Define required variables
  uint8_t pc1[2], pc2[2];
  bool bit_rdy[2];
  uint8_t bit1[2], bit2[2];

  // Bits loop
  for (uint8_t i = 0; i < 8; i++)
  {
    bit_rdy[0] = false;
    bit_rdy[1] = false;

    // Create bits until they satisfy VN condition
    while (!( bit_rdy[0] &&  bit_rdy[1])) {

      // Read pulse counts
      read_pulse_count(&pc1[0], &pc1[1]);
      read_pulse_count(&pc2[0], &pc2[1]);

      // RNG A
      if (!bit_rdy[0]) {

        // Transform pulse counts to bits
        bit1[0] = pc1[0] & 1;
        bit2[0] = pc2[0] & 1;

        // Check VN condition
        if (bit1[0] != bit2[0])
           bit_rdy[0] = true;
      }

      // RNG B
      if (!bit_rdy[1]) {

        // Transform pulse counts to bits
         bit1[1] = pc1[1] & 1;
         bit2[1] = pc2[1] & 1;

        // Check VN condition
        if (bit1[1] != bit2[1])
           bit_rdy[1] = true;
      }
    }

    // Assign bit values according to the VN definition
    // 01 -> 0 ; 10 -> 1
    if (bit1[0] > bit2[0])
      *rng_a |= bit_lshift_mask[i]; // Equivalent to *rng_a |= _BV(i); but faster
    if (bit1[1] > bit2[1])
      *rng_b |= bit_lshift_mask[i];
  }

  // Health monitoring
  #if defined(FIRMWARE_HEALTH_CONTINUOUS_ENABLED)
  rng_health_continuous->run_tests(rng_a, rng_b);
  #endif
}

void RNG::send_bytes(uint32_t n_bytes, uint8_t postproc_id, uint8_t comm_id, uint8_t request_id)
{
  // Validate postproc_id
  if (!validate_postproc_id(postproc_id))
    return;

  // Link to appropriate post-processing functiong
  RNG::byte_func_addr read_byte_func = random_byte_func_addrs[postproc_id];

  // Send header
  comm->write_msg_header(comm_id, n_bytes, request_id);

  // Initialize
  read_initialize();

  // Loop over n_bytes
  uint8_t rnd[2];
  for (uint32_t i = 0; i < n_bytes; i++) {

    // Timing debug
    if (d1_timing_debug)
      PORTE = 0b01000000; // PE6 (D1) output = HI

    // Read, test, and post-process random bytes
    (this->*read_byte_func)(&rnd[0], &rnd[1]);

    // Timing debug
    if (d1_timing_debug)
      PORTE = 0; // PE6 (D1) output = LO

    // Write random bytes
    comm->write(rnd, 2);
  }

  // Finalize
  read_finalize();
}

uint8_t RNG::gen_int8(uint8_t int_delta)
{
  // Vars
  uint8_t min_bits = (uint8_t)ceil(log(int_delta)/log(2));
  uint8_t min_bits_mask = (uint8_t)bit_mask_1s(min_bits);
  uint8_t rnd_int;

  // Loop and generate
  uint8_t rnd[2];
  while (true) {

    // Generate two random bytes
    read_byte(&rnd[0], &rnd[1]);

    // Xor them
    rnd_int = rnd[0] ^ rnd[1];

    // Apply bit mask
    rnd_int &= min_bits_mask;

    // Test range
    if (rnd_int < int_delta) {
      break;
    }
  }

  return rnd_int;
}

void RNG::send_int8s(uint32_t n_ints, uint8_t int_delta)
{
  // Validate postproc_id
  if (!validate_int_delta(int_delta))
    return;

  // Send header
  comm->write_msg_header(COMM_RNG_INT8S, n_ints);

  // Initialize
  read_initialize();

  // Generate and send ints
  uint8_t rnd8;
  for (uint32_t i=0; i < n_ints; i++) {
    rnd8 = gen_int8(int_delta);
    comm->write(rnd8);
  }

  // Finalize
  read_finalize();
}

uint16_t RNG::gen_int16(uint16_t int_delta)
{
  // Vars
  uint8_t min_bits = (uint8_t)ceil(log(int_delta)/log(2));
  uint16_t min_bits_mask = (uint16_t)bit_mask_1s(min_bits);
  uint8_t min_bytes = (uint8_t)ceil((float)min_bits/8);
  uint16_t rnd_int;

  // Loop and generate
  uint8_t rnd[2];
  while (true) {
    for (uint8_t i=0; i<min_bytes; i++) {

      // Generate two random bytes
      read_byte(&rnd[0], &rnd[1]);

      // Xor them and shift as required
      if (i == 0)
        rnd_int = rnd[0] ^ rnd[1];
      else
        rnd_int |= (rnd[0] ^ rnd[1]) << (i*8);
    }

    // Apply bit mask
    rnd_int &= min_bits_mask;

    // Test range
    if (rnd_int < int_delta) {
      break;
    }
  }

  return rnd_int;
}

void RNG::send_int16s(uint32_t n_ints, uint16_t int_delta)
{
  // Validate postproc_id
  if (!validate_int_delta(int_delta))
    return;

  // Send header
  comm->write_msg_header(COMM_RNG_INT16S, n_ints);

  // Initialize
  read_initialize();

  // Generate and send ints
  uint16_t rnd16;
  for (uint32_t i=0; i < n_ints; i++) {
    rnd16 = gen_int16(int_delta);
    comm->write(rnd16);
  }

  // Finalize
  read_finalize();
}

void RNG::start_bytes_stream(uint16_t n_bytes, uint8_t postproc_id, uint16_t interval_ms)
{
  // Validate pars
  if (!validate_postproc_id(postproc_id))
    return;
  if (!validate_byte_stream_delay(interval_ms))
    return;

  // Initialize vars
  stream_cfg.streaming = true;
  stream_cfg.ready = true;
  stream_cfg.n_bytes = n_bytes;
  stream_cfg.postproc_id = postproc_id;
  stream_cfg.interval_ms = interval_ms;

  // Delay type
  if (interval_ms == 0) {
    // See task_rng_stream_zero_delay()
  }
  else {
    // Configure Timer3
    timer3->setup_rng_interrupt(interval_ms); // Activates ISR (TIMER3_COMPA_vect)
  }
}

void RNG::stop_bytes_stream()
{
  stream_cfg.streaming = false;
  timer3->reset();
}

void RNG::send_bytes_stream()
{
  // The ready variable is used to prevent the device from becoming unresponsive
  // when the time took by send_bytes_stream() is larger than the stream delay
  stream_cfg.ready=false;
  send_bytes(stream_cfg.n_bytes, stream_cfg.postproc_id, COMM_RNG_STREAM_BYTES);
  stream_cfg.ready=true;
}

void RNG::send_bytes_stream_status()
{
  comm->write_msg_header(COMM_RNG_STREAM_STATUS, (uint8_t)stream_cfg.streaming);
}