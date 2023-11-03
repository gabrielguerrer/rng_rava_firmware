/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The RNG module coordinates pulse count measurements in the differential channels
outputs (V_CMP1 and V_CMP2) to generate and transmit random bits, bytes and
integers.

The RNG configuration uses the sampling interval parameter. During
initialization, the value is loaded from EEPROM memory, but it is also possible
to reconfigure it during runtime without affecting the EEPROM values.

A single pulse count measurement proceeds in read_pulse_count() as:

// Reset counters
timer1->reset_counter();
timer0->reset_counter();

// Wait for the sampling interval
delayMicroseconds(m_sampling_interval_us);

// Measure counters
*rng_a = timer1->read_counter();
*rng_b = timer0->read_counter();

Where the variables rng_a and rng_b are assigned the pulse count measurements
obtained from the differential channels V_CMP1 and V_CMP2, respectively. The
single bit assignment in read_bit() then proceeds by:

// Bit value derives from count parity
*rng_a = rng_a & 1;
*rng_b = rng_b & 1;

This performs an AND bitwise operation to verify whether the least significant
bit of the pulse count is set to 1, indicating an odd number. Consequently, the
variables rng_a and rng_b are assigned values of 1 or 0 based on pulse counts'
parity.

The generation of a single byte in the read_byte() function is achieved by repeating
the previous operations eight times and setting a 1 bit in the resultant byte if
the pulse count is odd:

// Clear output bytes
*rng_byte_a = 0;
*rng_byte_b = 0;

// Bits loop
for (uint8_t i = 0; i < 8; i++)
  {
  ... // Pulse count measurements: rng_a, rng_b

  // If odd, enable the corresponding bit in the byte output
  if (rng_a & 1)
    *rng_byte_a |= 1 << i;
  if (rng_b & 1)
    *rng_byte_b |= 1 << i;
  }

// Health monitoring
rng_health_continuous->run_tests(rng_byte_a, rng_byte_b);

Upon the completion of random bytes rng_byte_a and rng_byte_b, they are input
into the continuous test implemented in the rava_health module, which continuously
monitors the entropy quality.

The function responsible for coordinating the generation and transmission of N
random bytes is send_bytes(). Although not essential due to the inherent output
quality, it provides the option to include a post-processing step with the
following algorithms:
* Von Neumann
* XOR
* XOR a la Dichtl (Dichtl, Markus. "Bad and good ways of post-processing biased
  physical random numbers." Fast Software Encryption: 14th International Workshop,
  FSE 2007, Luxembourg, Luxembourg, March 26-28, 2007, Revised Selected Papers 14.
  Springer Berlin Heidelberg, 2007.)

The RNG module includes the start_bytes_stream() function, which utilizes Timer3
to trigger the generation and transmission of a specific number of random bytes
at regular intervals. If the interval is set to 0, the RNG is dedicated to
producing randomness at its maximum throughput.

Summarizing, the RNG class provides methods for generating and transmitting:
* Pulse counts -- send_pulse_counts()
* Single bits -- send_bits()
* Random Bytes -- send_bytes()
* Random bytes at regular intervals -- start_bytes_stream()
* Random integers in a certain (min, max) interval -- send_int8s() and send_int16s()

Where the generation of random floats in the (0, 1) interval is achieved in the
driver level by using random bytes as input.
*/

#ifndef RAVA_RNG_H
#define RAVA_RNG_H

#include <stdint.h>

enum RNG_BIT_SOURCE {
  BIT_SRC_AB=1,
  BIT_SRC_A,
  BIT_SRC_B,
  BIT_SRC_AB_XOR,
  BIT_SRC_AB_RND
  };

enum RNG_BYTE_POST_PROCESSING {
  PP_NONE,
  PP_XOR,
  PP_XOR_DICHTL,
  PP_VON_NEUMANN
  };

struct RNG_BYTE_STREAM {
  volatile bool streaming = false;
  volatile bool triggered = false; // Used in (ISR) TIMER3_COMPA_vect
  uint16_t n_bytes;
  uint8_t postproc_id;
  uint16_t interval_ms;
};

class RNG
{
  public:
    RNG();

    bool validate_sampling_interval(uint8_t sampling_interval_us);
    bool validate_bit_source(uint8_t bit_source);
    bool validate_postproc_id(uint8_t postproc_id);
    bool validate_byte_stream_delay(uint16_t interval_ms);
    bool validate_int_delta(uint8_t int_delta);
    bool validate_int_delta(uint16_t int_delta);

    void setup(bool eeprom_value, uint8_t sampling_interval_us=0);
    uint8_t get_sampling_interval();
    void send_setup();

    void setup_timing_debug_d1(bool on);

    void read_initialize();
    void read_finalize();

    void read_pulse_count(uint8_t* rng_a, uint8_t* rng_b);
    void send_pulse_counts(uint32_t n_counts);

    void read_bit(uint8_t* rng_a, uint8_t* rng_b, uint8_t &bit_source);
    void send_bits(uint8_t bit_source);

    void read_byte(uint8_t* rng_a, uint8_t* rng_b);
    void read_byte_pp_xor(uint8_t* rng_a, uint8_t* rng_b);
    void read_byte_pp_xor_dichtl(uint8_t* rng_a, uint8_t* rng_b);
    void read_byte_pp_von_neumann(uint8_t* rng_a, uint8_t* rng_b);
    void send_bytes(uint32_t n_bytes, uint8_t postproc_id, uint8_t comm_id, uint8_t request_id=0);

    uint8_t gen_int8(uint8_t int_delta);
    void send_int8s(uint32_t n_ints, uint8_t int_delta);
    uint16_t gen_int16(uint16_t int_delta);
    void send_int16s(uint32_t n_ints, uint16_t int_delta);

    void start_bytes_stream(uint16_t n_bytes, uint8_t postproc_id, uint16_t interval_ms);
    void stop_bytes_stream();
    void send_bytes_stream();
    void send_bytes_stream_status();

    RNG_BYTE_STREAM stream_cfg;

  protected:
    typedef void (RNG::*byte_func_addr)(uint8_t* rng_a, uint8_t* rng_b);
    byte_func_addr random_byte_func_addrs[4];

    uint8_t m_sampling_interval_us;
    bool d1_timing_debug = false;
};

#endif