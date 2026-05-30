/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <string.h>
#include "rava_rng.h"

// Global configuration variables
rng_config_t rng_cfg = {0};
rng_byte_stream_t rng_byte_stream_cfg = {0};

// Bit yield variables
static uint16_u bit_yield_buffer = {.i=0};
static uint8_t bits_yield_left = 0;

// Functions
static uint8_t rng_gen_bit_yield(uint8_t postproc_id);
static void rng_gen_byte_pp_xor(uint8_t *const byte_a, uint8_t *const byte_b);
static void rng_gen_byte_pp_xor_dichtl(uint8_t *const byte_a, uint8_t *const byte_b);

static void rng_write_pulse_counts(comm_interface_t *const comm, uint16_t n_counts, uint8_t rng_cores);
static void rng_write_bit(comm_interface_t *const comm, uint8_t rng_cores);
static void rng_write_bytes(comm_interface_t *const comm, uint16_t n_bytes, uint8_t rng_cores, uint8_t postproc_id);

static void rng_write_int8s(comm_interface_t *const comm, uint16_t n_ints, uint8_t int_delta, uint8_t postproc_id);
static void rng_write_int16s(comm_interface_t *const comm, uint16_t n_ints, uint16_t int_delta, uint8_t postproc_id);
static void rng_write_floats(comm_interface_t *const comm, uint16_t n_floats, uint8_t postproc_id);
static void rng_write_floats_downey(comm_interface_t *const comm, uint16_t n_floats, uint8_t postproc_id);

// Lookup table of random-byte generation methods
typedef void (*gen_byte_fn)(uint8_t *const byte_a, uint8_t *const byte_b);

static const gen_byte_fn gen_byte_fns[3] = {
  rng_gen_byte,
  rng_gen_byte_pp_xor,
  rng_gen_byte_pp_xor_dichtl,
};

/* ===========================
 * RAVA RNG
 * =========================== */

/*
Configures the sampling interval.
*/
bool rng_setup(uint8_t sampling_interval)
{
  rng_cfg.sampling_interval = sampling_interval;
  return true;
}

/*
Returns the sampling interval.
*/
uint8_t rng_get_sampling_interval(void)
{
  return rng_cfg.sampling_interval;
}

/*
Implements bit-yield functionality by generating two random bytes and sequentially returning their
individual bits until all 16 bits have been consumed.
*/
uint8_t rng_gen_bit_yield(uint8_t postproc_id)
{
  // Generate bytes
  if (bits_yield_left == 0) {
    gen_byte_fns[postproc_id](&bit_yield_buffer.b[0], &bit_yield_buffer.b[1]);
    bits_yield_left = 16;

    // Health monitoring
    rng_health_monitoring(&bit_yield_buffer.b[0], &bit_yield_buffer.b[1]);
  }

  uint8_t bit = bit_yield_buffer.i & 1;
  bit_yield_buffer.i >>= 1;
  bits_yield_left--;

  return bit;
}

/*
Post-processing: Generates pairs of random bytes and outputs their XOR combination.
*/
void rng_gen_byte_pp_xor(uint8_t *const byte_a, uint8_t *const byte_b)
{
  // Measure bytes
  uint8_t byte_aa, byte_bb;
  rng_gen_byte(byte_a, byte_b);
  rng_gen_byte(&byte_aa, &byte_bb);

  // XOR bytes
  *byte_a = *byte_a ^ byte_aa;
  *byte_b = *byte_b ^ byte_bb;
}

/*
Post-processing: Generates pairs of random bytes and outputs their XOR combination using the method
described in:
M. Dichtl, "Bad and Good Ways of Post-processing Biased Physical Random Numbers",
Fast Software Encryption (FSE 2007), Springer, 2007.
*/
void rng_gen_byte_pp_xor_dichtl(uint8_t *const byte_a, uint8_t *const byte_b)
{
  // Measure bytes
  uint8_t byte_aa, byte_bb;
  rng_gen_byte(byte_a, byte_b);
  rng_gen_byte(&byte_aa, &byte_bb);

  // XOR bytes a la Dichtl
  *byte_a = xor_dichtl(*byte_a, byte_aa);
  *byte_b = xor_dichtl(*byte_b, byte_bb);
}

/*
Generates and sends pulse counts.
*/
void rng_write_pulse_counts(comm_interface_t *const comm, uint16_t n_counts, uint8_t rng_cores)
{
  // Vars
  uint8_t pc_a, pc_b;

  // Initialize

  rng_read_initialize();

  // Loop
  for (uint16_t i = 0; i < n_counts; i++) {

    /////////////////////////////
    // Generate

    // Timing debug
    rng_timing_debug_on();

    // Generate pulse counts
    rng_gen_pulse_count(&pc_a, &pc_b);

    // Timing debug
    rng_timing_debug_off();

    /////////////////////////////
    // Send

    if (rng_cores == RNG_CORES_AB_DUAL || rng_cores == RNG_CORES_AB_ALT) {
      comm->write(pc_a);
      comm->write(pc_b);
    }
    else if (rng_cores == RNG_CORES_A) {
      comm->write(pc_a);
    }
    else if (rng_cores == RNG_CORES_B) {
      comm->write(pc_b);
    }
  }

  // Finalize
  rng_read_finalize();

  // Flush
  comm->flush();
}

/*
Generates and sends random bits.
*/
void rng_write_bit(comm_interface_t *const comm, uint8_t rng_cores)
{
  // Vars
  uint8_t bit_a, bit_b;

  // Initialize
  rng_read_initialize();

  /////////////////////////////
  // Generate

  // Timing debug
  rng_timing_debug_on();

  // Generate bit
  rng_gen_bit(&bit_a, &bit_b);

  // Timing debug
  rng_timing_debug_off();

  /////////////////////////////
  // Send

  if (rng_cores == RNG_CORES_AB_DUAL || rng_cores == RNG_CORES_AB_ALT) {
    comm->write(bit_a);
    comm->write(bit_b);
  }
  else if (rng_cores == RNG_CORES_AB_XOR) {
    comm->write(bit_a ^ bit_b);
  }
  else if (rng_cores == RNG_CORES_A) {
    comm->write(bit_a);
  }
  else if (rng_cores == RNG_CORES_B) {
    comm->write(bit_b);
  }

  // Finalize
  rng_read_finalize();

  // Flush
  comm->flush();
}

/*
Generates and sends random bytes.
*/
void rng_write_bytes(comm_interface_t *const comm, uint16_t n_bytes, uint8_t rng_cores, uint8_t postproc_id)
{
  // Vars
  uint8_t byte_a, byte_b;

  // Initialize
  rng_read_initialize();

  // Loop
  for (uint16_t i = 0; i < n_bytes; i++) {

    /////////////////////////////
    // Generate

    // Timing debug
    rng_timing_debug_on();

    // Generate bytes (and post-process accordingly)
    gen_byte_fns[postproc_id](&byte_a, &byte_b);

    // Health monitoring
    rng_health_monitoring(&byte_a, &byte_b);

    // Timing debug
    rng_timing_debug_off();

    /////////////////////////////
    // Send

    if (rng_cores == RNG_CORES_AB_DUAL || rng_cores == RNG_CORES_AB_ALT) {
      comm->write(byte_a);
      comm->write(byte_b);
    }
    else if (rng_cores == RNG_CORES_AB_XOR) {
      comm->write(byte_a ^ byte_b);
    }
    else if (rng_cores == RNG_CORES_A) {
      comm->write(byte_a);
    }
    else if (rng_cores == RNG_CORES_B) {
      comm->write(byte_b);
    }
  }

  // Finalize
  rng_read_finalize();

  // Flush
  comm->flush();
}

/*
Called by the implementation app at a fixed interval to generate and send random bytes.
*/
void rng_write_byte_stream(void)
{
  // IO Structure
  typedef struct {uint8_t rng_cores;} data_out_t;
  data_out_t data_out = {.rng_cores = rng_byte_stream_cfg.rng_cores};

  // Process Output
  uint16_t n_bytes_out = ((rng_byte_stream_cfg.rng_cores == RNG_CORES_AB_DUAL || rng_byte_stream_cfg.rng_cores == RNG_CORES_AB_ALT) ? 2 : 1) * rng_byte_stream_cfg.n_bytes;

  // Inject original req_id, and comm_id in comm
  rng_byte_stream_cfg.comm->msg.req_id = rng_byte_stream_cfg.req_id;
  rng_byte_stream_cfg.comm->msg.comm_id = COMM_RNG_GEN_BYTES;

  // Send Header
  send_rava_msg_header(rng_byte_stream_cfg.comm, CE_OK, n_bytes_out, sizeof(data_out), &data_out);

  // Send Bytes
  rng_write_bytes(rng_byte_stream_cfg.comm, rng_byte_stream_cfg.n_bytes, rng_byte_stream_cfg.rng_cores, rng_byte_stream_cfg.postproc_id);
}

/*
Generates and sends 8-bit integers uniformly distributed over the interval [0, int_delta].
*/
void rng_write_int8s(comm_interface_t *const comm, uint16_t n_ints, uint8_t int_delta, uint8_t postproc_id)
{
  // Vars
  uint8_t int_a, int_b;
  uint8_t bit_mask = gen_int8s_mask(int_delta);

  // Initialize
  rng_read_initialize();

  // Loop
  for (uint16_t i=0; i < n_ints; ) {

    /////////////////////////////
    // Generate

    // Generate random bytes
    gen_byte_fns[postproc_id](&int_a, &int_b);

    // Health monitoring
    rng_health_monitoring(&int_a, &int_b);

    // Apply bit mask
    int_a &= bit_mask;
    int_b &= bit_mask;

    /////////////////////////////
    // Test range and Send

    if (int_a <= int_delta) {
      i++;
      comm->write(int_a);
    }
    if (i == n_ints) {
      break;
    }
    if (int_b <= int_delta) {
      i++;
      comm->write(int_b);
    }
  }

  // Finalize
  rng_read_finalize();

  // Flush
  comm->flush();
}

/*
Generates and sends 16-bit integers uniformly distributed over the interval [0, int_delta].
*/
void rng_write_int16s(comm_interface_t *const comm, uint16_t n_ints, uint16_t int_delta, uint8_t postproc_id)
{
  // Vars
  uint16_u int_a, int_b;
  uint16_t bit_mask = gen_int16s_mask(int_delta);
  uint8_t min_bytes = (int_delta > 0xFF) ? 2 : 1;

  // Initialize
  rng_read_initialize();

  // Loop
  for (uint16_t i=0; i < n_ints; ) {

    /////////////////////////////
    // Generate

    // Generate bytes
    for (uint8_t j=0; j < min_bytes; j++) {
      gen_byte_fns[postproc_id](&int_a.b[j], &int_b.b[j]);

      // Health monitoring
      rng_health_monitoring(&int_a.b[j], &int_b.b[j]);
    }

    // Apply bit mask
    int_a.i &= bit_mask;
    int_b.i &= bit_mask;

    /////////////////////////////
    // Test range and Send
    if (int_a.i <= int_delta) {
      i++;
      comm->write_buf(int_a.b, 2);
    }
    if (i == n_ints) {
      break;
    }
    if (int_b.i <= int_delta) {
      i++;
      comm->write_buf(int_b.b, 2);
    }
  }

  // Finalize
  rng_read_finalize();

  // Flush
  comm->flush();
}

/*
Generates and sends floats uniformly distributed over the interval [0, 1).
*/
void rng_write_floats(comm_interface_t *const comm, uint16_t n_floats, uint8_t postproc_id)
{
  // Vars
  float_u f_a, f_b;

  // Initialize
  rng_read_initialize();

  // Loop
  for (uint16_t i=0; i < n_floats; ) {

    /////////////////////////////
    // Generate

    // Generate 3 random bytes
    for (uint8_t j=0; j < 3; j++)  {
      gen_byte_fns[postproc_id](&f_a.b[j], &f_b.b[j]);

      // Health monitoring
      rng_health_monitoring(&f_a.b[j], &f_b.b[j]);
    }

    // IEEE754 bit pattern for single precision floating point value in the range of 1.0 - 2.0
    // Uses the first 23 bits and fixes the float exponent to 127
    f_a.i = (f_a.i & 0x007FFFFF) | 0x3F800000;
    f_a.f -= 1;

    f_b.i = (f_b.i & 0x007FFFFF) | 0x3F800000;
    f_b.f -= 1;

    /////////////////////////////
    // Send

    i++;
    comm->write_buf(f_a.b, 4);

    if (i == n_floats) {
      break;
    }

    i++;
    comm->write_buf(f_b.b, 4);
  }

  // Finalize
  rng_read_finalize();

  // Flush
  comm->flush();
}

/*
Generates and sends floats uniformly distributed over the interval [0, 1) using the method
described in:
Downey, Allen B. "Generating pseudo-random floating-point values"
https://www.allendowney.com/research/rand/downey07randfloat.pdf
*/
void rng_write_floats_downey(comm_interface_t *const comm, uint16_t n_floats, uint8_t postproc_id)
{
  // Vars
  int8_t exp;
  uint32_t mant;
  float_u f;

  // Initialize
  rng_read_initialize();

  // Loop
  for (uint16_t i=0; i < n_floats; i++) {

    /////////////////////////////
    // Generate

    // Allen Downey's algorithm to generate a random float

    // Exponent (geometric distribution)
    exp = 126;
    while (rng_gen_bit_yield(postproc_id) == 0 && exp > 0) {
      exp--;
    }

    // Mantissa (23 bits)
    mant = 0;
    for (uint8_t j = 0; j < 23; j++) {
      mant = (mant << 1) | rng_gen_bit_yield(postproc_id);
    }

    f.i = ((uint32_t)exp << 23) | mant;

    /////////////////////////////
    // Send
    comm->write_buf(f.b, 4);
  }

  // Finalize
  rng_read_finalize();

  // Flush
  comm->flush();
}


/* ===========================
 * COMM
 * =========================== */

/*
Processes the request to send the current RNG configuration.
*/
 void comm_rng_get_config(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {rng_config_t rng_cfg;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  data_out.rng_cfg = rng_cfg;

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/*
Processes the request to apply the provided RNG configuration parameters.
*/
void comm_rng_set_config(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {rng_config_t rng_cfg;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  // Process Output
  if (!(
    pwm_boost_validate_setup_pars(&data_in.rng_cfg.pwm_boost) &&
    rng_validate_setup_pars(data_in.rng_cfg.sampling_interval) &&
    pwm_boost_setup(&data_in.rng_cfg.pwm_boost) &&  // Setup PWM
    rng_setup(data_in.rng_cfg.sampling_interval)    // Setup RNG
    )) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to enable/disable byte-generation timing debugging, exposed as a digital
signal on a designated device output pin.
*/
void comm_rng_set_timing_debug(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t on;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  // data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  // Process Output
  rng_setup_timing_debug(data_in.on);

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to send pulse-count measurements.
*/
void comm_rng_gen_pulse_counts(comm_interface_t *const comm) {
  // IO Structure
  typedef struct {uint16_t n_counts; uint8_t rng_cores;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;
  uint16_t n_bytes;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  if (!(data_in.n_counts > 0 &&
        data_in.n_counts <= RNG_GEN_MAX_NBYTES_PER_CORE &&
        data_in.rng_cores < RNG_CORES_ENUM_LAST &&
        data_in.rng_cores != RNG_CORES_AB_XOR)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Process Output
  n_bytes = ((data_in.rng_cores == RNG_CORES_AB_DUAL || data_in.rng_cores == RNG_CORES_AB_ALT) ? 2 : 1) * data_in.n_counts;

  // Send Header
  send_rava_msg_header(comm, CE_OK, n_bytes, 0, NULL);

  // Send PCs
  rng_write_pulse_counts(comm, data_in.n_counts, data_in.rng_cores);
}

/*
Processes the request to send random bits.
*/
void comm_rng_gen_bit(comm_interface_t *const comm){
  // IO Structure
  typedef struct {uint8_t rng_cores;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;
  uint16_t n_bytes;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  if (!(data_in.rng_cores < RNG_CORES_ENUM_LAST)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Process Output
  n_bytes = (data_in.rng_cores == RNG_CORES_AB_DUAL || data_in.rng_cores == RNG_CORES_AB_ALT) ? 2 : 1;

  // Send Header
  send_rava_msg_header(comm, CE_OK, n_bytes, 0, NULL);

  // Send Bit
  rng_write_bit(comm, data_in.rng_cores);
}

/*
Processes the request to send random bytes.

The parameter n_bytes specifies the number of bytes generated per RNG core and must not exceed
RNG_GEN_MAX_NBYTES_PER_CORE.
*/
void comm_rng_gen_bytes(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint16_t n_bytes; uint8_t rng_cores, postproc_id;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;
  uint16_t n_bytes_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  if (!(data_in.n_bytes > 0 &&
        data_in.n_bytes <= RNG_GEN_MAX_NBYTES_PER_CORE &&
        data_in.rng_cores < RNG_CORES_ENUM_LAST &&
        data_in.postproc_id < RNG_PP_ENUM_LAST) ) {

      send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
      return;
    }

  // Process Output
  n_bytes_out = ((data_in.rng_cores == RNG_CORES_AB_DUAL || data_in.rng_cores == RNG_CORES_AB_ALT) ? 2 : 1) * data_in.n_bytes;

  // Send Header
  send_rava_msg_header(comm, CE_OK, n_bytes_out, 0, NULL);

  // Send Bytes
  rng_write_bytes(comm, data_in.n_bytes, data_in.rng_cores, data_in.postproc_id);
}

/*
Processes the request to send u8 integers in the range [0, int_delta].
*/
void comm_rng_gen_int8s(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint16_t n_ints; uint8_t int_delta, postproc_id;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;
  uint16_t n_bytes;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  if (!(data_in.n_ints > 0 &&
        data_in.n_ints <= RNG_GEN_MAX_NBYTES_PER_CORE*2 &&
        data_in.int_delta > 0 &&
        data_in.postproc_id < RNG_PP_ENUM_LAST)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Process Output
  n_bytes = data_in.n_ints;

  // Send Header
  send_rava_msg_header(comm, CE_OK, n_bytes, 0, NULL);

  // Send Int8s
  rng_write_int8s(comm, data_in.n_ints, data_in.int_delta, data_in.postproc_id);
}

/*
Processes the request to send u16 integers in the range [0, int_delta].
*/
void comm_rng_gen_int16s(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint16_t n_ints, int_delta; uint8_t postproc_id;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;
  uint16_t n_bytes;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  if (!(data_in.n_ints > 0 &&
        data_in.n_ints <= RNG_GEN_MAX_NBYTES_PER_CORE &&
        data_in.int_delta > 0 &&
        data_in.postproc_id < RNG_PP_ENUM_LAST)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Process Output
  n_bytes = 2 * data_in.n_ints;

  // Send Header
  send_rava_msg_header(comm, CE_OK, n_bytes, 0, NULL);

  // Send Int16s
  rng_write_int16s(comm, data_in.n_ints, data_in.int_delta, data_in.postproc_id);
}

/*
Processes the request to send 4-byte floats in the range [0, 1).
*/
void comm_rng_gen_floats(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint16_t n_floats; uint8_t postproc_id;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;
  uint16_t n_bytes;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  if (!(data_in.n_floats > 0 &&
        data_in.n_floats <= RNG_GEN_MAX_NBYTES_PER_CORE/2 &&
        data_in.postproc_id < RNG_PP_ENUM_LAST)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Process Output
  n_bytes = 4 * data_in.n_floats;

  // Send Header
  send_rava_msg_header(comm, CE_OK, n_bytes, 0, NULL);

  // Send Floats
  rng_write_floats(comm, data_in.n_floats, data_in.postproc_id);
}

/*
Processes the request to send 4-byte floats in the range [0, 1) using Downey's method.
*/
void comm_rng_gen_floats_downey(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint16_t n_floats; uint8_t postproc_id;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;
  uint16_t n_bytes;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  if (!(data_in.n_floats > 0 &&
        data_in.n_floats <= RNG_GEN_MAX_NBYTES_PER_CORE &&
        data_in.postproc_id < RNG_PP_ENUM_LAST)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Process Output
  n_bytes = 4 * data_in.n_floats;

  // Send Header
  send_rava_msg_header(comm, CE_OK, n_bytes, 0, NULL);

  // Send Floats
  rng_write_floats_downey(comm, data_in.n_floats, data_in.postproc_id);
}

/*
Processes the request to start periodic random-byte streaming.
*/
void comm_rng_start_byte_stream(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint16_t n_bytes, stream_interval_ms; uint8_t rng_cores, postproc_id;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Validate Input
  if (!(data_in.n_bytes > 0 &&
        data_in.n_bytes <= RNG_GEN_MAX_NBYTES_PER_CORE &&
        data_in.stream_interval_ms <= rng_byte_stream_cfg.interval_ms_max &&
        data_in.rng_cores < RNG_CORES_ENUM_LAST &&
        data_in.postproc_id < RNG_PP_ENUM_LAST)) {

      send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
      return;
    }

  // Process Output
  rng_start_byte_stream(comm, data_in.n_bytes, data_in.stream_interval_ms, data_in.rng_cores, data_in.postproc_id);

  // Send Header
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to stop periodic random-byte streaming.
*/
void comm_rng_stop_byte_stream(comm_interface_t *const comm)
{
  // Process Output
  rng_stop_byte_stream();

  // Send Header
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to inform whether fixed-interval random-byte generation is enabled and the
associated request identifier.
*/
void comm_rng_get_status_byte_stream(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {bool streaming; uint16_t req_id;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  data_out.streaming = rng_byte_stream_cfg.streaming;
  if (data_out.streaming) {
    data_out.req_id = rng_byte_stream_cfg.req_id;
  }
  else {
    data_out.req_id = 0;
  }

  // Send Header
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}