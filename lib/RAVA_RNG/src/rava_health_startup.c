/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "rava_health_startup.h"
#include "rava_rng.h"

#define H_STARTUP_PULSE_COUNT_N 3125UL
#define H_STARTUP_PULSE_COUNT_AVG_PER_SAMPLING_INTERVAL 2.
#define H_STARTUP_PULSE_COUNT_AVG_DIFF_MIN 0.9

#define H_STARTUP_BIASCHISQ_NBYTES 3125UL   // 25K bits
#define H_STARTUP_BIAS_PERC_TRESHOLD 1.397  // norm(0, 1/np.sqrt(4*25000)).isf(.00001 / 2) * 100
#define H_STARTUP_CHISQ_MAX_TRESHOLD 362.99 // chi2(255).isf(.00001)

// Test Parameters
static uint32_t pc_n_counts = H_STARTUP_PULSE_COUNT_N;
static uint32_t bias_n_bytes = H_STARTUP_BIASCHISQ_NBYTES;

// Test Results
typedef struct health_startup_tests_t
{
  uint8_t result;
  float a, b, tresh;
} health_startup_tests_t;

static health_startup_tests_t pc = {0, 0.0f, 0.0f, 0.0f};
static health_startup_tests_t pc_diff = {0, 0.0f, 0.0f, H_STARTUP_PULSE_COUNT_AVG_DIFF_MIN};
static health_startup_tests_t bias = {0, 0.0f, 0.0f,  H_STARTUP_BIAS_PERC_TRESHOLD};
static health_startup_tests_t chisq = {0, 0.0f, 0.0f, H_STARTUP_CHISQ_MAX_TRESHOLD};

// Result
static bool hs_result = false;

// Funtions
static bool test_pulse_count_average(void);
static bool test_bias(void);

/* ===========================
 * RAVA HEALTH STARTUP
 * =========================== */

/*
Executes the startup health tests and returns the overall test result.
*/
bool health_startup_run_tests(void)
{
  hs_result = true;

  // Pulse count average
  hs_result &= test_pulse_count_average();

  // Bias
  hs_result &= test_bias();

  return hs_result;
}

/*
Returns the overall result of the most recently executed startup health test.
*/
bool health_startup_get_tests_result(void)
{
  return hs_result;
}

/*
Implements the pulse count statistical test.
*/
static bool test_pulse_count_average(void)
{
  uint8_t count_a, count_b;
  uint8_t count_a_prev, count_b_prev;
  uint32_t pc_a=0, pc_b=0;
  uint32_t pc_diff_a=0, pc_diff_b=0;

  // Initialize
  rng_read_initialize();

  // Enable global interrupts for USB enumeration
  sei();

  // Loop
  for (uint32_t i = 0; i < pc_n_counts; i++) {

    // Measure
    rng_gen_pulse_count(&count_a, &count_b);

    // Compute
    pc_a += count_a;
    pc_b += count_b;

    if (i == 0) {
      count_a_prev = count_a;
      count_b_prev = count_b;
    }
    else {
      pc_diff_a += abs(count_a - count_a_prev);
      pc_diff_b += abs(count_b - count_b_prev);

      count_a_prev = count_a;
      count_b_prev = count_b;
    }
  }

  // Finalize
  rng_read_finalize();

  // Compute averages
  pc.a = (float)pc_a / pc_n_counts;
  pc.b = (float)pc_b / pc_n_counts;

  pc_diff.a = (float)pc_diff_a / (pc_n_counts - 1);
  pc_diff.b = (float)pc_diff_b / (pc_n_counts - 1);

  // Test
  pc.tresh = (float)rng_get_sampling_interval() * H_STARTUP_PULSE_COUNT_AVG_PER_SAMPLING_INTERVAL;

  if ((pc.a >= pc.tresh) && (pc.b >= pc.tresh)) {
    pc.result = true;
    }
  else
    pc.result = false;

  if ((pc_diff.a > pc_diff.tresh) && (pc_diff.b > pc_diff.tresh)) {
    pc_diff.result = true;
    }
  else
    pc_diff.result = false;

  return pc.result && pc_diff.result;
}

/*
Implements bit-bias and byte-bias tests.
*/
static bool test_bias(void)
{
  // Bit bias
  uint8_t rnd_a, rnd_b;
  uint32_t n1s_a=0, n1s_b=0;
  uint8_t freq_a[256] = {0};
  uint8_t freq_b[256] = {0};

  // Initialize
  rng_read_initialize();

  // Enable global interrupts for USB enumeration
  sei();

  // Loop
  for (uint32_t i = 0; i < bias_n_bytes; i++) {

    // Measure
    rng_gen_byte(&rnd_a, &rnd_b);

    // Compute
    n1s_a += hamming_weight_8(rnd_a);
    n1s_b += hamming_weight_8(rnd_b);
    freq_a[rnd_a] += 1;
    freq_b[rnd_b] += 1;
  }

  // Finalize
  rng_read_finalize();

  // Compute bit bias
  bias.a = ((float)n1s_a / (8*bias_n_bytes) - 0.5) * 100;
  bias.b = ((float)n1s_b / (8*bias_n_bytes) - 0.5) * 100;

  // Test
  if ((fabs(bias.a) <= bias.tresh) && (fabs(bias.b) <= bias.tresh)) {
    bias.result = true;
    }
  else {
    bias.result = false;
  }

  // Byte bias
  float chisq_temp;
  float freq_expect = (float)bias_n_bytes / 256;
  chisq.a = 0;
  chisq.b = 0;

  // Compute byte chisq
  for (uint16_t i = 0; i < 256; i++) {
    chisq_temp = (float)freq_a[i] - freq_expect;
    chisq_temp *= chisq_temp;
    chisq.a += chisq_temp / freq_expect;

    chisq_temp = (float)freq_b[i] - freq_expect;
    chisq_temp *= chisq_temp;
    chisq.b += chisq_temp / freq_expect;
  }

  // Test
  if ((chisq.a < chisq.tresh) && (chisq.b < chisq.tresh)) {
    chisq.result = true;
    }
  else {
    chisq.result = false;
  }

  return bias.result && chisq.result;
}

/* ===========================
 * COMM
 * =========================== */

/*
Processes the request to execute the startup health tests and send the results.
*/
void comm_health_startup_run(comm_interface_t *const comm)
{
  // Run Tests
  health_startup_run_tests();

  // Send
  comm_health_startup_get_results(comm);
}

/*
Processes the request to send the global result of the startup health validation together with the
results of the individual startup tests.
*/
void comm_health_startup_get_results(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {bool result; health_startup_tests_t pc, pc_diff, bias, chisq;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  data_out.result = hs_result;
  data_out.pc = pc;
  data_out.pc_diff = pc_diff;
  data_out.bias = bias;
  data_out.chisq = chisq;

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}