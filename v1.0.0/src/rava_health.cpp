/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

#include <rava_health.h>
#include <rava_eeprom.h>
#include <rava_led.h>
#include <rava_rng.h>
#include <rava_comm.h>
#include <rava_tools.h>

#define H_STARTUP_PULSE_COUNT_N 3125UL
#define H_STARTUP_PULSE_COUNT_AVG_PER_SAMPLING_INTERVAL 2.
#define H_STARTUP_PULSE_COUNT_AVG_DIFF_MIN 0.9

#define H_STARTUP_BIASCHISQ_NBYTES 3125UL // 25K bits
#define H_STARTUP_BIAS_PERC_TRESHOLD 1.397  // norm(0, 1/np.sqrt(4*25000)).isf(.00001 / 2) * 100
#define H_STARTUP_CHISQ_MAX_TRESHOLD 362.99 // chi2(255).isf(.00001)

#define H_CONTINUOUS_REPETITION_COUNT_CUTOFF 4 // 1 + 20 / 7.8
#define H_CONTINUOUS_ADAPTIVE_PROPORTION_W 512U
#define H_CONTINUOUS_ADAPTIVE_PROPORTION_CUTOFF 13 // 1 + binom(512, 2**(-7.8)).ppf(1-2**(-20))

extern EEPROM* eeprom;
extern LED* led;
extern RNG* rng;
extern COMM* comm;

HEALTH_STARTUP::HEALTH_STARTUP()
{
  // Read parameters
  bias_n_bytes = H_STARTUP_BIASCHISQ_NBYTES;
  bias_abs_treshold = H_STARTUP_BIAS_PERC_TRESHOLD;
  chisq_max_treshold = H_STARTUP_CHISQ_MAX_TRESHOLD;
  
  pc_n_counts = H_STARTUP_PULSE_COUNT_N;
  pc_avg_diff_min = H_STARTUP_PULSE_COUNT_AVG_DIFF_MIN;
}

void HEALTH_STARTUP::run_tests()
{   
  bool success = true;

  // Pulse count average
  if (led)
    led->set_color(COLOR_PURPLE, 127);  // Purple color mid intensity during test

  success &= test_pulse_count_average();  
    
  // Bias
  if (led)
    led->set_color(COLOR_PURPLE, 255);  // Purple color high intensity during test
  
  success &= test_bias();
  
  // Inform result by light
  if (led) {
    if (success)
      led->set_color(COLOR_RED, 0); // Success, light off
    else
      led->set_color(COLOR_RED, 255); // Failure, red light
  }

  result = success;
}

bool HEALTH_STARTUP::get_tests_result()
{
  return result;
}

void HEALTH_STARTUP::send_results() 
{
  uint8_t pc_avg_n_bytes = 25;
  uint8_t bias_bit_n_bytes = 13;
  uint8_t bias_byte_n_bytes = 13;
  comm->write_msg_header(COMM_HEALTH_STARTUP_RESULTS, (uint8_t)result, pc_avg_n_bytes, bias_bit_n_bytes, 
                         bias_byte_n_bytes);
  
  // Pulse_count
  comm->write(pc_avg_a);
  comm->write(pc_avg_b);
  comm->write(pc_avg_dif_a);
  comm->write(pc_avg_dif_b);
  comm->write(pc_avg_min);
  comm->write(pc_avg_diff_min);
  comm->write((uint8_t)pc_result);

  // Bit bias
  comm->write(bias_a);
  comm->write(bias_b);
  comm->write(bias_abs_treshold);
  comm->write((uint8_t)bias_result);

  // Byte bias
  comm->write(chisq_a);
  comm->write(chisq_b);
  comm->write(chisq_max_treshold);
  comm->write((uint8_t)chisq_result);
}

bool HEALTH_STARTUP::test_pulse_count_average()
{
  uint8_t count_a, count_b;  
  uint8_t count_a_prev, count_b_prev;  
  uint32_t pc_a=0, pc_b=0; 
  uint32_t pc_dif_a=0, pc_dif_b=0;

  rng->read_initialize();
  
  // Loop
  for (uint32_t i = 0; i < pc_n_counts; i++) {

    // Measure
    rng->read_pulse_count(&count_a, &count_b);

    // Compute
    pc_a += count_a;
    pc_b += count_b;    

    if (i == 0) {
      count_a_prev = count_a;
      count_b_prev = count_b;
    }
    else {
      pc_dif_a += abs(count_a - count_a_prev);
      pc_dif_b += abs(count_b - count_b_prev);

      count_a_prev = count_a;
      count_b_prev = count_b;
    }
  }

  rng->read_finalize();

  // Compute averages
  pc_avg_a = (float)pc_a / pc_n_counts;
  pc_avg_b = (float)pc_b / pc_n_counts;
  
  pc_avg_dif_a = (float)pc_dif_a / (pc_n_counts - 1);
  pc_avg_dif_b = (float)pc_dif_b / (pc_n_counts - 1);

  // Test
  pc_avg_min = (float)rng->get_sampling_interval() * H_STARTUP_PULSE_COUNT_AVG_PER_SAMPLING_INTERVAL;

  if ((pc_avg_a >= pc_avg_min) && 
      (pc_avg_b >= pc_avg_min) &&
      (pc_avg_dif_a > pc_avg_diff_min) &&
      (pc_avg_dif_b > pc_avg_diff_min)) {

    pc_result = true;
    }
  else
    pc_result = false;

  return pc_result;
}

bool HEALTH_STARTUP::test_bias()
{  
  // Bit bias
  uint8_t rnd_a, rnd_b;
  uint32_t n1s_a=0, n1s_b=0;  
  uint8_t freq_a[256] = {0};
  uint8_t freq_b[256] = {0};
 
  rng->read_initialize();

  // Loop
  for (uint32_t i = 0; i < bias_n_bytes; i++) {

    // Measure
    rng->read_byte(&rnd_a, &rnd_b);

    // Compute
    n1s_a += hamming_weight_8(rnd_a);
    n1s_b += hamming_weight_8(rnd_b);
    freq_a[rnd_a] += 1;
    freq_b[rnd_b] += 1;
  }

  rng->read_finalize();

  // Compute bit bias
  bias_a = ((float)n1s_a / (8*bias_n_bytes) - 0.5) * 100;
  bias_b = ((float)n1s_b / (8*bias_n_bytes) - 0.5) * 100;

  // Test
  if ((abs(bias_a) <= bias_abs_treshold) && 
      (abs(bias_b) <= bias_abs_treshold)) {
    
    bias_result = true;
    }

  // Byte bias
  float chisq_temp;
  float freq_expect = (float)bias_n_bytes / 256;
  chisq_a = 0;
  chisq_b = 0;

  // Compute byte chisq
  for (uint16_t i = 0; i < 256; i++) {
    chisq_temp = (float)freq_a[i] - freq_expect;
    chisq_temp *= chisq_temp;
    chisq_a += chisq_temp / freq_expect;

    chisq_temp = (float)freq_b[i] - freq_expect;
    chisq_temp *= chisq_temp;
    chisq_b += chisq_temp / freq_expect;
  }

  // Test
  if ((chisq_a < chisq_max_treshold) && 
      (chisq_b < chisq_max_treshold)) {
    
    chisq_result = true;
    }
  else
    chisq_result = false;

  return bias_result && chisq_result;
}

HEALTH_CONTINUOUS::HEALTH_CONTINUOUS()
{
  // Read parameters
  nrc_cutoff = H_CONTINUOUS_REPETITION_COUNT_CUTOFF;
  nap_cutoff = H_CONTINUOUS_ADAPTIVE_PROPORTION_CUTOFF;
}

void HEALTH_CONTINUOUS::run_tests(const uint8_t* const rng_a, const uint8_t* const rng_b)
{  
  // Clone rng data ; the original data cannot be modified
  m_rng_a = *rng_a;
  m_rng_b = *rng_b;

  // Run tests
  nist_repetition_count();
  nist_adaptive_proportion();
}

void HEALTH_CONTINUOUS::send_errors()
{
  comm->write_msg_header(COMM_HEALTH_CONTINUOUS_ERRORS, (uint8_t) 8);

  // Repetition count
  comm->write(nrc_error_a);
  comm->write(nrc_error_b);

  // Reset errors
  nrc_error_a = 0;
  nrc_error_b = 0;

  // Adaptive proportion
  comm->write(nap_error_a);
  comm->write(nap_error_b);
  
  // Reset errors
  nap_error_a = 0;
  nap_error_b = 0;
}

void HEALTH_CONTINUOUS::nist_repetition_count()
{  
  // RNG A
  if (m_rng_a == m_rng_prev_a) {
    nrc_counter_a++;

    if (nrc_counter_a >= nrc_cutoff)
      nrc_error_a++; // Count error on rng_a
  }
  else {
    m_rng_prev_a = m_rng_a;
    nrc_counter_a = 1;
    }
  
  // RNG B
  if (m_rng_b == m_rng_prev_b) {
    nrc_counter_b++;
    
    if (nrc_counter_b >= nrc_cutoff)
      nrc_error_b++; // Count error on rng_b
  }
  else {
    m_rng_prev_b = m_rng_b;
    nrc_counter_b = 1;
    }
}

void HEALTH_CONTINUOUS::nist_adaptive_proportion()
{
  // RNG A
  if (nap_iter_a == 0) {
    nap_target_a = m_rng_a;
    nap_counter_a = 1;
    nap_iter_a++;
  }
  else {
    if (m_rng_a == nap_target_a) {
      nap_counter_a++;

      if (nap_counter_a == nap_cutoff)
        nap_error_a++; // Count error on rng_a
    }    

    nap_iter_a++;
    if (nap_iter_a == H_CONTINUOUS_ADAPTIVE_PROPORTION_W)
      nap_iter_a = 0;
  }

  // RNG B
  if (nap_iter_b == 0) {
    nap_target_b = m_rng_b;
    nap_counter_b = 1;
    nap_iter_b++;
  }
  else {
    if (m_rng_b == nap_target_b) {
      nap_counter_b++;

      if (nap_counter_b == nap_cutoff)
        nap_error_b++; // Count error on rng_b
    }    

    nap_iter_b++;
    if (nap_iter_b == H_CONTINUOUS_ADAPTIVE_PROPORTION_W)
      nap_iter_b = 0;
  }
}