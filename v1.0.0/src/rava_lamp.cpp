/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

#include <rava_lamp.h>
#include <rava_rng.h>
#include <rava_led.h>
#include <rava_eeprom.h>
#include <rava_comm.h>
#include <rava_tools.h>

extern RNG* rng;
extern LED* led;
extern EEPROM* eeprom;
extern COMM* comm;
extern COMM_USB* usb;

#define LAMP_TRIAL_INTERVAL_MS 34 // 30 fps
#define LAMP_TRIAL_NBYTES 1 // 8 draws per trial (240 draws/s)
// #define LAMP_EXP_MOVING_WINDOW_MS 51000 // 51s (12K draws)
#define LAMP_EXP_MOVING_WINDOW_MS 30000 // 30s (7056 draws)
#define LAMP_FEDBMAG_MIN 63

#define DELAY_FADE_MS 1000
#define DELAY_CO_OSCILATE_MS 4000
#define DELAY_RESET_MS 2000

#define EXP_DURATION_MIN_MS 60000
#define EXP_Z_SIGNIFICANT_ABS_MIN 1.0

uint8_t led_colors[8] = {COLOR_RED, COLOR_ORANGE, COLOR_YELLOW, COLOR_GREEN, 
                         COLOR_CYAN, COLOR_BLUE, COLOR_PURPLE, COLOR_PINK};

uint8_t get_led_color_idx(uint8_t led_color_hue)
{
  uint8_t i;
  for (i=0; i < 8; i++) {
    if (led_color_hue == led_colors[i]) {
      break;
    }
  }
  return i;
}

bool LAMP::validate_setup_pars(uint32_t exp_dur_max_ms, float exp_z_significant, uint8_t exp_mag_smooth_n_trials)
{
  if (exp_dur_max_ms < EXP_DURATION_MIN_MS)
    return false;
  if (abs(exp_z_significant) < EXP_Z_SIGNIFICANT_ABS_MIN)
    return false;
  if (exp_mag_smooth_n_trials == 0)
    return false;
  return true;
}

void LAMP::setup()
{
  // Read default parameters
  trial_interval_ms = LAMP_TRIAL_INTERVAL_MS;
  trial_rnd_n_bytes = LAMP_TRIAL_NBYTES;
  exp_mov_window_ms = LAMP_EXP_MOVING_WINDOW_MS;
  
  // Read EEPROM parameters  
  eeprom->read_lamp(&exp_dur_max_ms, &exp_z_significant, &exp_mag_smooth_n_trials);

  // Initiate ticks vars
  setup_ticks();

  // Initiate exp vars
  free(exp_feedb_mags);
  exp_feedb_mags = (uint8_t*)malloc(exp_mag_smooth_n_trials);
  
  trial_n_per_window = (uint16_t)round((float)exp_mov_window_ms / trial_interval_ms);  

  // Jump to color oscilate
  tick_jump(ticks_co_fade_in);
}

void LAMP::setup_ticks()
{
  // Color oscilate
  ticks_co_fade_in = 1;

  ticks_co_oscilate = ticks_co_fade_in;
  ticks_co_oscilate += (uint16_t)round((float)DELAY_FADE_MS / LED_WDT_TICK_INTERVAL_MS);
  
  ticks_co_fade_out = ticks_co_oscilate;
  ticks_co_fade_out += (uint16_t)round((float)DELAY_CO_OSCILATE_MS / LED_WDT_TICK_INTERVAL_MS);

  ticks_co_exp_reset = ticks_co_fade_out;
  ticks_co_exp_reset += (uint16_t)round((float)DELAY_FADE_MS / LED_WDT_TICK_INTERVAL_MS);
  
  ticks_co_fade_in2 = ticks_co_exp_reset;
  ticks_co_fade_in2 += (uint16_t)round((float)DELAY_RESET_MS / LED_WDT_TICK_INTERVAL_MS);

  ticks_co_end = ticks_co_fade_in2;
  ticks_co_end += (uint16_t)round((float)DELAY_FADE_MS / LED_WDT_TICK_INTERVAL_MS);

  // Color fade out
  ticks_cfo_fade_out = ticks_co_end + 1;

  ticks_cfo_exp_reset = ticks_cfo_fade_out;
  ticks_cfo_exp_reset += (uint16_t)round((float)DELAY_FADE_MS / LED_WDT_TICK_INTERVAL_MS);

  ticks_cfo_fade_in = ticks_cfo_exp_reset;
  ticks_cfo_fade_in += (uint16_t)round((float)DELAY_RESET_MS / LED_WDT_TICK_INTERVAL_MS);

  ticks_cfo_end = ticks_cfo_fade_in;
  ticks_cfo_end += (uint16_t)round((float)DELAY_FADE_MS / LED_WDT_TICK_INTERVAL_MS);

  // Experiment
  ticks_exp_start = ticks_cfo_end + 1;
  
  ticks_exp_end = ticks_exp_start;
  ticks_exp_end += (uint32_t)round((float)exp_dur_max_ms / LED_WDT_TICK_INTERVAL_MS);

  ticks_trial_interval = (uint16_t)round((float)trial_interval_ms / LED_WDT_TICK_INTERVAL_MS);
}

void LAMP::tick_increment()
{
  tick_counter += 1.;
  tick_new = true;
}

void LAMP::tick_jump(uint16_t jump_to)
{
  tick_counter = jump_to - 1;
}

void LAMP::process()
{
  // Got a new counter tick? Happens every LED_WDT_TICK_INTERVAL_MS -- see ISR (WDT_vect)
  if (!tick_new)
    return;

  // Reset new tick flag
  tick_new = false;

  // Color oscilate
  if (tick_counter <= ticks_co_end) {
    process_color_oscilate();
  }
  // Color fade out
  else if (tick_counter <= ticks_cfo_end) {
    process_color_fade_out();
  }
  // Experiment trial. Happens every LAMP_TRIAL_INTERVAL_MS
  else {
    if (tick_counter % ticks_trial_interval == 0) {
      process_experiment();
    }
  }
}

void LAMP::process_color_oscilate()
{
  // Oscilate and jump to experiment start
  if (tick_counter == ticks_co_fade_in)
    led->fade_intensity(255, DELAY_FADE_MS);

  else if (tick_counter == ticks_co_oscilate)
    led->fade_color_oscillate(3, DELAY_CO_OSCILATE_MS);

  else if (tick_counter == ticks_co_fade_out)
    led->fade_intensity(0, DELAY_FADE_MS);

  else if (tick_counter == ticks_co_exp_reset)
    experiment_reset_vars();

  else if (tick_counter == ticks_co_fade_in2)
    led->fade_intensity(trial_mag, DELAY_FADE_MS);

  else if (tick_counter == ticks_co_end)
    tick_jump(ticks_exp_start);
}

void LAMP::process_color_fade_out()
{
  // Fade out and jump to experiment start
  if (tick_counter == ticks_cfo_fade_out)
    led->fade_intensity(0, DELAY_FADE_MS);

  else if (tick_counter == ticks_cfo_exp_reset)
    experiment_reset_vars();

  else if (tick_counter == ticks_cfo_fade_in)
    led->fade_intensity(trial_mag, DELAY_FADE_MS);

  else if (tick_counter == ticks_cfo_end)
    tick_jump(ticks_exp_start);
}

void LAMP::process_experiment()
{
  // Experiment running
  if (tick_counter <= ticks_exp_end) {

    // Run experiment trial
    float z_score = experiment_trial();

    // Check if z-score is above significance treshold and finish the round
    if (abs(z_score) > exp_z_significant) {
      
      // Update statistics vars
      exp_n += 1;
      exp_n_zsig += 1;
      uint8_t color_idx = get_led_color_idx(led->get_color());
      exp_colors[color_idx] += 1;

      // Jump to color oscilate
      tick_jump(ticks_co_oscilate);
    }
  }

  // Experiment finished within chance
  else {

    // Update statistics vars
    exp_n += 1;
    uint8_t color_idx = get_led_color_idx(led->get_color());
    exp_colors[color_idx] += 1;

    // Jump to fade out
    tick_jump(ticks_cfo_fade_out);
  }
}

void LAMP::experiment_reset_vars()
{
  // Reset
  trial_i = 0;
  trial_hits = trial_n_per_window * trial_rnd_n_bytes * 8 / 2;
  array_init(exp_feedb_mags, exp_mag_smooth_n_trials, 0);
  trial_mag = 0;

  // Pick a random color    
  uint8_t idx_color = rng->gen_int8(8);
  led->set_color(led_colors[idx_color], 0);  
}

float LAMP::experiment_trial()
{
  // Increment trial counter
  trial_i += 1; 

  // Get random bytes
  for (uint8_t i=0; i < trial_rnd_n_bytes; i++) {
    // Generate
    rng->read_byte(&trial_rnd[0], &trial_rnd[1]);
    // XOR both bytes
    trial_rnd[0] = trial_rnd[0] ^ trial_rnd[1]; 
    // Count the number of 1s
    trial_hits += hamming_weight_8(trial_rnd[0]); 
    // Correct by the expected 50%
    trial_hits -= 4;
  }

  // Calc z-score ; Normal approximation to the binomial distribution
  trial_draws = trial_n_per_window * trial_rnd_n_bytes * 8;
  float trial_draws_f = (float)trial_draws;
  trial_z = ((float)trial_hits - trial_draws_f * 0.5) / sqrt(trial_draws_f * 0.25); 
  
  // Correct z-score to a max 2.0 value
  float trial_z_correct;
  if (exp_z_significant > 2.0)
    trial_z_correct = trial_z * 2.0 / exp_z_significant;
  else
    trial_z_correct = trial_z;

  // Calc feedback magnitude
  float trial_p = 1. - 2 * normal_sf(abs(trial_z_correct)); // Two-tailed stats
  uint8_t mag = (uint8_t)round(trial_p * 255);

  // Average feedback magnitude over exp_mag_smooth_n_trials values
  exp_feedb_mags[trial_i % exp_mag_smooth_n_trials] = mag;
  uint32_t mag_avg = array_sum(exp_feedb_mags, exp_mag_smooth_n_trials) / exp_mag_smooth_n_trials;
  
  // Set min value if necessary
  if (mag_avg < LAMP_FEDBMAG_MIN)
    mag_avg = LAMP_FEDBMAG_MIN;
  
  // Trial magnitude
  trial_mag = (uint8_t)mag_avg;

  // Feedback led
  led->set_intensity(trial_mag);

  // Send debug info?
  if (exp_send_debug) {
    if (trial_i % (500/LAMP_TRIAL_INTERVAL_MS) == 0) // Send every 1/2s
      usb->write_msg_header(COMM_LAMP_DEBUG, trial_z, mag, trial_mag);
  }

  // Return z-score
  return trial_z;
}

void LAMP::experiment_debug(bool send_debug_info)
{
  exp_send_debug = send_debug_info;
}

bool LAMP::experiment_debugging()
{
  return exp_send_debug;
}

void LAMP::send_statistics()
{
  comm->write_msg_header(COMM_LAMP_STATISTICS, exp_n, exp_n_zsig, (uint8_t)16);
  
  for (uint8_t i=0; i < 8; i++) {
    comm->write(exp_colors[i]);
  }

  // Clear statistics vars
  exp_n = 0;
  exp_n_zsig = 0;
  array_init(exp_colors, 8, 0);
}