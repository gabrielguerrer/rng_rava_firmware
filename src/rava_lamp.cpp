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
#include <rava_timers.h>
#include <rava_tools.h>

extern RNG* rng;
extern LED* led;
extern EEPROM* eeprom;
extern COMM* comm;
extern TIMER1* timer1;
extern TIMER3* timer3;

#define LAMP_TICK_INTERVAL_MS 50 // Lamp clock at 20 FPS
#define LAMP_FEEDBMAG_MIN 16 // Feedback magnitude minimum value

#define DELAY_FADE_MS 1000.
#define DELAY_CO_OSCILATE_MS 4000.
#define DELAY_RESET_MS 3000.

uint8_t lamp_colors[LAMP_COLORS] = {COLOR_RED, COLOR_ORANGE, COLOR_YELLOW, COLOR_GREEN,
                         COLOR_CYAN, COLOR_BLUE, COLOR_PURPLE, COLOR_PINK};

uint8_t gen_random_byte()
{
  // Generate random bytes
  uint8_t rnd_a, rnd_b;

  rng->read_initialize();
  rng->read_byte(&rnd_a, &rnd_b);
  rng->read_finalize();

  // XOR both bytes
  return xor_dichtl(rnd_a, rnd_b);
}

uint8_t gen_random_color(uint8_t* color_idx, uint8_t* color_shift)
{
  uint8_t rnd = gen_random_byte();
  *color_idx = 0b111 & rnd; // Ranging from 0 to 7
  
  if (*color_idx == 7) {
    *color_shift = 32;
  }
  else {
    *color_shift = lamp_colors[(*color_idx) + 1] - lamp_colors[*color_idx];
  }

  return lamp_colors[*color_idx];
}

void sound_timer1_d4(uint16_t freq_hz, uint8_t volume)
{
  if ((freq_hz == 0) || (volume == 0)){
    timer1->reset();
    return;
  }

  float pwm_top = round(2000000. / freq_hz) - 1; // top = (16MHz / 8) / freq_hz - 1
  float volume_dutyc = round((float)volume / 255 * TIMER13_SOUND_VOLUME_DUTYC_MAX);
  timer1->setup_pwm_pb7(TIMER013_CLK_DIV_8, (uint16_t)pwm_top, (uint8_t)volume_dutyc);
}

bool LAMP::validate_setup_pars(uint16_t exp_movwin_n_trials, uint16_t exp_deltahits_sigevt, uint16_t exp_dur_max_s,
  uint8_t exp_mag_smooth_n_trials, uint8_t exp_mag_colorchg_thld, uint8_t sound_volume)
{
  if ((exp_movwin_n_trials < 10) || (exp_movwin_n_trials > 1200)) {
    return false;
  }
  if (exp_deltahits_sigevt == 0) {
    return false;
  }
  if (exp_dur_max_s < 10) {
    return false;
  }
  if (exp_mag_smooth_n_trials == 0) {
    return false;
  }

  return true;
}

void LAMP::setup()
{
  on = true;

  // Read EEPROM parameters
  eeprom->read_lamp(&exp_movwin_n_trials, &exp_deltahits_sigevt, &exp_dur_max_s, &exp_mag_smooth_n_trials, &exp_mag_colorchg_thld, &sound_volume);

  // Setup LAMP clock
  timer3->setup_interrupt(LAMP_TICK_INTERVAL_MS, 1);

  // Initiate ticks vars
  setup_ticks();

  // Initiate color-oscilating sounds
  setup_sound();

  // Allocate vars
  exp_feedb_mags = (uint8_t*)malloc(exp_mag_smooth_n_trials);
  exp_movwin_hits = (uint8_t*)malloc(exp_movwin_n_trials);

  // Jump to color oscilate
  tick_jump(ticks_ins_fade_in);
}

void LAMP::setup_ticks()
{
  // Instructions
  ticks_ins_fade_in = 1;

  ticks_ins_oscilate = ticks_ins_fade_in;
  ticks_ins_oscilate += (uint16_t)round((float)DELAY_FADE_MS / LAMP_TICK_INTERVAL_MS);

  ticks_ins_fade_out = ticks_ins_oscilate;
  ticks_ins_fade_out += (uint16_t)round((float)DELAY_CO_OSCILATE_MS / LAMP_TICK_INTERVAL_MS);

  ticks_ins_exp_reset = ticks_ins_fade_out;
  ticks_ins_exp_reset += (uint16_t)round((float)DELAY_FADE_MS / LAMP_TICK_INTERVAL_MS);

  ticks_ins_end = ticks_ins_exp_reset;
  ticks_ins_end += (uint16_t)round((float)DELAY_RESET_MS / LAMP_TICK_INTERVAL_MS);

  // Experiment
  ticks_exp_start = ticks_ins_end + 1;

  ticks_exp_end = ticks_exp_start;
  ticks_exp_end += (uint32_t)round((float)exp_dur_max_s * 1000. / LAMP_TICK_INTERVAL_MS);
}

void LAMP::free_memory()
{
  free(exp_feedb_mags);
  free(exp_movwin_hits);

  // Avoid Undefined Behaviour if called twice
  exp_feedb_mags = NULL;
  exp_movwin_hits = NULL;
}

void LAMP::stop()
{
  on = false;

  // Stop LAMP clock
  timer3->reset();

  // Reconfigure Timer1 to measure RNG pulse counts
  timer1->setup_rng();

  // Free LAMP memory
  free_memory();

  // LED fade out
  led->fade_intensity(0, 1000);
}

void LAMP::setup_sound()
{
  // Sound Ticks; All last 1.9s
  ticks_ins_notes[0] = ticks_ins_oscilate + 1;
  ticks_ins_notes[1] = ticks_ins_notes[0] + 7;
  ticks_ins_notes[2] = ticks_ins_notes[1] + 6;
  ticks_ins_notes[3] = ticks_ins_notes[2] + 5;
  ticks_ins_notes[4] = ticks_ins_notes[3] + 4;
  ticks_ins_notes[5] = ticks_ins_notes[4] + 4;
  ticks_ins_notes[6] = ticks_ins_notes[5] + 12;

  // Sound Notes
  freq_co_notes[0] = 220; // A
  freq_co_notes[1] = 233; // A#
  freq_co_notes[2] = 293; // D
  freq_co_notes[3] = 329; // E
  freq_co_notes[4] = 349; // F
  freq_co_notes[5] = 440; // A
  freq_co_notes[6] = 0;
}

void LAMP::sound_invert_notes()
{
  uint16_t freq_co_notes_clone[LAMP_SOUND_NOTES];

  for (uint8_t i=0; i < LAMP_SOUND_NOTES; i++) {
    freq_co_notes_clone[i] = freq_co_notes[i];
  }

  for (uint8_t i=0; i < LAMP_SOUND_NOTES - 1; i++) {
    freq_co_notes[i] = freq_co_notes_clone[LAMP_SOUND_NOTES - 2 - i];
  }
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
  if (on == false) {
    return;
  }

  // Got a new clock tick?
  if (!tick_new) {
    return;
  }

  // Reset new tick flag
  tick_new = false;

  // Instructions
  if (tick_counter <= ticks_ins_end) {
    process_instructions();
  }

  // Experiment trial
  else if (tick_counter >= ticks_exp_start) {
    process_experiment();
  }
}

void LAMP::process_instructions()
{
  // Oscilate and jump to experiment start
  if (tick_counter == ticks_ins_fade_in) {
    led->fade_intensity(255, DELAY_FADE_MS);
  }

  else if (tick_counter == ticks_ins_oscilate) {
    led->set_intensity(255);
    led->fade_color_oscillate(3, DELAY_CO_OSCILATE_MS);
  }

  else if (tick_counter == ticks_ins_fade_out) {
    led->fade_intensity(0, DELAY_FADE_MS);
  }

  else if (tick_counter == ticks_ins_exp_reset) {
    experiment_reset_vars();
  }

  else if (tick_counter == ticks_ins_end) {
    tick_jump(ticks_exp_start);
  }

  // Sound
  if (sound_volume) {
      for (uint8_t i=0; i<LAMP_SOUND_NOTES; i++) {
        if (tick_counter == ticks_ins_notes[i]) {
          // Plays sound disrupting Timer1 pulse count standard setup. Timer1 is reconfigured on experiment_reset_vars()
          sound_timer1_d4(freq_co_notes[i], sound_volume);
          break;
        }
      }
  }
}

void LAMP::process_experiment()
{
  // Experiment running
  if (tick_counter <= ticks_exp_end) {

    // Run experiment trial; Retrieves random bytes and calculates trial_delta_hits, trial_z, trial_p, trial_mag
    experiment_trial();

    // LED Feedback
    if (trial_mag > exp_mag_colorchg_thld) {

      // Above mag treshold to start changing color?
      float color_new = ((float)trial_mag - exp_mag_colorchg_thld)/(255 - exp_mag_colorchg_thld) * trial_color_shift + trial_color;
      led->set_color((uint8_t)round(color_new), trial_mag);
    }

    else {
      led->set_color(trial_color, trial_mag);
    }

    // Send debug info?
    if (exp_send_debug) {
      send_debug();
    }

    // Check if trial_delta_hits is above significance treshold to finish the round
    if (abs(trial_delta_hits) >= exp_deltahits_sigevt) {

      // Check minimum trial time
      if (trial_i >= exp_movwin_n_trials) {

        // Update statistics vars
        exp_n += 1;
        exp_n_zsig += 1;
        exp_colors[trial_color_idx] += 1;

        // Oscilate LED colors        
        tick_jump(ticks_ins_oscilate);
      }
    }

    // Increment trial counter
    trial_i += 1;
  }

  // Experiment finished within chance
  else {
    // Update statistics vars
    exp_n += 1;
    exp_colors[trial_color_idx] += 1;

    // Fade LED out
    tick_jump(ticks_ins_fade_out);
  }
}

void LAMP::experiment_reset_vars()
{
  // Reconfigure Timer1 to measure RNG pulse counts
  timer1->setup_rng();

  // Sound alternates between ascending and descending scale
  sound_invert_notes();

  // Reset vars
  trial_i = 0;
  trial_draws = exp_movwin_n_trials * 8;
  trial_movwin_hits = exp_movwin_n_trials * 4;
  array_init(exp_movwin_hits, exp_movwin_n_trials, 4);
  array_init(exp_feedb_mags, exp_mag_smooth_n_trials, 0);
  trial_mag = 0;

  // Draw and set random color
  trial_color = gen_random_color(&trial_color_idx, &trial_color_shift);
  led->fade_stop(); // Necessary on some devices due to discrepancies between WDT and Timer3
  led->set_color(trial_color, 0);
}

void LAMP::experiment_trial()
{
  // Generate a random byte
  uint8_t trial_byte = gen_random_byte();

  // Count the number of 1s
  uint8_t trial_hits = hamming_weight_8(trial_byte);

  // Update hits counter within the moving window
  trial_movwin_hits -= exp_movwin_hits[trial_i % exp_movwin_n_trials];
  exp_movwin_hits[trial_i % exp_movwin_n_trials] = trial_hits;
  trial_movwin_hits += trial_hits;

  // Calc z-score ; Normal approximation to the binomial distribution
  trial_delta_hits = trial_movwin_hits - (trial_draws / 2);
  trial_z = (float)trial_delta_hits / sqrt((float)trial_draws * 0.25);

  // Calc p-value; Two-tailed stats
  trial_p = 1. - 2 * normal_sf(abs(trial_z));

  // Calc feedback magnitude
  uint8_t mag = (uint8_t)round(trial_p * 255);

  // Average feedback magnitude over exp_mag_smooth_n_trials values
  exp_feedb_mags[trial_i % exp_mag_smooth_n_trials] = mag;
  float mag_avg = array_sum(exp_feedb_mags, exp_mag_smooth_n_trials);
  mag_avg = round(mag_avg / exp_mag_smooth_n_trials);

  // Check minimum value
  if (mag_avg < LAMP_FEEDBMAG_MIN) {
    mag_avg = LAMP_FEEDBMAG_MIN;
  }

  trial_mag = (uint8_t)mag_avg;
}

void LAMP::experiment_debug(bool send_debug_info)
{
  exp_send_debug = send_debug_info;
}

bool LAMP::experiment_debugging()
{
  return exp_send_debug;
}

void LAMP::send_debug()
{
  if (trial_i % 4 == 0) { // Send every 200ms
    comm->write_msg_header(COMM_LAMP_DEBUG, trial_z, trial_mag, (uint8_t)6);
    comm->write((uint16_t)trial_delta_hits);
    comm->write((uint32_t)trial_i);
  }
}

void LAMP::send_statistics()
{
  comm->write_msg_header(COMM_LAMP_STATISTICS, exp_n, exp_n_zsig, (uint8_t)16);
  for (uint8_t i=0; i < LAMP_COLORS; i++) {
    comm->write(exp_colors[i]);
  }

  // Clear statistics vars
  exp_n = 0;
  exp_n_zsig = 0;
  array_init(exp_colors, LAMP_COLORS, 0);
}