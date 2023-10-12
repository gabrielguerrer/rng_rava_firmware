/**
 * Copyright (c) 2023 Gabriel Guerrer
 * 
 * Distributed under the MIT license - See LICENSE for details 
 */

/*
The lamp module offers the possibility of using an attached LED to create a 
decorative object that employs the RAVA's generated bytes to randomly vary the 
LED's intensity based on the statistical deviation from chance expectations. 
The greater the deviation, the more intense the light becomes. 

In the firmware, the lamp mode is activated when the macros LED_ENABLED and 
LED_LAMP are defined in rava_config.h. When activated, the lamp mode initiates 
after the startup health tests and continues until the reception of the first 
serial command. At that moment, the lamp mode is deactivated, the LEDs are 
powered off, and the RAVA circuit resumes its regular operation.

The lamp use the MCU's Watchdog timer, which is configured to trigger the 
ISR (WDT_vect) interrupt every 16ms (refer to rava_interrupts.h), subsequently 
calling LAMP::tick_increment(). The new tick is detected by the process() 
function, invoked within the main loop of rava_firmware, leading to the 
corresponding LAMP updates.

The lamp mode relies on three parameters stored in the EEPROM memory: 
exp_dur_max_ms, exp_z_significant, and exp_mag_smooth_n_trials. Other important 
parameters mentioned below are defined on rava_lamp.cpp.

The lamp mode is a repetition of experiments lasting a maximum interval of 
exp_dur_max_ms. Each experiment starts by randomly choosing a color from eight 
possibilities. During the experiment, the color is fixed, variating its 
intensity only. 

The experiment consists of trials that occur at regular intervals defined by 
trial_interval_ms and invoked by the experiment_trial() function. The individual 
trial steps include:
 * Generating N=trial_rnd_n_bytes random bytes
 * Obtaining the quantity of 1s bits in the total of 8*trial_rnd_n_bytes draws
 * Incrementing the trial_hits variable by the 1s quantity while subtracting the 
   expected value by chance. This variable reflects the accumulated results 
   within a moving window of exp_mov_window_ms duration.
 * Computing the z-score associated with trial_hits using the normal 
   approximation to the binomial distribution
 * Transforming the z-score into a double-tailed p-value probability
 * Converting the p-value into a feedback magnitude that ranges from 0 to 255
 * Calculating the average of the last exp_mag_smooth_n_trials magnitude values
 * Adjusting the LED intensity to match the average magnitude

If the z-score exceeds the exp_z_significance threshold, the experiment 
concludes with a color oscillating pattern, similar to what is observed during 
the lamp startup. However, if the experiment's maximum duration (exp_dur_max_ms) 
is reached before this threshold is met, the experiment concludes with a gradual 
light fade-off pattern.

Users have the flexibility to modify the EEPROM parameters to achieve the 
following outcomes:
 * exp_dur_max_ms: Adjust the maximum duration of each experiment
 * exp_z_significant: Alter the probability of the experiment concluding with 
   the color-oscillating pattern
 * exp_mag_smooth_n_trials: Control the smoothness of color intensity variations

Users can refer to the measure_lamp_statistics() function on the driver side. 
This function measures the likelihood of observing a color-oscillating pattern 
over time.
*/

#ifndef RAVA_LAMP_H
#define RAVA_LAMP_H

#include <stdint.h>

class LAMP
{
  public:
    bool validate_setup_pars(uint32_t exp_dur_max_ms, float exp_z_significant, uint8_t exp_mag_smooth_n_trials);
    void setup();

    void setup_ticks();
    void tick_increment();
    void tick_jump(uint16_t jump_to);

    void process();
    void process_color_oscilate();
    void process_color_fade_out();
    void process_experiment();

    void experiment_reset_vars();
    float experiment_trial();

    void experiment_debug(bool send_debug_info);
    bool experiment_debugging();

    void send_statistics();

  private:
    // Trial vars
    uint32_t trial_i;
    uint16_t trial_interval_ms, trial_n_per_window;
    uint8_t trial_rnd_n_bytes, trial_rnd[2];
    uint32_t trial_draws, trial_hits;
    float trial_z;
    uint8_t trial_mag;

    // Experiment vars
    uint32_t exp_mov_window_ms, exp_dur_max_ms;
    uint8_t* exp_feedb_mags;
    uint8_t exp_mag_smooth_n_trials;
    float exp_z_significant;

    // Experiment statistics vars
    uint16_t exp_n=0, exp_n_zsig=0;
    uint16_t exp_colors[8] = {0};

    // Ticks vars
    volatile bool tick_new = false;
    volatile uint32_t tick_counter;

    uint16_t ticks_co_fade_in, ticks_co_oscilate, ticks_co_fade_out, ticks_co_exp_reset, ticks_co_fade_in2, ticks_co_end;
    uint16_t ticks_cfo_fade_out, ticks_cfo_exp_reset, ticks_cfo_fade_in, ticks_cfo_end;
    uint16_t ticks_trial_interval;
    uint32_t ticks_exp_start, ticks_exp_window, ticks_exp_end;

    // Debug var
    bool exp_send_debug = false;
};

#endif