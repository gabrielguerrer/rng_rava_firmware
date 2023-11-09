/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <rava_led.h>
#include <rava_timers.h>
#include <rava_comm.h>

#define LED_INTENSITY_DIM true

extern WDT* wdt;
extern COMM* comm;

LED::LED():
intensity_dim(LED_INTENSITY_DIM)
{
  // Setup watch dog timer for LED fading
  wdt->setup_interrupt(WDT_INTERVAL_16MS);

  // Setup FastLED library
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(m_leds, LED_N);
}

void LED::set_color(uint8_t hue, uint8_t intensity)
{
  // Make intensity 128 appear half as bright as intensity 255?
  if (intensity_dim) {
    uint8_t intensity_show = intensity == 0 ? 0 : dim8_raw(127 + (intensity+1)/2);
    FastLED.showColor(CHSV(hue, 255, intensity_show));
  }
  else
    FastLED.showColor(CHSV(hue, 255, intensity));

  m_hue = hue;
  m_intensity = intensity;
}

void LED::set_intensity(uint8_t intensity)
{
  if (f_intensity.fading)
    return;

  // Make intensity 128 appear half as bright as intensity 255?
  if (intensity_dim) {
    uint8_t intensity_show = intensity == 0 ? 0 : dim8_raw(127 + (intensity+1)/2);
    FastLED.showColor(CHSV(m_hue, 255, intensity_show));
  }
  else
    FastLED.showColor(CHSV(m_hue, 255, intensity));

  m_intensity = intensity;
}

uint8_t LED::get_color()
{
  return m_hue;
}

uint8_t LED::get_intensity()
{
  return m_intensity;
}

void LED::tick_increment()
{
  f_color.tick_counter += 1;
  f_intensity.tick_counter += 1;
  tick_new = true;
}

void LED::fade_intensity(uint8_t intensity_target, uint16_t duration_ms)
{
  // Validate pars
  if (duration_ms == 0)
    return;

  int16_t int_delta = (int16_t)intensity_target - m_intensity;
  if (int_delta == 0)
    return;

  // Setup fading parameters
  f_intensity.fading = true;
  f_intensity.val_init = m_intensity;
  f_intensity.val_tgt = intensity_target;
  f_intensity.val_delta = int_delta;
  f_intensity.t_delta_ticks = round((float)duration_ms / LED_WDT_TICK_INTERVAL_MS);

  // Reset fade counter
  f_intensity.tick_counter = 0;
}

void LED::fade_color(uint16_t hue_target, uint16_t duration_ms, bool find_shortest_hue_delta)
{
  // Validate pars
  if (duration_ms == 0)
    return;

  int16_t hue_delta = hue_target - m_hue;
  if (hue_delta == 0)
    return;

  // Find shortest path
  if (find_shortest_hue_delta) {
    int16_t hue_delta1 = hue_target + 256 - m_hue;
    int16_t hue_delta2 = hue_target - 256 - m_hue;
    if ((abs(hue_delta1) <= abs(hue_delta)) && (abs(hue_delta1) < abs(hue_delta2))) {
      hue_delta = hue_delta1;
    }
    else if ((abs(hue_delta2) <= abs(hue_delta)) && (abs(hue_delta2) < abs(hue_delta1))) {
      hue_delta = hue_delta2;
    }
  }

  // Setup fading parameters
  f_color.fading = true;
  f_color.val_init = m_hue;
  f_color.val_tgt = hue_target;
  f_color.val_delta = hue_delta;
  f_color.t_delta_ticks = round((float)duration_ms / LED_WDT_TICK_INTERVAL_MS);

  // Reset fade counter
  f_color.tick_counter = 0;
}

void LED::fade_color_oscillate(uint8_t n_cycles, uint16_t duration_ms)
{
  // Validate pars
  if (n_cycles == 0)
    return;
  if (duration_ms == 0)
    return;

  int16_t hue_target = n_cycles*256 + m_hue;
  fade_color(hue_target, duration_ms, false);
}

void LED::fade_stop()
{
  f_intensity.fading = false;
  f_color.fading = false;
}

void LED::fade_process()
{
  // Got a new counter tick?
  if (!tick_new)
    return;

  // Reset new tick flag
  tick_new = false;

  // Intensity fade
  if (f_intensity.fading)
  {
    if (f_intensity.tick_counter < f_intensity.t_delta_ticks) {
      float intensity = (float)f_intensity.tick_counter / f_intensity.t_delta_ticks * f_intensity.val_delta
                        + f_intensity.val_init;
      uint8_t intensity_int = (uint8_t)round(intensity);
      set_color(m_hue, intensity_int);
    }
    else {
      f_intensity.fading = false;
      set_color(m_hue, f_intensity.val_tgt);
    }
  }

  // Color fade
  if (f_color.fading)
  {
    if (f_color.tick_counter < f_color.t_delta_ticks) {
      float _color = (float)f_color.tick_counter / f_color.t_delta_ticks * f_color.val_delta + f_color.val_init;
      uint8_t color_int = (uint8_t)round(_color);
      set_color(color_int, m_intensity);
    }
    else {
      f_color.fading = false;
      set_color(f_color.val_tgt, m_intensity);
    }
  }
}

void LED::send_status()
{
  comm->write_msg_header(COMM_LED_STATUS, m_hue, m_intensity, f_color.fading, f_intensity.fading);
}