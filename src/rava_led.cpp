/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <rava_led.h>
#include <rava_timers.h>
#include <rava_eeprom.h>
#include <rava_comm.h>

extern WDT* wdt;
extern EEPROM* eeprom;
extern COMM* comm;

#define LED_PIN 30 // PD5

LED::LED()
{
  // Setup watch dog timer for fading functionality
  wdt->setup_interrupt(WDT_INTERVAL_16MS);

  // Retrieve number of leds from EEPROM
  eeprom->read_led(&led_n);
  const uint8_t const_led_n = led_n;

  // Allocate memory
  fastleds = (CRGB*)malloc(const_led_n * sizeof(CRGB));

  // Setup FastLED library
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(fastleds, const_led_n);
}

void LED::set_color(uint8_t hue, uint8_t intensity)
{
  led_hue = hue;
  led_intensity = intensity;

  if (led_n == 0) {
    return;
  }

  else if (led_n == 1) {
    fastleds[0] = CHSV(hue, 255, intensity);
  }

  else {
    // Full intensity LEDs
    uint8_t intensity_per_led = 256 / led_n;
    uint8_t leds_full = intensity / intensity_per_led;
    for (uint8_t i = 0; i < leds_full; i++) {
      fastleds[i] = CHSV(hue, 255, 255);
    }

    // Partial intensity LEDs
    uint8_t intensity_remain = intensity % intensity_per_led;
    fastleds[leds_full] = CHSV(hue, 255, led_n * intensity_remain);

    // Clear remaining
    for (uint8_t i = leds_full + 1; i < led_n; i++) {
      fastleds[i] = CHSV(hue, 255, 0);
    }
  }

  // Update LEDs
  FastLED.show();
}

void LED::set_intensity(uint8_t intensity)
{
  if (f_intensity.fading) {
    return;
  }

  set_color(led_hue, intensity);
}

uint8_t LED::get_color()
{
  return led_hue;
}

uint8_t LED::get_intensity()
{
  return led_intensity;
}

void LED::tick_increment()
{
  f_intensity.tick_counter += 1;
  f_color.tick_counter += 1;
  tick_new = true;
}

void LED::fade_intensity(uint8_t intensity_target, uint16_t duration_ms)
{
  // Validate pars
  if (duration_ms == 0) {
    return;
  }

  int16_t int_delta = (int16_t)intensity_target - led_intensity;
  if (int_delta == 0) {
    return;
  }

  // Setup parameters
  f_intensity.val_init = led_intensity;
  f_intensity.val_tgt = intensity_target;
  f_intensity.val_delta = int_delta;
  f_intensity.t_delta_ticks = round((float)duration_ms / LED_WDT_TICK_INTERVAL_MS);
  f_intensity.fading = true;

  // Reset counter
  f_intensity.tick_counter = 0;
}

void LED::fade_color(uint16_t hue_target, uint16_t duration_ms, bool find_shortestled_hue_delta)
{
  // Validate pars
  if (duration_ms == 0) {
    return;
  }

  int16_t hue_delta = hue_target - led_hue;
  if (hue_delta == 0) {
    return;
  }

  // Find shortest path
  if (find_shortestled_hue_delta) {
    int16_t hue_delta1 = hue_target + 256 - led_hue;
    int16_t hue_delta2 = hue_target - 256 - led_hue;
    if ((abs(hue_delta1) <= abs(hue_delta)) && (abs(hue_delta1) < abs(hue_delta2))) {
      hue_delta = hue_delta1;
    }
    else if ((abs(hue_delta2) <= abs(hue_delta)) && (abs(hue_delta2) < abs(hue_delta1))) {
      hue_delta = hue_delta2;
    }
  }

  // Setup parameters
  f_color.val_init = led_hue;
  f_color.val_tgt = hue_target;
  f_color.val_delta = hue_delta;
  f_color.t_delta_ticks = round((float)duration_ms / LED_WDT_TICK_INTERVAL_MS);
  f_color.fading = true;

  // Reset counter
  f_color.tick_counter = 0;
}

void LED::fade_color_oscillate(uint8_t n_cycles, uint16_t duration_ms)
{
  // Validate pars
  if (n_cycles == 0) {
    return;
  }
  if (duration_ms == 0) {
    return;
  }

  uint16_t hue_target = (uint16_t)n_cycles*256 + led_hue;
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
  if (!tick_new) {
    return;
  }

  // Reset new tick flag
  tick_new = false;

  // Intensity fade
  if (f_intensity.fading) {

    if (f_intensity.tick_counter < f_intensity.t_delta_ticks) {
      float intensity = (float)f_intensity.tick_counter / f_intensity.t_delta_ticks * f_intensity.val_delta + f_intensity.val_init;
      uint8_t intensity_int = (uint8_t)round(intensity);
      set_color(led_hue, intensity_int);
    }
    else {
      f_intensity.fading = false;
      set_color(led_hue, f_intensity.val_tgt);
    }
  }

  // Color fade
  if (f_color.fading) {

    if (f_color.tick_counter < f_color.t_delta_ticks) {
      float _color = (float)f_color.tick_counter / f_color.t_delta_ticks * f_color.val_delta + f_color.val_init;
      uint8_t color_int = (uint8_t)round(_color);
      set_color(color_int, led_intensity);
    }
    else {
      f_color.fading = false;
      set_color(f_color.val_tgt, led_intensity);
    }
  }
}

void LED::send_status()
{
  comm->write_msg_header(COMM_LED_STATUS, led_hue, led_intensity, f_color.fading, f_intensity.fading);
}