/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
This module introduces the LED class used to control a WS2812B LED that can be
optionally attached to the LED pin in the RAVA device.

The intensity and color fading/oscilating methods use the MCU's Watchdog timer
, which is configured to trigger the ISR (WDT_vect) interrupt every 16ms (refer
to rava_interrupts.h), subsequently calling LED::tick_increment(). The new tick
is detected by the fade_process() function, invoked within the main loop of
rava_firmware, leading to the corresponding LED updates.
*/

#ifndef RAVA_LED_H
#define RAVA_LED_H

#include <stdint.h>
#include <FastLED.h>

#define LED_WDT_TICK_INTERVAL_MS 17

#define LED_PIN 30 // PD5
#define LED_N 16

enum LED_COLOR_HUES {
  COLOR_RED = HUE_RED,
  COLOR_ORANGE = 16,
  COLOR_YELLOW = 32,
  COLOR_GREEN = HUE_GREEN,
  COLOR_CYAN = HUE_AQUA,
  COLOR_BLUE = HUE_BLUE,
  COLOR_PURPLE = HUE_PURPLE,
  COLOR_PINK = HUE_PINK
};

struct LED_FADE_TYPE {
  bool fading = false;
  uint8_t val_init;
  uint16_t val_tgt;
  int16_t val_delta;
  uint16_t t_delta_ticks;
  volatile uint16_t tick_counter;
};

class LED
{
  public:
    LED();

    void set_color(uint8_t hue, uint8_t intensity);
    void set_intensity(uint8_t intensity);
    uint8_t get_color();
    uint8_t get_intensity();

    void tick_increment();
    void fade_intensity(uint8_t intensity_target, uint16_t duration_ms);
    void fade_color(uint16_t hue_target, uint16_t duration_ms, bool find_shortest_hue_delta=true);
    void fade_color_oscillate(uint8_t n_cycles, uint16_t duration_ms);
    void fade_stop();
    void fade_process();

    void send_status();

  private:
    CRGB m_leds[LED_N];
    uint8_t m_hue=0, m_intensity=0;
    bool intensity_dim;

    volatile bool tick_new = false;
    LED_FADE_TYPE f_intensity;
    LED_FADE_TYPE f_color;
};

#endif