/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The peripherals module provides support for the five exposed digital ports D1–D5, which can be
configured for input and output operations, as well as a variety of alternate MCU-specific
functionalities.

Each port provides access to a distinct subset of MCU peripherals and hardware capabilities,
described individually below.

// D1 (PE6)
PE6 alternate funcions are:
 - INT6 - External Interrupt
 - AIN0 - Analog Comparator Positive input

Port D1 is specially utilized in the RAVA device for:
 - Implementing fast read and write operations. Since PORTE contains only one additional channel
   (PE2, permanently connected to GND), the entire register can be written directly without
   requiring individual bit set/clear operations. This enables the functions `d1_read_fast()` and
   `d1_write_lo/hi_fast()`
 - Receiving external trigger signals through `d1_setup_trigger_input()`
 - Implemementing the positive input of analog comparator applications through
   `d1_setup_comparator()`

// D2 (PC7)
PC7 alternate funcions are:
 - ICP3 - Timer 3 Input Capture
 - CLK0 - System Clock Output
 - OC4A - Timer 4 Output Compare A

Port D2 is specially used in the RAVA device for:
  - Measuring the time interval between digital pulses through `d2_setup_timer3_input_capture()`
  - Outputting the MCU system clock signal when the CKOUT fuse bit is enabled

// D3 (PC6)
PC6 alternate funcions are:
 - OC3A - Timer 3 Output Compare A
 - _OC4A - Timer 4 Output Compare A

Port D3 is specially utilized in the RAVA device for:
 - Exposing Timer 3 functionalities through `setup_timer3_trigger_output()`, `setup_timer3_pwm()`,
   and `setup_timer3_sound()`
 - Timer 4 is reserved exclusively for PWM boost generation and is therefore not exposed through D3

// D4 (PB7)
PB7 alternate funcions are:
 - OC0A - Timer 0 Output Compare A
 - OC1C - Timer 1 Output Compare C
 - PCINT7 - Pin Change Interrupt

Port D4 is specially utilized in the RAVA device for:
 - Detecting digital voltage transitions through `d4_setup_pin_change()`

// D5 (PB5)
PB5 alternate funcions are:
 - OC1A - Timer 1 Output Compare A
 - PCINT5 - Pin Change Interrupt source
 - OC4B - Timer 4 Output Compare B
 - ADC12 - Analog-to-Digital Converter

Port D5 is specially utilized in the RAVA device for:
  - Implementing ADC-based applications
  - Implementing the negative input of analog comparator applications, enabled through
    `d1_setup_comparator(true)`
*/

#ifndef RAVA8_PERIPHERALS_H
#define RAVA8_PERIPHERALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rava_comm.h>
#include "rava8_adc.h"
#include "rava8_device.h"
#include "rava8_timers.h"

/* ===========================
 * RAVA8 PERIPHERALS
 * =========================== */

enum PERIPHERALS_MODES
{
  PERIPH_INPUT,
  PERIPH_OUTPUT,
};

enum PERIPHERALS_COMMS
{
  PERIPH_MODE,
  PERIPH_READ,
  PERIPH_WRITE,
  PERIPH_PULSE,
  PERIPH_COMM_ENUM_LAST,
};

typedef struct periph_t {
  volatile uint8_t *port;
  volatile uint8_t *ddr;
  volatile uint8_t *pin;
  uint8_t _port_i;
} periph_t;

extern const periph_t *periphs[6];

// Generic
void periph_mode_output(uint8_t port_id);
void periph_mode_input(uint8_t port_id);
void periph_write_lo(uint8_t port_id);
void periph_write_hi(uint8_t port_id);
void periph_write_pulse(uint8_t port_id, uint16_t duration_us);
bool periph_read(uint8_t port_id);

// D1
static inline void d1_write_lo_fast(void) {PORTE = 0;};
static inline void d1_write_hi_fast(void) {PORTE = 0b01000000;};
static inline void d1_write_pulse_fast(uint16_t duration_us) {d1_write_hi_fast(); device_delay_us(duration_us); d1_write_lo_fast();};
static inline bool d1_read_fast(void) {return (PINE & 0b01000000) != 0;};

// D2
typedef struct input_capture_cfg_t {
  uint16_t req_id;
  comm_interface_t *comm;
} input_capture_cfg_t;

extern input_capture_cfg_t input_capture_cfg;

// COMM
void comm_periph_digi(comm_interface_t *const comm);
void comm_periph_d1_digi_fast(comm_interface_t *const comm);
void comm_periph_d1_trigger_input(comm_interface_t *const comm);
void comm_periph_d1_comparator(comm_interface_t *const comm);
void comm_periph_d1_device_delay_us_test(comm_interface_t *const comm);
void comm_periph_d2_timer3_input_capture(comm_interface_t *const comm);
void comm_periph_d3_timer3_periodic_trigger_output(comm_interface_t *const comm);
void comm_periph_d3_timer3_pwm(comm_interface_t *const comm);
void comm_periph_d3_timer3_sound(comm_interface_t *const comm);
void comm_periph_d4_pin_change(comm_interface_t *const comm);
void comm_periph_d5_adc(comm_interface_t *const comm);

#ifdef __cplusplus
}
#endif

#endif