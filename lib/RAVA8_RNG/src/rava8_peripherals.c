/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <math.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <rava_comm.h>
#include <rava_rng.h>
#include "rava8_peripherals.h"
#include "rava8_comm_handlers.h"

#define PERIPHERALS_N 5

const periph_t d1 = {.port=&PORTE, .ddr=&DDRE, .pin=&PINE, ._port_i=0b01000000}; // PE6
const periph_t d2 = {.port=&PORTC, .ddr=&DDRC, .pin=&PINC, ._port_i=0b10000000}; // PC7
const periph_t d3 = {.port=&PORTC, .ddr=&DDRC, .pin=&PINC, ._port_i=0b01000000}; // PC6
const periph_t d4 = {.port=&PORTB, .ddr=&DDRB, .pin=&PINB, ._port_i=0b10000000}; // PB7
const periph_t d5 = {.port=&PORTB, .ddr=&DDRB, .pin=&PINB, ._port_i=0b00100000}; // PB5

const periph_t *periphs[6] = {NULL, &d1, &d2, &d3, &d4, &d5};
static uint8_t periph_modes[6] = {0, PERIPH_INPUT, PERIPH_INPUT, PERIPH_INPUT, PERIPH_INPUT, PERIPH_INPUT};

input_capture_cfg_t input_capture_cfg = {0};

// D1
static void d1_reset_trigger_input();
static void d1_setup_trigger_input();

static inline void d1_reset_comparator() {adc_reset();};
static void d1_setup_comparator(uint8_t neg_to_d5);

static void d1_device_delay_us_test(uint16_t interval_us);

// D2
static inline void d2_reset_timer3() {timer3_reset();};
static void d2_setup_timer3_input_capture();
static void d2_write_timer3_input_capture_interval();

// D3
static inline void d3_reset_timer3() {timer3_reset();};
static void d3_setup_timer3_trigger_output(uint16_t interval_ms);
static void d3_setup_timer3_pwm(uint8_t freq_prescaler, uint16_t top, uint16_t duty);
static void d3_setup_timer3_sound(uint16_t freq_hz, uint8_t volume);

// D4
static void d4_reset_pin_change();
static void d4_setup_pin_change();

// D5
static inline void d5_reset_adc() {adc_reset();};

/* ===========================
 * RAVA8 PERIPHERALS
 * =========================== */

/*
Configures the digital port in output mode.
*/
void periph_mode_output(uint8_t port_id)
{
  if (periph_modes[port_id] != PERIPH_OUTPUT) {
    *periphs[port_id]->port &= ~periphs[port_id]->_port_i; // Low state
    *periphs[port_id]->ddr |= periphs[port_id]->_port_i; // Output mode
    periph_modes[port_id] = PERIPH_OUTPUT;
  }
}

/*
Configures the digital port in input mode.
*/
void periph_mode_input(uint8_t port_id)
{
  if (periph_modes[port_id] != PERIPH_INPUT) {
    *periphs[port_id]->port &= ~periphs[port_id]->_port_i; // Low state (Disable pull-up)
    *periphs[port_id]->ddr &= ~periphs[port_id]->_port_i; // Input mode
    periph_modes[port_id] = PERIPH_INPUT;
  }
}

/*
Drives the digital port to a LOW logic level.
*/
void periph_write_lo(uint8_t port_id)
{
  periph_mode_output(port_id);
  *periphs[port_id]->port &= ~periphs[port_id]->_port_i;
}

/*
Drives the digital port to a HIGH logic level.
*/
void periph_write_hi(uint8_t port_id)
{
  periph_mode_output(port_id);
  *periphs[port_id]->port |= periphs[port_id]->_port_i;
}

/*
Drives the digital port to a HIGH logic level for the specified duration in microseconds.
*/
void periph_write_pulse(uint8_t port_id, uint16_t duration_us)
{
  periph_write_hi(port_id);
  device_delay_us(duration_us);
  periph_write_lo(port_id);
}

/*
Measures the current logic level of the digital port.
*/
bool periph_read(uint8_t port_id)
{
  periph_mode_input(port_id);
  return (*periphs[port_id]->pin & periphs[port_id]->_port_i) != 0;
}

/* ===========================
 * D1
 * =========================== */

/*
Disables the D1 external trigger input functionality.
*/
static void d1_reset_trigger_input()
{
  EICRB = 0;
  EIMSK = 0; // Disable interrupt
}

/*
Enables the D1 external trigger input functionality.

Rising-edge transitions on D1 trigger the ISR (INT6_vect) interrupt service routine.
*/
static void d1_setup_trigger_input()
{
  periph_mode_input(1);
  d1_reset_trigger_input();

  // Configure INT6 to trigger on a rising edge at D1
  EICRB = _BV(ISC61) | _BV(ISC60);
  // Enable the INT6 external interrupt
  EIMSK |= _BV(INT6);
}

/*
Configures the analog comparator functionality using D1 as the positive input.

If neg_to_d5 == true, D5 is configured as the negative comparator input. Otherwise, the internal
bandgap reference is used as the negative input.

Whenever the positive input voltage exceeds the negative input voltage, the ISR (ANALOG_COMP_vect)
interrupt service routine is triggered.
*/
static void d1_setup_comparator(uint8_t neg_to_d5)
{
  adc_comp_setup(neg_to_d5);
}

/*
Generates a HIGH pulse with the specified duration in microseconds.

This function is used to validate the timing accuracy of the device_delay_us() delay function,
which implements the RNG sampling interval. The generated pulse can be measured with external
instrumentation, such as an oscilloscope.
*/
static void d1_device_delay_us_test(uint16_t interval_us)
{
  cli();
  d1_write_hi_fast();
  device_delay_us(interval_us);
  d1_write_lo_fast();
  sei();
}

/* ===========================
 * D2
 * =========================== */

/*
Enables measurement of the time interval between digital pulses on port D2.

Each rising-edge pulse detected on D2 triggers the ISR(TIMER3_CAPT_vect) interrupt service routine,
which subsequently calls d2_write_timer3_input_capture_interval().

Timer3 overflow is configured to occur every 1s, triggereing ISR (TIMER3_OVF_vect), which
increments timer3_overflow_n to extend the measurable interval range between pulses.
*/
static void d2_setup_timer3_input_capture()
{
  periph_mode_input(2);
  timer3_setup_input_capture_pc7();
}

/*
Callback function triggered by ISR(TIMER3_CAPT_vect).

Sends the time interval, in seconds, between consecutive digital pulses detected on port D2.
*/
static void d2_write_timer3_input_capture_interval()
{
  // IO Structure
  typedef struct {float interval_s;} data_out_t;
  data_out_t data_out;

  // Measure counts
  data_out.interval_s = timer3_stop_chronometer();

  // Inject original req_id, and comm_id in comm
  input_capture_cfg.comm->msg.req_id = input_capture_cfg.req_id;
  input_capture_cfg.comm->msg.comm_id = COMM_PERIPH_D2_TIMER3_INPUT_CAPTURE;

  // Send Header
  send_rava_msg_header(input_capture_cfg.comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/* ===========================
 * D3
 * =========================== */

/*
Generates a periodic pulse signal on D3 with an approximately 100 µs HIGH duration and the
specified period in milliseconds.
*/
static void d3_setup_timer3_trigger_output(uint16_t interval_ms)
{
  timer3_setup_trigger_output_pc6(interval_ms);
}

/*
Generates a PWM signal on D3 using the specified parameters.
*/
static void d3_setup_timer3_pwm(uint8_t freq_prescaler, uint16_t top, uint16_t duty)
{
  timer3_setup_pwm_pc6(freq_prescaler, top, duty);
}

/*
Generates an audio-frequency signal on D3 using the specified parameters.
*/
static void d3_setup_timer3_sound(uint16_t freq_hz, uint8_t volume)
{
  float pwm_top = round(2000000. / freq_hz) - 1; // top = (16MHz / 8) / freq_hz - 1
  float volume_dutyc = round((float)volume / 255 * TIMER13_SOUND_VOLUME_DUTYC_MAX);
  timer3_setup_pwm_pc6(TIMER013_CLK_DIV_8, (uint16_t)pwm_top, (uint16_t)volume_dutyc);
}

/* ===========================
 * D4
 * =========================== */

/*
Disables the D4 pin change interrupt functionality.
*/
static void d4_reset_pin_change()
{
  PCICR = 0;
  PCMSK0 = 0;
}

/*
Enables the D4 pin change interrupt functionality.

Any logic-level transition on D4 triggers the ISR(PCINT0_vect) interrupt service routine.
*/
static void d4_setup_pin_change()
{
  periph_mode_input(4);
  d4_reset_pin_change();

  // Pin Change Interrupt Enable
  PCICR = _BV(PCIE0);

  // Pin Change Enable Mask; Enable PCINT7
  PCMSK0 |= _BV(PCINT7);
}

/* ===========================
 * COMM
 * =========================== */

/*
Processes requests for generic digital operations on ports Di.

Supported operations include:
 - Configuring input/output mode
 - Reading the digital logic level
 - Writing HIGH/LOW logic levels
 - Generating digital pulses

This request ensures that the appropriate input/output mode is configured before executing read or
write operations.
*/
void comm_periph_digi(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t periph_id, digi_comm, digi_comm_par; uint16_t pulse_duration_us;} data_in_t;
  typedef struct {uint8_t digi_state;} data_out_t;
  data_in_t  data_in;
  data_out_t data_out = {0};

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  if (!(data_in.periph_id > 0 &&
      data_in.periph_id <= PERIPHERALS_N &&
      data_in.digi_comm < PERIPH_COMM_ENUM_LAST)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  if (data_in.digi_comm == PERIPH_PULSE &&
      data_in.pulse_duration_us == 0) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }
  // Process Output

  // MODE
  if (data_in.digi_comm == PERIPH_MODE) {
    if (data_in.digi_comm_par == PERIPH_OUTPUT) {
      periph_mode_output(data_in.periph_id);
    }
    else {
      periph_mode_input(data_in.periph_id);
    }
  }

  // READ
  else if (data_in.digi_comm == PERIPH_READ) {
    data_out.digi_state = periph_read(data_in.periph_id);
  }

  // WRITE
  else if (data_in.digi_comm == PERIPH_WRITE) {
    if (data_in.digi_comm_par) {
      periph_write_hi(data_in.periph_id);
    }
    else {
      periph_write_lo(data_in.periph_id);
    }
  }

  // PULSE
  else if (data_in.digi_comm == PERIPH_PULSE) {
    periph_write_pulse(data_in.periph_id, data_in.pulse_duration_us);
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/*
Processes requests for fast digital operations on port D1.

Supported operations include:
 - Configuring input/output mode
 - Reading the digital logic level
 - Writing HIGH/LOW logic levels
 - Generating digital pulses

This request assumes the input/output mode has been properly configured before executing read or
write operations.
*/
void comm_periph_d1_digi_fast(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t digi_comm, digi_comm_par; uint16_t pulse_duration_us;} data_in_t;
  typedef struct {uint8_t digi_state;} data_out_t;
  data_in_t  data_in;
  data_out_t data_out = {0};

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  if (!(data_in.digi_comm < PERIPH_COMM_ENUM_LAST)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  if (data_in.digi_comm == PERIPH_PULSE &&
      data_in.pulse_duration_us == 0) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Process Output

  // MODE
  if (data_in.digi_comm == PERIPH_MODE) {
    if (data_in.digi_comm_par == PERIPH_OUTPUT) {
      periph_mode_output(1);
    }
    else {
      periph_mode_input(1);
    }
  }

  // READ
  else if (data_in.digi_comm == PERIPH_READ) {
    data_out.digi_state = d1_read_fast();
  }

  // WRITE
  else if (data_in.digi_comm == PERIPH_WRITE) {
    if (data_in.digi_comm_par) {
      d1_write_hi_fast();
    }
    else {
      d1_write_lo_fast();
    }
  }

  // PULSE
  else if (data_in.digi_comm == PERIPH_PULSE) {
    d1_write_pulse_fast(data_in.pulse_duration_us);
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}

/*
Processes the request to enable/disable the D1 external trigger input functionality.
*/
void comm_periph_d1_trigger_input(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t on;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  // Process Output
  if (data_in.on) {
    d1_setup_trigger_input();
  }
  else {
    d1_reset_trigger_input();
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to enable/disable the D1 comparator functionality.
*/
void comm_periph_d1_comparator(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t on, neg_to_d5;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  // Process Output
  if (data_in.on) {
    d1_setup_comparator(data_in.neg_to_d5);
  }
  else {
    d1_reset_comparator();
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to test the device_delay_us() delay function;
*/
void comm_periph_d1_device_delay_us_test(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint16_t interval_us;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  if (!(data_in.interval_us > 0)) {

      send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
      return;
    }

  // Process Output
  periph_mode_output(1);
  d1_device_delay_us_test(data_in.interval_us);

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to enable/disable the D2 input capture functionality.

Currently disabled, as it generates asynchronous messages that must be properly handled by the host.
*/
void comm_periph_d2_timer3_input_capture(comm_interface_t *const comm)
{
  /*
  // IO Structure
  typedef struct {uint8_t on;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  // Process Output
  if (data_in.on) {
    input_capture_cfg.comm = comm;
    input_capture_cfg.req_id = comm->msg.req_id;
    d2_setup_timer3_input_capture();
  }
  else {
    d2_reset_timer3();
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
  */
}

/*
Processes the request to enable/disable the D3 pediodic trigger functionality.
*/
void comm_periph_d3_timer3_periodic_trigger_output(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t on; uint16_t interval_ms;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  if (!(data_in.interval_ms > 0 && data_in.interval_ms <= TIMER3_MAXIMUM_DELAY_MS)) {

    send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
    return;
  }

  // Process Output
  if (rng_byte_stream_cfg.streaming) {
    send_rava_msg_header(comm, CE_TIMER3_BUSY, 0, 0, NULL);
    return;
  }

  if (data_in.on) {
    d3_setup_timer3_trigger_output(data_in.interval_ms);
  }
  else {
    d3_reset_timer3();
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to enable/disable the D3 PWM functionality.
*/
void comm_periph_d3_timer3_pwm(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t on, freq_prescaler; uint16_t top, duty;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  if (!(data_in.freq_prescaler < TIMER013_CLK_ENUM_LAST)) {

      send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
      return;
    }

  // Process Output
  if (rng_byte_stream_cfg.streaming) {
    send_rava_msg_header(comm, CE_TIMER3_BUSY, 0, 0, NULL);
    return;
  }

  if (data_in.on) {
    d3_setup_timer3_pwm(data_in.freq_prescaler, data_in.top, data_in.duty);
  }
  else {
    d3_reset_timer3();
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to enable/disable the D3 sound functionality.
*/
void comm_periph_d3_timer3_sound(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t on, volume; uint16_t freq_hz;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  // Process Output
  if (rng_byte_stream_cfg.streaming) {
    send_rava_msg_header(comm, CE_TIMER3_BUSY, 0, 0, NULL);
    return;
  }

  if (data_in.on && data_in.volume > 0 && data_in.freq_hz > 0) {
    d3_setup_timer3_sound(data_in.freq_hz, data_in.volume);
  }
  else {
    d3_reset_timer3();
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to enable/disable the D4 pin change functionality.
*/
void comm_periph_d4_pin_change(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t on;} data_in_t;
  //typedef struct {} data_out_t;
  data_in_t  data_in;
  //data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  // Process Output
  if (data_in.on) {
    d4_setup_pin_change();
  }
  else {
    d4_reset_pin_change();
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request to perform an ADC measurement on D5.
*/
void comm_periph_d5_adc(comm_interface_t *const comm)
{
  // IO Structure
  typedef struct {uint8_t on, ref_5v, clk_prescaler, oversampling_n_bits;} data_in_t;
  typedef struct {float adc_measure;} data_out_t;
  data_in_t  data_in;
  data_out_t data_out;

  // Input Deserialization
  if (comm->msg.data_len != sizeof(data_in)) {
    send_rava_msg_header(comm, CE_INVALID_INPUT_TYPES, 0, 0, NULL);
    return;
    }
  memcpy(&data_in, comm->msg.data, sizeof(data_in));

  // Process Input
  if (!(data_in.clk_prescaler > 0 &&
        data_in.clk_prescaler < ADC_CLK_ENUM_LAST &&
        data_in.oversampling_n_bits <= 6)) {

      send_rava_msg_header(comm, CE_INVALID_INPUT_VALUES, 0, 0, NULL);
      return;
    }

  // Process Output
  if (data_in.on) {
    data_out.adc_measure = adc_read_ch12_volts(data_in.ref_5v, data_in.clk_prescaler, data_in.oversampling_n_bits);
  }
  else {
    d5_reset_adc();
    data_out.adc_measure = 0.;
  }

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);
}