/**
 * Copyright (c) 2023 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
This is the main file that orchestrates the RAVA's firmware functioning. Its
core functions are:

1. Setup: At the outset, the code initializes instances of various components in
accordance with the preprocessor variables specified in rava_config.h. Each
component class proceeds through its initialization routines to configure the
microcontroller (MCU) for its intended functionality. Some classes retrieve
their configuration parameters from the device's EEPROM memory.

2. Health Startup Tests: Following initialization, startup tests are conducted
to assess the randomness quality. In the event of a test failure, the code
enters a loop, invoking task_health_startup_failed() to report the problem.
During this state, entropy generation is suspended until the health-related
issues are resolved.

3. Command Processing: Assuming the startup tests succeed, the code enters a
loop within task_serial_read(). This function monitors the serial interface for
incoming commands from a leader device. Each received command initiates a
specific action, which may or may not involve sending information back.

Command identification codes are defined in rava_comm.h, and categorized into
distinct groups, each associated with a specific source file and header. These
categories encompass device, eeprom, pwm, rng, health, led, lamp, peripherals,
and interfaces.

The firmware code includes a documentation section at the beginning of each
header file, elucidating the module's purpose. To gain a deeper understanding of
how the device operates, users are encouraged to explore the code and these
explanations by following the sequence of module initializations within the
setup() function.
*/

#include <Arduino.h>

#include <rava_adc.h>
#include <rava_comm.h>
#include <rava_config.h>
#include <rava_device.h>
#include <rava_eeprom.h>
#include <rava_health.h>
#include <rava_interfaces.h>
#include <rava_interrupts.h>
#include <rava_lamp.h>
#include <rava_led.h>
#include <rava_peripherals.h>
#include <rava_pwm.h>
#include <rava_rng.h>
#include <rava_timers.h>
#include <rava_tools.h>

/////////////////////////////
// VARIABLES

# define STARTUP_DELAY_STABILIZE_US 50000 // 50ms

// MCU COMM
COMM* comm;
COMM_USB* usb;
COMM_SERIAL* serial;

// MCU BASE COMPONENTS
DEVICE* dev;
EEPROM* eeprom;
TIMER0* timer0;
TIMER1* timer1;
TIMER3* timer3;
TIMER4* timer4;
WDT* wdt;
ADC_COMP* adc_comp;

// RAVA COMPONENTS
PWM* pwm;
RNG* rng;
LED* led;
LAMP* lamp;

// RAVA HEALTH
HEALTH_STARTUP* rng_health_startup;
HEALTH_CONTINUOUS* rng_health_continuous;

// RAVA PERIPHERALS
D1* d1;
D2* d2;
D3* d3;
D4* d4;
D5* d5;
PERIPH* ds[5];

// RAVA INTERFACES
#if defined (FIRMWARE_INTERFACE_DS18B20_ENABLED)
INTERF_DS18B20* interf_ds18b20;
#endif

/////////////////////////////
// SETUP

void setup()
{
  // MCU COMM
  usb = new COMM_USB();

  #if defined(FIRMWARE_COMM_SERIAL1_ENABLED)
  serial = new COMM_SERIAL();
  #endif

  // MCU BASE COMPONENTS
  dev = new DEVICE();
  eeprom = new EEPROM();
  timer0 = new TIMER0();
  timer1 = new TIMER1();
  timer3 = new TIMER3();
  timer4 = new TIMER4();
  wdt = new WDT();
  adc_comp = new ADC_COMP();

  // RAVA COMPONENTS
  pwm = new PWM();
  rng = new RNG();

  #if defined(FIRMWARE_LED_ENABLED)
  led = new LED();
  #endif

  #if defined(FIRMWARE_LED_ENABLED) && defined(FIRMWARE_LAMP_ENABLED)
  lamp = new LAMP();
  #endif

  // RAVA HEALTH
  #if defined(FIRMWARE_HEALTH_STARTUP_ENABLED)
  rng_health_startup = new HEALTH_STARTUP();
  #endif

  #if defined(FIRMWARE_HEALTH_CONTINUOUS_ENABLED)
  rng_health_continuous = new HEALTH_CONTINUOUS();
  #endif

  // RAVA PERIPHERALS
  #if defined(FIRMWARE_PERIPHERALS_ENABLED)
  d1 = new D1();
  d2 = new D2();
  d3 = new D3();
  d4 = new D4();
  d5 = new D5();

  ds[0] = (PERIPH*) d1;
  ds[1] = (PERIPH*) d2;
  ds[2] = (PERIPH*) d3;
  ds[3] = (PERIPH*) d4;
  ds[4] = (PERIPH*) d5;
  #endif

  // RAVA INTERFACES
  #if defined(FIRMWARE_INTERFACE_DS18B20_ENABLED)
  interf_ds18b20 = new INTERF_DS18B20(DS18B20_PIN, DS18B20_RESOLUTION);
  #endif

  // Sleep for a while to let the circuit stabilize
  delayMicroseconds(STARTUP_DELAY_STABILIZE_US);

  // Health startup test
  #if defined(FIRMWARE_HEALTH_STARTUP_ENABLED)
  rng_health_startup->run_tests();

  if (!rng_health_startup->get_tests_result())
    // Error: Loop on task_health_startup_failed()
    return;
  #endif

  // Reset Health continuous errors
  #if defined(FIRMWARE_HEALTH_CONTINUOUS_ENABLED)
  rng_health_continuous->reset_errors();
  #endif

  // Enable LAMP mode
  #if defined(FIRMWARE_LED_ENABLED) && defined(FIRMWARE_LAMP_ENABLED)
  lamp->setup();
  #endif
}

/////////////////////////////
// TASKS

void task_health_startup_failed()
{
  uint8_t msg_bytes[COMM_MSG_LEN-1];

  // Is there any available command?
  uint8_t msg_command_id = usb->read_msg_header(msg_bytes);

  if (msg_command_id) {
    // Yes, specify the communication device to which the modules will refer
    comm = (COMM*) usb;
  }
  else {
    // No available command, exit
    return;
  }

  // Take an action according to the command id

  // DEVICE_SERIAL_NUMBER
  if (msg_command_id == COMM_DEVICE_SERIAL_NUMBER)
    dev->send_serial_number();

  // EEPROM_FIRMWARE
  else if (msg_command_id == COMM_EEPROM_FIRMWARE)
    eeprom->send_firmware();

  // HEALTH_STARTUP_RUN
  else if (msg_command_id == COMM_HEALTH_STARTUP_RUN)
    rng_health_startup->run_tests();

  // HEALTH_STARTUP_RESULTS
  else if (msg_command_id == COMM_HEALTH_STARTUP_RESULTS)
    rng_health_startup->send_results();
}

void task_serial_read(COMM* comm_task)
{
  uint8_t msg_bytes[COMM_MSG_LEN-1];

  // Is there any available command?
  uint8_t msg_command_id = comm_task->read_msg_header(msg_bytes);

  if (msg_command_id) {

    // Yes, specify the communication device to which the modules will refer
    comm = (COMM*) comm_task;

    // Disable LAMP on first comm contact
    #if defined(FIRMWARE_LED_ENABLED) && defined(FIRMWARE_LAMP_ENABLED)

    // Is LAMP sending debugging information?
    if (!lamp->experiment_debugging()) {

      // Switching from uncontacted to contacted?
      if (!comm_task->get_contacted()) {

        // Switch LAMP off: see task_lamp()
        comm_task->set_contacted(true);

        // LED fade out
        led->fade_intensity(0, 1000);
      }
    }
    #endif
  }
  else {
    // No available command, exit
    return;
  }

  // Take an action according to the command id
  switch (msg_command_id)
  {
    // DEVICE
    case COMM_DEVICE_SERIAL_NUMBER: {
      dev->send_serial_number();
      break;
      }

    case COMM_DEVICE_TEMPERATURE: {
      dev->send_temperature();
      break;
      }

    case COMM_DEVICE_FREE_RAM: {
      dev->send_free_ram();
      break;
      }

    case COMM_DEVICE_REBOOT: {
      wdt->reboot_device();
      break;
      }

    // EEPROM
    case COMM_EEPROM_RESET_TO_DEFAULT: {
      eeprom->erase();
      eeprom->update_default();

      // Force PWM and RNG to load EEPROM parameters
      pwm->setup(true);
      rng->setup(true);
      break;
    }

    case COMM_EEPROM_DEVICE: {
      uint8_t send = msg_bytes[1];
      uint16_t slope = unpack_int(msg_bytes[2], msg_bytes[3]);
      int16_t intercept = (int16_t)unpack_int(msg_bytes[4], msg_bytes[5]);

      if (send)
        eeprom->send_device();
      else
        eeprom->update_device(slope, intercept);
      break;
    }

    case COMM_EEPROM_FIRMWARE: {
      eeprom->send_firmware();
      break;
    }

    case COMM_EEPROM_PWM: {
      uint8_t send = msg_bytes[1];
      uint8_t pwm_freq_id = msg_bytes[2];
      uint8_t pwm_duty = msg_bytes[3];

      if (send) {
        eeprom->send_pwm();
      }
      else {
        eeprom->update_pwm(pwm_freq_id, pwm_duty);
        pwm->setup(false, pwm_freq_id, pwm_duty);
      }
      break;
    }

    case COMM_EEPROM_RNG: {
      uint8_t send = msg_bytes[1];
      uint8_t sampling_interval = msg_bytes[2];

      if (send) {
        eeprom->send_rng();
      }
      else {
        eeprom->update_rng(sampling_interval);
        rng->setup(false, sampling_interval);
      }
      break;
    }

    case COMM_EEPROM_LED: {
      uint8_t send = msg_bytes[1];
      uint8_t led_attached = msg_bytes[2];

      if (send)
        eeprom->send_led();
      else
        eeprom->update_led(led_attached);
      break;
    }

    case COMM_EEPROM_LAMP: {
      uint8_t send = msg_bytes[1];

      if (send)
        eeprom->send_lamp();
      else {
        uint8_t exp_mag_smooth_n_trials = msg_bytes[2];
        uint8_t extra_n_bytes = msg_bytes[3];
        uint8_t extra_bytes[extra_n_bytes];

        comm_task->read(extra_bytes, extra_n_bytes);
        uint32_t exp_dur_max_ms = unpack_long(extra_bytes[0], extra_bytes[1], extra_bytes[2], extra_bytes[3]);
        float exp_z_significant = unpack_float(extra_bytes[4], extra_bytes[5], extra_bytes[6], extra_bytes[7]);

        eeprom->update_lamp(exp_dur_max_ms, exp_z_significant, exp_mag_smooth_n_trials);
      }
      break;
    }

    // PWM
    case COMM_PWM_SETUP: {
      uint8_t send = msg_bytes[1];
      uint8_t freq_id = msg_bytes[2];
      uint8_t duty = msg_bytes[3];

      if (send)
        pwm->send_setup();
      else
        pwm->setup(false, freq_id, duty);
      break;
    }

    // RNG
    case COMM_RNG_SETUP: {
      uint8_t send = msg_bytes[1];
      uint8_t sampling_interval = msg_bytes[2];

      if (send)
        rng->send_setup();
      else
        rng->setup(false, sampling_interval);
      break;
    }

    case COMM_RNG_PULSE_COUNTS: {
      uint32_t n_counts = unpack_long(msg_bytes[1], msg_bytes[2], msg_bytes[3], msg_bytes[4]);

      rng->send_pulse_counts(n_counts);
      break;
    }

    case COMM_RNG_BITS: {
      uint8_t bit_source = msg_bytes[1];

      rng->send_bits(bit_source);
      break;
    }

    case COMM_RNG_BYTES: {
      uint32_t n_bytes = unpack_long(msg_bytes[1], msg_bytes[2], msg_bytes[3], msg_bytes[4]);
      uint8_t postproc_id = msg_bytes[5];
      uint8_t request_id = msg_bytes[6];

      rng->send_bytes(n_bytes, postproc_id, COMM_RNG_BYTES, request_id);
      break;
    }

    case COMM_RNG_TIMING_DEBUG_D1: {
      uint8_t on = msg_bytes[1];

      rng->setup_timing_debug_d1(on);
      break;
    }

    case COMM_RNG_INT8S: {
      uint32_t n_ints = unpack_long(msg_bytes[1], msg_bytes[2], msg_bytes[3], msg_bytes[4]);
      uint8_t int_delta = msg_bytes[5];

      rng->send_int8s(n_ints, int_delta);
      break;
    }

    case COMM_RNG_INT16S: {
      uint32_t n_ints = unpack_long(msg_bytes[1], msg_bytes[2], msg_bytes[3], msg_bytes[4]);
      uint16_t int_delta = unpack_int(msg_bytes[5], msg_bytes[6]);

      rng->send_int16s(n_ints, int_delta);
      break;
    }

    case COMM_RNG_FLOATS: {
      uint32_t n_floats = unpack_long(msg_bytes[1], msg_bytes[2], msg_bytes[3], msg_bytes[4]);

      rng->send_floats(n_floats);
      break;
    }

    case COMM_RNG_STREAM_START: {
      uint16_t n_bytes = unpack_int(msg_bytes[1], msg_bytes[2]);
      uint8_t postproc_id = msg_bytes[3];
      uint16_t stream_interval_ms = unpack_int(msg_bytes[4], msg_bytes[5]);

      rng->start_bytes_stream(n_bytes, postproc_id, stream_interval_ms);
      break;
    }

    case COMM_RNG_STREAM_STOP: {
      rng->stop_bytes_stream();
      break;
    }

    case COMM_RNG_STREAM_STATUS: {
      rng->send_bytes_stream_status();
      break;
    }

    // HEALTH
    #if defined(FIRMWARE_HEALTH_STARTUP_ENABLED)

    case COMM_HEALTH_STARTUP_RUN: {
      rng_health_startup->run_tests();
      break;
    }

    case COMM_HEALTH_STARTUP_RESULTS: {
      rng_health_startup->send_results();
      break;
    }
    #endif

    #if defined(FIRMWARE_HEALTH_CONTINUOUS_ENABLED)

    case COMM_HEALTH_CONTINUOUS_ERRORS: {
      rng_health_continuous->send_errors();
      break;
    }
    #endif

    // LED
    #if defined(FIRMWARE_LED_ENABLED)

    case COMM_LED_COLOR: {
      uint8_t hue = msg_bytes[1];
      uint8_t intensity = msg_bytes[2];

      led->set_color(hue, intensity);
      break;
    }

    case COMM_LED_COLOR_FADE: {
      uint8_t hue_target = msg_bytes[1];
      uint32_t duration_ms = unpack_long(msg_bytes[2], msg_bytes[3], msg_bytes[4], msg_bytes[5]);

      led->fade_color(hue_target, duration_ms);
      break;
    }

    case COMM_LED_COLOR_OSCILLATE: {
      uint8_t n_cycles = msg_bytes[1];
      uint32_t duration_ms = unpack_long(msg_bytes[2], msg_bytes[3], msg_bytes[4], msg_bytes[5]);

      led->fade_color_oscillate(n_cycles, duration_ms);
      break;
    }

    case COMM_LED_INTENSITY: {
      uint8_t intensity = msg_bytes[1];

      led->set_intensity(intensity);
      break;
    }

    case COMM_LED_INTENSITY_FADE: {
      uint8_t intensity_target = msg_bytes[1];
      uint16_t duration_ms = unpack_int(msg_bytes[2], msg_bytes[3]);

      led->fade_intensity(intensity_target, duration_ms);
      break;
    }

    case COMM_LED_FADE_STOP: {
      led->fade_stop();
      break;
    }

    case COMM_LED_STATUS: {
      led->send_status();
      break;
    }
    #endif

    // LAMP
    #if defined(FIRMWARE_LED_ENABLED) && defined(FIRMWARE_LAMP_ENABLED)

    case COMM_LAMP_MODE: {
      uint8_t lamp_on = msg_bytes[1];
      if (lamp_on) {
        comm_task->set_contacted(false);
        lamp->setup();
      }
      else {
        comm_task->set_contacted(true);
      }
      break;
    }

    case COMM_LAMP_STATISTICS: {
      lamp->send_statistics();
      break;
    }

    case COMM_LAMP_DEBUG: {
      uint8_t on = msg_bytes[1];

      lamp->experiment_debug(on);
      break;
    }
    #endif

    // PERIPHERALS
    #if defined(FIRMWARE_PERIPHERALS_ENABLED)

    case COMM_PERIPH_MODE: {
      uint8_t periph_id = msg_bytes[1];
      uint8_t mode = msg_bytes[2];
      if ((periph_id == 0) || (periph_id > 5))
        break;

      if (mode == PERIPH_INPUT)
        (ds[periph_id-1])->mode_input();
      else if (mode == PERIPH_OUTPUT)
        (ds[periph_id-1])->mode_output();
      break;
    }

    case COMM_PERIPH_READ: {
      uint8_t periph_id = msg_bytes[1];
      if ((periph_id == 0) || (periph_id > 5))
        break;

      ds[periph_id-1]->send_digi_state();
      break;
    }

    case COMM_PERIPH_WRITE: {
      uint8_t periph_id = msg_bytes[1];
      uint8_t digi_state = msg_bytes[2];
      if ((periph_id == 0) || (periph_id > 5))
        break;

      if (digi_state)
        ds[periph_id-1]->write_hi();
      else
        ds[periph_id-1]->write_lo();
      break;
    }

    case COMM_PERIPH_PULSE: {
      uint8_t periph_id = msg_bytes[1];
      uint16_t pulse_duration_us = unpack_int(msg_bytes[2], msg_bytes[3]);
      if ((periph_id == 0) || (periph_id > 5))
        break;

      ds[periph_id-1]->write_pulse(pulse_duration_us);
      break;
    }

    case COMM_PERIPH_D1_TRIGGER_INPUT: {
      uint8_t on = msg_bytes[1];

      if (on)
        d1->setup_trigger_input();
      else
        d1->reset_trigger_input();
      break;
    }

    case COMM_PERIPH_D1_COMPARATOR: {
      uint8_t on = msg_bytes[1];
      uint8_t neg_to_d5 = msg_bytes[2];

      if (on)
        d1->setup_comparator(neg_to_d5);
      else
        d1->reset_comparator();
      break;
    }

    case COMM_PERIPH_D1_DELAY_US_TEST: {
      uint8_t delay_us = msg_bytes[1];

      d1->mode_output();
      d1->delay_us_test(delay_us);
      break;
    }

    case COMM_PERIPH_D2_TIMER3_INPUT_CAPTURE: {
      uint8_t on = msg_bytes[1];

      if (on)
        d2->setup_timer3_input_capture();
      else
        d2->reset_timer3_input_capture();
      break;
    }

    case COMM_PERIPH_D3_TIMER3_TRIGGER_OUTPUT: {
      uint8_t on = msg_bytes[1];
      uint16_t interval_ms = unpack_int(msg_bytes[2], msg_bytes[3]);

      if (on)
        d3->setup_timer3_trigger_output(interval_ms);
      else
        d3->reset_timer3_trigger_output();
      break;
    }

    case COMM_PERIPH_D3_TIMER3_PWM: {
      uint8_t on = msg_bytes[1];
      uint8_t freq_prescaler = msg_bytes[2];
      uint16_t top = unpack_int(msg_bytes[3], msg_bytes[4]);
      uint16_t duty = unpack_int(msg_bytes[5], msg_bytes[6]);

      if (on)
        d3->setup_timer3_pwm(freq_prescaler, top, duty);
      else
        d3->reset_timer3_pwm();
      break;
    }

    case COMM_PERIPH_D4_PIN_CHANGE: {
      uint8_t on = msg_bytes[1];

      if (on)
        d4->setup_pin_change();
      else
        d4->reset_pin_change();
      break;
    }

    case COMM_PERIPH_D5_ADC: {
      uint8_t on = msg_bytes[1];
      uint8_t ref_5v = msg_bytes[2];
      uint8_t clk_prescaler = msg_bytes[3];
      uint8_t oversampling_n_bits = msg_bytes[4];

      if (on)
        d5->send_adc_reading(ref_5v, clk_prescaler, oversampling_n_bits);
      else
        d5->reset_adc();
      break;
    }
    #endif

    // INTERFACES
    #if defined(FIRMWARE_INTERFACE_DS18B20_ENABLED)

    case COMM_INTERFACE_DS18B20: {
      interf_ds18b20->send_read();
      break;
    }
    #endif

  }
}

// Streaming mode: Send data when triggered
void task_rng_stream()
{
  if ((rng->stream_cfg.streaming) && (rng->stream_cfg.triggered)){
    rng->send_bytes_stream();
  }
}

// Processes LED color and intensity fadings
void task_led_fade()
{
  led->fade_process();
}

// Processes lamp mode, when no serial connection was made to the device
void task_lamp()
{
  bool comm_contacted = usb->get_contacted();

  #if defined(FIRMWARE_COMM_SERIAL1_ENABLED)
  comm_contacted |= serial->get_contacted();
  #endif

  // Stay in LAMP mode while the device is uncontacted
  if (!comm_contacted) {
    lamp->process();
  }
}

/////////////////////////////
// LOOP

void loop()
{
  // Startup tests failed
  #if defined(FIRMWARE_HEALTH_STARTUP_ENABLED)
  if (!rng_health_startup->get_tests_result()) {
    task_health_startup_failed();
    return;
  }
  #endif

  // Startup tests successful

  // Read USB interface and act accordingly
  task_serial_read(usb);

  // Read Serial1 interface and act accordingly
  #if defined(FIRMWARE_COMM_SERIAL1_ENABLED)
  task_serial_read(serial);
  #endif

  // RNG Streaming?
  task_rng_stream();

  // LED fading?
  #if defined(FIRMWARE_LED_ENABLED)
  task_led_fade();
  #endif

  // LAMP enabled?
  #if defined(FIRMWARE_LED_ENABLED) && defined(FIRMWARE_LAMP_ENABLED)
  task_lamp();
  #endif
}