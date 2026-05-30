/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
This is the main file which orchestrates the ATmega32U4 RAVA8 Firmware.
*/

#include <stdint.h>
#include <avr/power.h>

#include <rava_comm.h>
#include <rava_health_startup.h>

#include <rava8_comm_handlers.h>
#include <rava8_comm_usart.h>
#include <rava8_comm_usbcdc.h>
#include <rava8_comm_usbcdc_descriptors.h>
#include <rava8_device.h>
#include <rava8_eeprom.h>
#include <rava8_interrupts.h>
#include <rava8_peripherals.h>
#include <rava8_pwm_boost.h>
#include <rava8_rng.h>
#include <rava8_timers.h>

#include "rava8_config.h"

/* ===========================
 * RAVA8
 * =========================== */

static inline void task_health_startup_failed(comm_interface_t *const comm);
static inline void task_rava_read(comm_interface_t *const comm);
static inline void task_rng_stream(void);
static inline void loop_health_startup_failed(void);

/* ===========================
 * SETUP
 * =========================== */

/*
Initializes the communication interfaces, EEPROM support, then configures the PWM boost stage, RNG
sampling interval, and Timers 0 and 1, which are required for proper entropy generation.

After initialization, the firmware optionally performs the RNG startup health tests. If a test
fails, the initialization sequence is retried once. If the second attempt also fails, the firmware
enters an infinite error loop. It is recommended that the host verify whether the startup health
tests completed successfully when establishing communication with the device.
*/
void setup(void)
{
	// Initialize WDT
  wdt_init();

	// Disable clock division (typically already configured by fuses)
	clock_prescale_set(clock_div_1);

	// Disable unused peripherals
	power_spi_disable();
	power_twi_disable();
	power_usart1_disable();

	// Compute Serial Number
	device_calc_serial_number();

	// Configure the PLL to generate the 48 MHz clock required by USB and Timer4
	pll_setup_48mhz();

	// Initialize USB CDC
  usbcdc_init();

	// Enable interrupts
	sei();

  #if defined(COMM_USART_ENABLED)
	// Initialize USART
  usart_init();
  #endif

  // Initialize EEPROM
  eeprom_init();

  // Initialize PWM BOOST (Timer4)
  pwm_boost_init();

	// Initialize RNG (Timer0 and Timer1)
  rng_init();

	#ifdef HEALTH_STARTUP_ENABLED
  // Health startup test
  health_startup_run_tests();

	// Failed? Try again
	if (!health_startup_get_tests_result()) {
	health_startup_run_tests();
	}

  // Failed? Loop and process commands
  loop_health_startup_failed();

	// Reset byte counter
	device_gen_bytes_count = 0;
  #endif
}

/* ===========================
 * TASKS
 * =========================== */

/*
In the event of a startup health test failure, the firmware enters a loop that invokes this task
with limited functionality in order to report the error condition. The user may attempt to restore
the EEPROM to its default values and rerun the tests.

During this state, entropy generation remains suspended until the health-related issues are
resolved.
*/
static inline void task_health_startup_failed(comm_interface_t *const comm_if)
{
  // Is there any valid request?
  if (!read_rava_msg(comm_if)) {
    return;
  }

	// Valid message arrived and available in in comm->msg

  // Take an action according to the command id
	uint8_t comm_id = comm_if->msg.comm_id;

  if (comm_id == COMM_DEVICE_GET_INFO) {
    comm_device_get_info(comm_if);
  }
  else if (comm_id == COMM_EEPROM_RESET_TO_DEFAULT) {
    comm_eeprom_reset_to_default(comm_if);
  }
  else if (comm_id == COMM_HEALTH_STARTUP_RUN) {
    comm_health_startup_run(comm_if);
  }
  else if (comm_id == COMM_HEALTH_STARTUP_GET_RESULTS) {
    comm_health_startup_get_results(comm_if);
  }
	else {
		send_rava_msg_header(comm_if, CE_HEALTH_STARTUP_FAILED, 0, 0, NULL);
	}
}

/*
Monitors the communication interface for incoming messages from the host device. After checking for
potential errors, valid commands are dipatched to the appropriate handler functions.
*/
static inline void task_rava_read(comm_interface_t *const comm_if)
{
  // Is there any valid request?
  if (!read_rava_msg(comm_if)) {
		return;
	}

	// Valid message arrived and available in in comm->msg

	// Validate comm_id
	uint8_t comm_id = comm_if->msg.comm_id;
	if (comm_id >= RAVA_COMM_HANDLERS_COUNT + RAVA8_COMM_HANDLERS_COUNT) {
		send_rava_msg_header(comm_if, CE_INVALID_COMM_ID, 0, 0, NULL);
		return;
	}

	// Check if handler exists, and then execute it
	if (comm_id < RAVA_COMM_HANDLERS_COUNT && rava_comm_handlers[comm_id]) {
		rava_comm_handlers[comm_id](comm_if);
	}
	else if (comm_id >= RAVA_COMM_HANDLERS_COUNT && rava8_comm_handlers[comm_id - RAVA_COMM_HANDLERS_COUNT]) {
		rava8_comm_handlers[comm_id - RAVA_COMM_HANDLERS_COUNT](comm_if);
	}
	else {
		send_rava_msg_header(comm_if, CE_INVALID_COMM_ID, 0, 0, NULL);
	}
}

/*
Implements periodic random-byte streaming functionality.

Random bytes are generated and transmitted whenever `streaming` and `triggered` flags are set.
*/
static inline void task_rng_stream(void)
{
	// Streaming mode: Send data when triggered
  if (rng_byte_stream_cfg.streaming && rng_byte_stream_cfg.triggered) {
    rng_write_byte_stream();

		// Stream trigger
		if (rng_byte_stream_cfg.interval_ms == 0) {
			// Always on
			rng_byte_stream_cfg.triggered = true;
		}
		else {
			// Wait for next TIMER3_COMPA_vect interrupt
			rng_byte_stream_cfg.triggered = false;
		}
  }
}

/* ===========================
 * LOOP
 * =========================== */

/*
The firmware loops through these functions, searching for valid messages across the available
communication interfaces while also executing byte streaming and LUFA tasks.
*/
static inline void loop(void)
{
	for (;;) {

		// Read and parse USB interface
		task_rava_read(&comm_usbcdc_if);

		// Read and parse Serial1 interface
		#if defined(COMM_USART_ENABLED)
		task_rava_read(&comm_usart_if);
		#endif

		// RNG Streaming?
		task_rng_stream();

		// LUFA
		CDC_Device_USBTask(&lufa_usbcdc_if);
		USB_USBTask();
	}
}

/*
Loop executed following a startup health test failure.
*/
static inline void loop_health_startup_failed(void)
{
  while (!health_startup_get_tests_result()) {
		task_health_startup_failed(&comm_usbcdc_if);

		#ifdef COMM_USART_ENABLED
		task_health_startup_failed(&comm_usart_if);
		#endif

		// LUFA
		CDC_Device_USBTask(&lufa_usbcdc_if);
		USB_USBTask();
  }
}

/* ===========================
 * MAIN
 * =========================== */

/*
Performs initialization and executes the main loop.
*/
int main(void) {
  setup();
	loop();

  return 0;
}