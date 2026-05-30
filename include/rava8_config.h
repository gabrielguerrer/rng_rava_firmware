/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Defines the compile-time feature configuration of the RAVA8_RNG library.

Features can be enabled or disabled through preprocessor macros to control which functionalities
are included in the final firmware build.
*/

#ifndef RAVA8_CONFIG_H
#define RAVA8_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================
 * RAVA8
 * =========================== */

#define HEALTH_STARTUP_ENABLED    // Enable Health startup code
#define HEALTH_CONTINUOUS_ENABLED // Enable Health continuous code
// #define COMM_USART_ENABLED        // Enable USART interface
// #define PERIPHERALS_ENABLED       // Enable Peripherals functionality
// #define RNG_TIMING_DEBUG_ENABLED  // Enable monitoring byte generation timing

#ifdef __cplusplus
}
#endif

#endif