/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The RAVA8 RNG module implements the core randomness-generation functionality of the RAVA
architecture.

Entropy is obtained from the differential comparator outputs V_CMP1 and V_CMP2, whose pulse streams
are counted through Timers 1 and 0, respectively. The pulse counts are then converted into random
bits according to parity: odd counts produce bit 1, while even counts produce bit 0. Randomness is
generated concurrently across both RNG channels.

Random byte generation is achieved by repeating the pulse-count acquisition and parity evaluation
process eight times, assembling the resulting bits into output bytes.

The RNG configuration is based on a sampling-interval parameter defining the measurement duration
for each pulse-count acquisition. During initialization, this parameter is loaded from EEPROM
memory, although it may also be modified during runtime without affecting the stored EEPROM
configuration.

After generation, output bytes may optionally be processed by the continuous health-monitoring
tests which monitors the quality of the generated entropy in real time.
*/

#ifndef RAVA8_RNG_H
#define RAVA8_RNG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rava_comm.h>

/* ===========================
 * RAVA8 RNG
 * =========================== */

void rng_init(void);
bool rng_validate_setup_pars(uint8_t sampling_interval);
bool rng_setup_from_eeprom(void);

void rng_read_initialize(void);
void rng_read_finalize(void);

void rng_setup_timing_debug(bool on);
void rng_timing_debug_on(void);
void rng_timing_debug_off(void);

void rng_gen_pulse_count(uint8_t *const pc_a, uint8_t *const pc_b);
void rng_gen_bit(uint8_t *const bit_a, uint8_t *const bit_b);
void rng_gen_byte(uint8_t *const byte_a, uint8_t *const byte_b);

void rng_health_monitoring(const uint8_t *const byte_a, const uint8_t *const byte_b);

void rng_start_byte_stream(comm_interface_t *const comm, uint16_t n_bytes, uint16_t interval_ms, uint8_t rng_cores, uint8_t postproc_id);
void rng_stop_byte_stream(void);

#ifdef __cplusplus
}
#endif

#endif