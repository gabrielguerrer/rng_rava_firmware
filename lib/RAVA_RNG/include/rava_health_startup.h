/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
The health module implements entropy monitoring tests based on the NIST recommendations described
in:

M. Sonmez, E. Barker, J. Kelsey, K. McKay, M. Baish, and M. Boyle,
SP 800-90B: Recommendation for the Entropy Sources Used for Random Bit Generation,
National Institute of Standards and Technology (NIST), 2018.

At power-up, the rava_health_startup module executes startup tests to verify the correct operation
of the noise sources. If all startup tests pass, the device becomes ready to receive commands and
generate random data. Otherwise, the failure condition is reported and randomness generation
commands are rejected while the firmware remains in the `task_health_startup_failed()` loop.

The startup tests evaluate bit and byte bias, and the average pulse count statistics. Distribution
thresholds are selected to correspond to a false-positive rate of approximately 0.001%.
*/

#ifndef RAVA_HEALTH_STARTUP_H
#define RAVA_HEALTH_STARTUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "rava_comm.h"

/* ===========================
 * RAVA HEALTH STARTUP
 * =========================== */

bool health_startup_run_tests(void);
bool health_startup_get_tests_result(void);

// COMM
void comm_health_startup_run(comm_interface_t *const comm);
void comm_health_startup_get_results(comm_interface_t *const comm);

#ifdef __cplusplus
}
#endif

#endif