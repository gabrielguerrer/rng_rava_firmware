/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
Defines the USART transport implementation for RAVA8, based on the LUFA library.
*/

#ifndef RAVA8_COMM_USART_H
#define RAVA8_COMM_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rava_comm.h>

/* ===========================
 * RAVA8 COMM USART
 * =========================== */

#ifndef USART_BAUD
#define USART_BAUD 250000
#endif

extern comm_interface_t comm_usart_if;

void usart_init(void);

#ifdef __cplusplus
}
#endif

#endif