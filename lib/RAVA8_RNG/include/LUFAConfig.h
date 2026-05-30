/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

/*
This configures LUFA's compile time options.
*/

#ifndef LUFA_CONFIG_H
#define LUFA_CONFIG_H

//General USB Driver Related Tokens
#define USE_STATIC_OPTIONS      (USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED | USB_OPT_MANUAL_PLL)
#define USB_DEVICE_ONLY

// USB Device Mode Driver Related Tokens
#define FIXED_CONTROL_ENDPOINT_SIZE      8
#define FIXED_NUM_CONFIGURATIONS         1
#define INTERRUPT_CONTROL_ENDPOINT

#endif