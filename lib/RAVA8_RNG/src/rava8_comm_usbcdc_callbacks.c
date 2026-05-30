/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

// #include <avr/sleep.h>
#include "rava8_comm_usbcdc_callbacks.h"
#include "rava8_comm_usbcdc.h"

/* ===========================
 * RAVA8 COMM USB CDC CALLBACKS
 * =========================== */

/*
Event handler for the library USB Connection event
*/
void EVENT_USB_Device_Connect(void)
{
}

/*
Event handler for the library USB Disconnection event
*/
void EVENT_USB_Device_Disconnect(void)
{
}

/*
Event handler for the library USB Configuration Changed event
*/
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&lufa_usbcdc_if);
}

/*
Event handler for the library USB Control Request reception event
*/
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&lufa_usbcdc_if);

	/*
	Unforntunatelly this didn't work. Device wakes up, but the ability to receive and send bytes is
	lost afterward.

	// USB enumeration completed?
	if (USB_DeviceState == DEVICE_STATE_Configured) {

		// Enables interrupt to wake the device whenever the host sends data
		Endpoint_SelectEndpoint(CDC_RX_EPADDR);
		UEIENX |= _BV(RXOUTE);

		// Enter idle sleep mode, which stops the CPU clock
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();
	}
	*/
}

/*
Host stopped USB activity.

Linux typically does not autosuspend CDC ACM devices by default, unless runtime power management is
explicitly enabled (e.g. via udev rules). Therefore, no attempt is made here to enter a deeper
low-power sleep mode.
*/
void EVENT_USB_Device_Suspend(void)
{
}

/*
Bus activity resumed.
*/
void EVENT_USB_Device_WakeUp(void)
{
}

/*
CDC class driver callback function the processing of changes to the virtual control lines sent
from the host.
*/
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	// bool host_ready = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) != 0;
}