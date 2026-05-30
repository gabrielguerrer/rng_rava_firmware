/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <stddef.h>
#include "rava_device.h"

uint16_t device_request_count = 0;
uint32_t device_gen_bytes_count = 0;

/* ===========================
 * RAVA DEVICE
 * =========================== */

/* ===========================
 * COMM
 * =========================== */

/*
Processes the ping request. Command used to verify that the device is responsive and communication
is operational.
*/
void comm_device_ping(comm_interface_t *const comm)
{
  // Send
  send_rava_msg_header(comm, CE_OK, 0, 0, NULL);
}

/*
Processes the request, returning the number of processed device requests and the total number of
generated random bytes accumulated since the previous call. After transmitting the response, both
usage counters are reset to zero.
*/
void comm_device_get_usage(comm_interface_t *const comm)
{
  // IO Structure
  //typedef struct {} data_in_t;
  typedef struct {uint16_t request_count; uint32_t gen_bytes_count;} data_out_t;
  //data_in_t  data_in;
  data_out_t data_out;

  // Process Output
  data_out.request_count = device_request_count;
  data_out.gen_bytes_count = device_gen_bytes_count;

  // Send
  send_rava_msg_header(comm, CE_OK, 0, sizeof(data_out), &data_out);

  // Reset counters
  device_request_count = 0;
  device_gen_bytes_count = 0;
}