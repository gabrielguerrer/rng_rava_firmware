/**
 * Copyright (c) 2026 Gabriel Guerrer
 *
 * Distributed under the MIT license - See LICENSE for details
 */

#include <util/crc16.h>
#include "rava_comm.h"
#include "rava_device.h"
#include "rava_rng.h"

/* ===========================
 * RAVA COMM
 * =========================== */

/*
Processes the communication stream using a byte-oriented state machine parser, with parser states
defined in COMM_STATE_IDS.

Since the RAVA device only accepts incoming messages with R_LEN == 0, random-payload reception is
disabled in the device parser. Random payload retrieval is instead expected to be implemented by
the host-side parser.

The function consumes one byte at a time and returns true when either:
- A valid message has been successfully parsed
- A protocol error has been detected
*/
bool parse_rava_byte(struct comm_state_t *const state, struct comm_message_t *const msg, uint8_t byte) {
  switch (state->id) {

    // SOF
    case CS_WAIT_SOF_A: {
      msg->req_error = CE_OK;
      if (byte == COMM_SOF_A) {
        state->id = CS_WAIT_SOF_B;
      }
      break;
    }

    case CS_WAIT_SOF_B: {
      if (byte == COMM_SOF_B) {
        state->id = CS_READ_REQ_ID;
      } else if (byte == COMM_SOF_A) {
        // Stay in sync
        state->id = CS_WAIT_SOF_B;
      } else {
        state->id = CS_WAIT_SOF_A;
      }
      break;
    }

    // REQ_ID
    case CS_READ_REQ_ID: {
      msg->req_id = byte;
      state->crc = _crc_ccitt_update(0xFFFF, byte);
      state->id = CS_READ_REQ_ERROR;
      break;
    }

    // REQ_STATUS
    case CS_READ_REQ_ERROR:
      msg->req_error = byte;
      state->crc = _crc_ccitt_update(state->crc, byte);
      state->id = CS_READ_CMD_ID;
      break;

    // READ_CMD
    case CS_READ_CMD_ID: {
      msg->comm_id = byte;
      state->crc = _crc_ccitt_update(state->crc, byte);
      state->id = CS_READ_RAND_LEN_A;
      break;
    }

    // READ_RAND_LEN
    case CS_READ_RAND_LEN_A:
      msg->rand_len = byte;
      state->crc = _crc_ccitt_update(state->crc, byte);
      state->id = CS_READ_RAND_LEN_B;
      break;

    case CS_READ_RAND_LEN_B:
      msg->rand_len |= ((uint16_t)byte << 8);
      state->crc = _crc_ccitt_update(state->crc, byte);

      if (msg->rand_len > 2 * RNG_GEN_MAX_NBYTES_PER_CORE) {
        msg->req_error = CE_INVALID_RAND_LEN;
        state->id = CS_WAIT_SOF_A;
        // Error
        return true;
      }

      state->id = CS_READ_DATA_LEN;
      break;

    // READ_DATA_LEN
    case CS_READ_DATA_LEN: {
      msg->data_len = byte;
      if (msg->data_len > COMM_DATA_MAX_LEN) {
        msg->req_error = CE_INVALID_DATA_LEN;
        state->id = CS_WAIT_SOF_A;
        // Error
        return true;
      }
      state->crc = _crc_ccitt_update(state->crc, byte);
      state->data_idx = 0;
      if (msg->data_len == 0) {
        state->id = CS_READ_CRC_A;
      } else {
        state->id = CS_READ_DATA;
      }
      break;
    }

    // READ_DATA
    case CS_READ_DATA: {
      msg->data[state->data_idx++] = byte;
      state->crc = _crc_ccitt_update(state->crc, byte);
      if (state->data_idx >= msg->data_len) {
        state->id = CS_READ_CRC_A;
      }
      break;
    }

    // READ_CRC
    case CS_READ_CRC_A: {
      state->crc_rcv = byte;
      state->id = CS_READ_CRC_B;
      break;
    }

    case CS_READ_CRC_B: {
      state->crc_rcv |= ((uint16_t)byte << 8);
      if (state->crc_rcv != state->crc) {
        msg->req_error = CE_INVALID_CRC;
        state->id = CS_WAIT_SOF_A;
        // Error
        return true;
      }
      // Command properly formed in msg
      state->id = CS_WAIT_SOF_A;
      return true;

      // state->rand_idx = 0;
      // if (msg->rand_len == 0) {
      //   state->id = CS_WAIT_SOF_A;
      //   // Command properly formed in msg
      //   return true;
      // }
      // else {
      //   state->id = CS_READ_RAND;
      // }
      // break;
    }

    // // RAND
    // case CS_READ_RAND: {
    //   msg->rand[state->rand_idx++] = byte;
    //   if (state->rand_idx >= msg->rand_len) {
    //     state->id = CS_WAIT_SOF_A;
    //     // Command properly formed in msg
    //     return true
    //   }
    //   break;
    // }
  }

  // Command not ready yet
  return false;
}

/*
Reads and parses incoming bytes from the communication interface until a complete message or
protocol error is detected.

The function continuously consumes bytes available through the provided communication interface and
feeds them to the state machine parser. When a valid message is received, the parsed data is stored
in comm_if->msg and the function returns true.

Returns:
- True  : A complete message or parsing error was detected
- False : No complete message is currently available
*/
bool read_rava_msg(comm_interface_t *const comm_if)
{
  // Byte available?
  uint8_t b;
  if (comm_if->read(&b)) {

    // Process RAVA msg
    bool msg_found = parse_rava_byte(&comm_if->state, &comm_if->msg, b);
    if (msg_found) {

      // Update usage counter
      if (comm_if->msg.comm_id != COMM_DEVICE_GET_USAGE) {
        device_request_count += 1;
      }

      // Check for errors
      if (comm_if->msg.req_error != CE_OK) {
        send_rava_msg_header(comm_if, comm_if->msg.req_error, 0, 0, NULL);
        return false;
      }

      return true;
    }
  }
  return false;
}

/*
Writes a RAVA protocol message header to the communication interface.

The function serializes and transmits the protocol frame header, optional data payload, and CRC16
checksum using the provided communication interface.

The function only transmits the message header and optional DATA section. If random payload bytes
are required (R_LEN > 0), they must be transmitted separately after this function returns.
*/
void send_rava_msg_header(comm_interface_t *const comm_if,
  uint8_t req_error, uint16_t rand_len, uint8_t data_len, const void *const data)
{
  uint16_u rand_len_u = {.i = rand_len};

  // Write message header
  uint8_t msg_header[COMM_HEADER_MIN_LEN-2];
  msg_header[0] = COMM_SOF_A;
  msg_header[1] = COMM_SOF_B;
  msg_header[2] = comm_if->msg.req_id;
  msg_header[3] = req_error;
  msg_header[4] = comm_if->msg.comm_id;
  msg_header[5] = rand_len_u.b[0];
  msg_header[6] = rand_len_u.b[1];
  msg_header[7] = data_len;

  comm_if->write_buf(msg_header, sizeof(msg_header));

  // Write message data
  comm_if->write_buf((uint8_t*)data, data_len);

  // Compute CRC
  uint16_u crc = {.i = 0xFFFF};
  for (uint8_t i = 2; i < COMM_HEADER_MIN_LEN-2; i++) {
      crc.i = _crc_ccitt_update(crc.i, (msg_header)[i]);
  }
  for (uint8_t i = 0; i < data_len; i++) {
      crc.i = _crc_ccitt_update(crc.i, ((uint8_t*)data)[i]);
  }

  // Write CRC
  comm_if->write_buf(crc.b, 2);

  // Flush
  comm_if->flush();
}