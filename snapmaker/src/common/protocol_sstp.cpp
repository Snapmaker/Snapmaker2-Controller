/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "protocol_sstp.h"
#include "debug.h"

#include <libmaple/ring_buffer.h>
#include "MapleFreeRTOS1030.h"

// marlin headers
#include "src/core/serial.h"
#include "src/libs/hex_print_routines.h"

#define LOG_HEAD  "SSTP: "
#define TIMEOUT_FOR_HEADER      (2)
#define TIMEOUT_FOR_NEXT_BYTE   (1)

// min br = 115200bps, = 14.4 bytes/ms
// max data length = 516 bytes in transferring FW
// so, max MS to wait = 516 / 14.4 = 36ms
#define DELAY_MS_FOR_DATA       (5)
#define TIMEOUT_COUNT_FOR_DATA  (100/DELAY_MS_FOR_DATA)

/* checkout event from UART RX ring buffer
 * Note that we may call this function many times
 * for one complete event
 */
ErrCode ProtocolSSTP::Parse(ring_buffer *rb, uint8_t *out, uint16_t &size) {
  int16_t   c = -1;
  int       i;
  uint16_t  timeout = 0;
  uint16_t  length = 0;
  uint16_t  calc_chk = 0;
  uint16_t  recv_chk = 0;

  // no enough bytes for one command
  if (rb_full_count(rb) < SSTP_PDU_HEADER_SIZE)
    return E_NO_RESRC;

  // ok, we have enough bytes, now find SOF
  // if no SOF in all available bytes, return with same state
  do {
      if (c == SSTP_PDU_SOF_H) {
        c = rb_safe_remove(rb);
        if (c == SSTP_PDU_SOF_L) {
          break;
        }
        else {
          continue;
        }
      }
      else {
        c = rb_safe_remove(rb);
      }
  } while (c != -1);

  if (c == -1) {
    // not found SOF
    return E_NO_SOF;
  }

  // if it doesn't have enough bytes in ring buffer for header, just return
  while (rb_full_count(rb) < (SSTP_PDU_HEADER_SIZE - 1)) {
    vTaskDelay(pdMS_TO_TICKS(1));
    if (++timeout > TIMEOUT_FOR_HEADER) {
      SERIAL_ECHOLN(LOG_HEAD "timeout to wait for PDU header");
      return E_NO_HEADER;
    }
  }

  // arrive here, we got SOF, and also have enough bytes for one command header
  // so just start actually check if we got correct header.
  // read all of header
  for (i = 2; i < (SSTP_PDU_HEADER_SIZE); i++) {
    timeout = 0;
    do {
      c = rb_safe_remove(rb);
      if (c != -1)
        break;
      else {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (++timeout > TIMEOUT_FOR_NEXT_BYTE) {
          SERIAL_ECHOLNPAIR(LOG_HEAD "not enough bytes for header, just got ", i);
          return E_NO_HEADER;
        }
      }
    } while (1);
    header_[i] = (uint8_t)c;
  }

  // now we know the length, just check if we have correct data length
  length = (uint16_t)(header_[SSTP_PDU_IDX_DATA_LEN_H]<<8 | header_[SSTP_PDU_IDX_DATA_LEN_L]);
  // 1. check if we got normal length
  if (length > SSTP_RECV_BUFFER_SIZE) {
    SERIAL_ECHOLNPAIR(LOG_HEAD "length out of range, recv: ", hex_word(length));
    return E_INVALID_DATA_LENGTH;
  }

  // 2. verify the length with checksum
  c = header_[SSTP_PDU_IDX_DATA_LEN_H]^header_[SSTP_PDU_IDX_DATA_LEN_L];
  if (header_[SSTP_PDU_IDX_LEN_CHK] != (uint8_t)c) {
    SERIAL_ECHOLNPAIR(LOG_HEAD "length checksum error, recv: ", hex_byte(header_[SSTP_PDU_IDX_LEN_CHK]), "calc: ", hex_byte((uint8_t)c));
    return E_INVALID_DATA_LENGTH;
  }

  // ok, got correct length, checkout the command checksum then switch state
  recv_chk = (uint16_t)(header_[SSTP_PDU_IDX_CHKSUM_H]<<8 | header_[SSTP_PDU_IDX_CHKSUM_L]);

  timeout = 0;
  while (rb_full_count(rb) < length) {
    vTaskDelay(pdMS_TO_TICKS(DELAY_MS_FOR_DATA));
    if (++timeout > TIMEOUT_COUNT_FOR_DATA) {
      SERIAL_ECHOLNPAIR(LOG_HEAD "not enough bytes for data: ", length);
      return E_NO_DATA;
    }
  }

  // read all data field in UART RX buffer
  for (int i = 0; i < length; i++) {
    timeout = 0;
    do {
      c = rb_safe_remove(rb);
      if (c != -1)
        break;
      else {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (++timeout > TIMEOUT_FOR_NEXT_BYTE) {
          SERIAL_ECHOLN(LOG_HEAD "timeout wait for next byte of data");
          return E_NO_DATA;
        }
      }
    } while (1);
    out[i] = (uint8_t)c;
  }

  calc_chk = CalcChecksum(out, length);

  // calc checksum of data
  if (calc_chk != recv_chk) {
    SNAP_DEBUG_CMD_CHECKSUM_ERROR(true);
    SERIAL_ECHOLNPAIR(LOG_HEAD "uncorrect calc checksum: ", hex_word(calc_chk), ", recv chksum: ", hex_word(recv_chk));
    if (length > 0) {
      SERIAL_ECHO(LOG_HEAD "content:");
      for (int i = 0; i < length; i++) {
        SERIAL_ECHOPAIR(" ", hex_byte(out[i]));
      }
      SERIAL_EOL();
      SERIAL_EOL();
    }
    return E_INVALID_DATA;
  }

  // now we get an available command
  size = length;

  return E_SUCCESS;
}

ErrCode ProtocolSSTP::Parse(RingBuffer<uint8_t> &ring, uint8_t *out, uint16_t &length) {
  uint16_t calc_chk = 0;


  switch (state_) {
  case PROTOCOL_SSTP_STATE_IDLE:
    // SOF has 2 bytes, so try to read 2 bytes
    if (ring.RemoveMulti(header_, (int32_t)SSTP_PDU_SOF_SIZE) != SSTP_PDU_SOF_SIZE)
      return E_NO_RESRC;

    // check if we got SOF
    if (header_[0] != SSTP_PDU_SOF_H || header_[1] != SSTP_PDU_SOF_L)
      return E_NO_SOF;

    state_ = PROTOCOL_SSTP_STATE_FOUND_SOF;
    timeout_ = 0;

  case PROTOCOL_SSTP_STATE_FOUND_SOF:
    // check if ring buffer has the remain of protocol header_
    if (ring.Available() < (SSTP_HEADER_SIZE - SSTP_PDU_SOF_SIZE)) {
      if (++timeout_ > SSTP_HEADER_TIMEOUT) {
        state_ = PROTOCOL_SSTP_STATE_IDLE;
        return E_TIMEOUT;
      }
      return E_NO_HEADER;
    }

    ring.RemoveMulti(header_ + SSTP_PDU_SOF_SIZE, (int32_t)(SSTP_HEADER_SIZE - SSTP_PDU_SOF_SIZE));

    // confirm the checksum of length_
    if (header_[SSTP_PDU_IDX_LEN_CHK] != (uint8_t)(header_[SSTP_PDU_IDX_DATA_LEN_H]^header_[SSTP_PDU_IDX_DATA_LEN_L])) {
      state_ = PROTOCOL_SSTP_STATE_IDLE;
      return E_INVALID_DATA_LENGTH;
    }

    // to avoid length_ is out of range
    length_ = header_[SSTP_PDU_IDX_DATA_LEN_H]<<8 | header_[SSTP_PDU_IDX_DATA_LEN_L];
    if (length_ > SSTP_INVALID_DATA_LENGTH) {
      state_ = PROTOCOL_SSTP_STATE_IDLE;
      return E_INVALID_DATA_LENGTH;
    }

    state_ = PROTOCOL_SSTP_STATE_GOT_LENGTH;
    timeout_ = 0;

  case PROTOCOL_SSTP_STATE_GOT_LENGTH:
    if (ring.Available() < length_) {
      if (++timeout_ > SSTP_DATA_TIMEOUT) {
        state_ = PROTOCOL_SSTP_STATE_IDLE;
        return E_TIMEOUT;
      }
      return E_NO_DATA;
    }

    ring.RemoveMulti(out, (int32_t)length_);

    calc_chk = CalcChecksum(out, length_);
    state_ = PROTOCOL_SSTP_STATE_IDLE;

    if (calc_chk != (uint16_t)(header_[SSTP_PDU_IDX_CHKSUM_H]<<8 | header_[SSTP_PDU_IDX_CHKSUM_L])) {
      return E_INVALID_DATA;
    }

    length = length_;

    return E_SUCCESS;

    default:
      break;
  }

  state_ = PROTOCOL_SSTP_STATE_IDLE;
  return E_INVALID_STATE;
}


ErrCode ProtocolSSTP::Package(uint8_t *in, uint8_t *out, uint16_t &length) {
  uint16_t checksum;
  uint16_t index = 0;

  out[index++] = SSTP_PDU_SOF_H;
  out[index++] = SSTP_PDU_SOF_L;

  // length
  out[index++] = (uint8_t)(length>>8);
  out[index++] = (uint8_t)(length & 0x00FF);

  out[index++] = SSTP_PDU_VERSION;

  // checksum of length
  out[index++] = (uint8_t)((uint8_t)(length>>8) ^ (uint8_t)(length & 0x00FF));

  checksum = CalcChecksum(in, length);

  // checksum of data field
  out[index++] = (uint8_t)(checksum>>8);
  out[index++] = (uint8_t)(checksum & 0x00FF);

  for (int i = 0; i < length; i++)
    out[index++] = in[i];

  length = index;

  return E_SUCCESS;
}


uint16_t ProtocolSSTP::CalcChecksum(uint8_t *buffer, uint16_t length) {
  uint32_t volatile checksum = 0;

  if (!length || !buffer)
    return 0;

  for (int j = 0; j < (length - 1); j = j + 2)
    checksum += (uint32_t)(buffer[j] << 8 | buffer[j + 1]);

  if (length % 2)
    checksum += buffer[length - 1];

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  return (uint16_t)checksum;
}


uint16_t ProtocolSSTP::CalcChecksum(SSTP_Event_t &event) {
  uint32_t volatile checksum = 0;
  uint16_t size = event.length;
  uint16_t start = 0;

  if (event.id >= SSTP_INVALID_EVENT_ID)
    return 0;

  /* when send out event, we will have a event structure
   * so we will have independent event_id and maybe more one op_code.
   * If yes, need to calculate them into checksum
   */
 
  if (size > 0) {
    // data field exists
    if (event.op_code < SSTP_INVALID_OP_CODE) {
      // event id and op code exist
      checksum = (event.id<<8 | (event.op_code&0x00FF));
    }
    else {
      // No independent op_code
      checksum = (event.id<<8 | event.data[0]);
      start = 1;
    } 
  }
  else {
    // no data field
    if (event.op_code < SSTP_INVALID_OP_CODE) {
      // only event id and op code
      checksum = (event.id<<8 | (event.op_code&0x00FF));
    }
    else {
      // just event_id, no op_code
      checksum = event.id;
    }

    goto out;
  }


  for (int j = start; j < (size - 1); j = j + 2)
    checksum += (uint32_t)(event.data[j] << 8 | event.data[j + 1]);

  if ((size - start) % 2) {
    checksum += event.data[size - 1];
  }

out:
  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
 
  checksum = ~checksum;
  return (uint16_t)checksum;
}
