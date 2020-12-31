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
#ifndef SNAPMAKER_PROTOCOL_SSTP_H_
#define SNAPMAKER_PROTOCOL_SSTP_H_

#include "error.h"
#include "../utils/ring_buffer.h"

#include <libmaple/ring_buffer.h>

// protocol relative macros
#define SSTP_PDU_SOF_H   0xAA
#define SSTP_PDU_SOF_L   0x55

#define SSTP_PDU_SOF_SIZE 2

#define SSTP_PDU_VERSION  0

#define SSTP_PDU_HEADER_SIZE   8

// index of each field in command header
#define SSTP_PDU_IDX_DATA_LEN    2
#define SSTP_PDU_IDX_DATA_LEN_H  2
#define SSTP_PDU_IDX_DATA_LEN_L  3

#define SSTP_PDU_IDX_VERSION   4
#define SSTP_PDU_IDX_LEN_CHK   5

#define SSTP_PDU_IDX_CHKSUM    6
#define SSTP_PDU_IDX_CHKSUM_H  6
#define SSTP_PDU_IDX_CHKSUM_L  7

#define SSTP_PDU_IDX_EVENT_ID  8
#define SSTP_PDU_IDX_OP_CODE   9
#define SSTP_PDU_IDX_DATA0     10

#define SSTP_TIMEOUT_FOR_HEADER    (2)

#define SSTP_TIMEOUT_FOR_NEXT_BYTE (1)

#define SSTP_INVALID_EVENT_ID      ((uint16_t)0x100)
#define SSTP_INVALID_OP_CODE       ((uint16_t)0x100)
#define SSTP_INVALID_DATA_LENGTH   ((uint16_t)1024)

// size of protocol header
#define SSTP_HEADER_SIZE   8

// timeout to wait for header field
#define SSTP_HEADER_TIMEOUT  (10)
// timeout to wait for data field
#define SSTP_DATA_TIMEOUT    (1000)

#define SSTP_EVENT_ID_INVALID (0xFFFF)
#define SSTP_OP_CODE_INVALID  (0xFFFF)

#define SSTP_RECV_BUFFER_SIZE (1024)

// convert lcoal word to the bytes in protocol data unit
#define WORD_TO_PDU_BYTES(buff, var)  do { \
                                        (buff)[0] = (uint8_t)((var)>>24 & 0x000000FF);  \
                                        (buff)[1] = (uint8_t)((var)>>16 & 0x000000FF);  \
                                        (buff)[2] = (uint8_t)((var)>>8 & 0x000000FF);  \
                                        (buff)[3] = (uint8_t)((var) & 0x000000FF);  \
                                      } while (0)

// convert lcoal word to the bytes in protocol data unit, and moving index forward
#define WORD_TO_PDU_BYTES_INDEX_MOVE(buff, var, index)  do { \
                                                          (buff)[index++] = (uint8_t)((var)>>24 & 0x000000FF);  \
                                                          (buff)[index++] = (uint8_t)((var)>>16 & 0x000000FF);  \
                                                          (buff)[index++] = (uint8_t)((var)>>8 & 0x000000FF);  \
                                                          (buff)[index++] = (uint8_t)((var) & 0x000000FF);  \
                                                        } while (0)

// convert bytes in protocol unit to local word
#define PDU_TO_LOCAL_WORD(var, buff)   do { \
                                                var = (typeof(var))((buff)[0]<<24 | (buff)[1]<<16 | (buff)[2]<<8 | (buff)[3]); \
                                              } while (0)

// convert local half word to bytes in PDU
#define HWORD_TO_PDU_BYTES_INDE_MOVE(buff, var, index)  do { \
                                                    (buff)[index++] = (uint8_t)((var)>>8 & 0x00FF);  \
                                                    (buff)[index++] = (uint8_t)((var) & 0x00FF);  \
                                                  } while (0)

// convert bytes in protocol unit to local half word
#define PDU_TO_LOCAL_HALF_WORD(var, buff)  do { \
                                                    var = (typeof(var))((buff)[0]<<8 | (buff)[1]); \
                                                  } while (0)


enum ProtocolSM2State {
  PROTOCOL_SSTP_STATE_IDLE,
  PROTOCOL_SSTP_STATE_FOUND_SOF,
  PROTOCOL_SSTP_STATE_GOT_LENGTH,
  PROTOCOL_SSTP_STATE_GOT_DATA,

  PROTOCOL_SSTP_STATE_INVALID
};


typedef struct {
  uint16_t id;
  uint16_t op_code;

  uint16_t length;
  uint8_t  *data;
} SSTP_Event_t;


class ProtocolSSTP {
  public:
    ProtocolSSTP() {
      timeout_ = 0;
      length_ = 0;
      state_ = PROTOCOL_SSTP_STATE_IDLE;
    }

    ErrCode Parse(RingBuffer<uint8_t> &ring, uint8_t *out, uint16_t &length);
    ErrCode Parse(ring_buffer *rb, uint8_t *out, uint16_t &length);

    ErrCode Package(uint8_t *in_data, uint8_t *out, uint16_t &length);

    uint16_t CalcChecksum(SSTP_Event_t &event);

  private:
    uint16_t CalcChecksum(uint8_t *buffer, uint16_t length);


  private:
    ProtocolSM2State state_;
    uint8_t cmd_bytes_;

    uint8_t  header_[SSTP_HEADER_SIZE];
    uint16_t length_;
    uint32_t timeout_;
};


#endif  // #ifndef PROTOCOL_SSTP_H_
