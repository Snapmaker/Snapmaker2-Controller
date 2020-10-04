#include "protocol_sstp.h"


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
  uint16_t volatile size = event.length;
  uint16_t volatile start = 0;

  /* when send out event, we will have a event structure
   * so we will have independent event_id and maybe more one op_code.
   * If yes, need to calculate them into checksum
   */
  if (event.id < SSTP_INVALID_EVENT_ID) {
    if (event.op_code < SSTP_INVALID_OP_CODE) {
      checksum += (event.id<<8 | (event.op_code&0x00FF));
    }
    else if (size > 0) {
      // No independent op_code, but have data field
      checksum += (event.id<<8 | event.data[0]);
      start = 1;
    }
    else {
      // just event_id, no op_code, and no data field
      checksum += event.id;

      checksum = ~checksum;

      return (uint16_t)checksum;
    }
  }

  for (int j = start; j < (size - 1); j = j + 2)
    checksum += (uint32_t)(event.data[j] << 8 | event.data[j + 1]);

  if ((size - start) % 2) {
    checksum += event.data[size - 1];
  }

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  return (uint16_t)checksum;
}
