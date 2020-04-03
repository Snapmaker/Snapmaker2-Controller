#include "host.h"
#include "snap_dbg.h"

#include "../Marlin.h"
#include "event_handler.h"

#include "../libs/hex_print_routines.h"

Host hmi;


void Host::Init() {
  HMISERIAL.begin(115200);

  mlock_uart_ = xSemaphoreCreateMutex();

  configASSERT(mlock_uart_);

  task_started_ = true;
}


/* checkout event from UART RX ring buffer
 * Note that we may call this function many times
 * for one complete event
 */
ErrCode Host::CheckoutCmd(uint8_t *cmd, uint16_t *size) {
  int       c = -1;
  uint16_t  volatile high_byte, low_byte;
  uint16_t  calc_chk = 0;

  switch (sta_checkout_cmd_) {
  case STA_CHK_CMD_NONE:
    // no enough bytes for one command
    if (HMISERIAL.available() < COMMAND_PACKET_MIN_SIZE)
      return E_NO_RESRC;

    // ok, we have enough bytes, now find SOF
    // if no SOF in all available bytes, return with same state
    do {
      c = HMISERIAL.read();
        if (c == SOF_H) {
          if (HMISERIAL.read() == SOF_L) {
            sta_checkout_cmd_ = STA_CHK_CMD_GOT_SOF;
            break;
          }
          else {
            continue;
          }
        }
    } while (c != -1);

    if (c == -1) {
      return E_FAILURE;
    }

  case STA_CHK_CMD_GOT_SOF:
    // arrive here, the state maybe still STA_CHK_CMD_NONE, it doesn't matter
    // follow statement will make it return
    if (HMISERIAL.available() < (COMMAND_PACKET_MIN_SIZE - COMMAND_SOF_SIZE))
      return E_NO_RESRC;

    // arrive here, we got SOF, and also have enough bytes for one command
    // so just start actually check if there is a avalible command, I mean if all checksums
    // are right.
    high_byte = (uint8_t)HMISERIAL.read();
    low_byte = (uint8_t)HMISERIAL.read();

    // skip version field
    HMISERIAL.read();

    // read checksum for length
    c = HMISERIAL.read();
    if ((uint8_t)c != (uint8_t)(high_byte^low_byte)) {
      // All right, Y we got uncorrect checksum for length!
      // exit with reseting state, find SOF again in next loop
      sta_checkout_cmd_ = STA_CHK_CMD_NONE;
      length_ = 0;
      return E_FAILURE;
    }

    // now we know the length, just check if we have correct available
    // bytes in UART ring buffer
    length_ = (uint16_t)(high_byte<<8 | low_byte);

    high_byte = (uint8_t)HMISERIAL.read();
    low_byte = (uint8_t)HMISERIAL.read();
    checksum_ = (uint16_t)(high_byte<<8 | low_byte);

    sta_checkout_cmd_ = STA_CHK_CMD_GOT_CHECKSUM;

  case STA_CHK_CMD_GOT_CHECKSUM:

    if (HMISERIAL.available() < length_) {
      if (++timeout_ > 2) {
        sta_checkout_cmd_ = STA_CHK_CMD_NONE;
        timeout_ = 0;
        length_ = 0;
        checksum_ = 0;
        SERIAL_ECHOLNPAIR("not enough bytes for data: ", length_);
        return E_NO_RESRC;
      }
    }

    // read all data field in UART RX buffer
    for (int i = 0; i < length_; i++) {
      cmd[i] = (uint8_t)HMISERIAL.read();
    }

    calc_chk = CalcChecksum(cmd, length_);

    sta_checkout_cmd_ = STA_CHK_CMD_NONE;
    timeout_ = 0;

    // calc checksum of data
    if (calc_chk != checksum_) {
      SNAP_DEBUG_CMD_CHECKSUM_ERROR(true);
      SERIAL_ECHOLNPAIR("uncorrect calc checksum: ", hex_word(calc_chk), ", recv chksum: ", hex_word(checksum_));
      if (length_ > 0)
        SERIAL_ECHOLNPAIR("eid: ", cmd[0], ", opc: ", cmd[1]);
      checksum_ = 0;
      length_ = 0;
      return E_FAILURE;
    }

    // now we get an available command
    *size = length_;

    checksum_ = 0;
    length_ = 0;

    return E_SUCCESS;

  default:
    break;
  }

  return E_FAILURE;
}


uint16_t Host::CalcChecksum(uint8_t *buffer, uint16_t size) {
  uint32_t volatile checksum = 0;

  if (!size || !buffer)
    return 0;

  for (int j = 0; j < (size - 1); j = j + 2)
    checksum += (uint32_t)(buffer[j] << 8 | buffer[j + 1]);

  if (size % 2)
    checksum += buffer[size - 1];

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  return (uint16_t)checksum;
}


uint16_t Host::CalcChecksum(Event_t &event) {
  uint32_t volatile checksum = 0;
  uint16_t volatile size = event.length;
  uint16_t volatile start = 0;

  SERIAL_ECHOLNPAIR("send eid: ", hex_word(event.id), ", opc: ", hex_word(event.op_code), ", len: ", event.length);

  /* when send out event, we will have a event structure
   * so we will have independent event_id and maybe more one op_code.
   * If yes, need to calculate them into checksum
   */
  if (event.id < INVALID_EVENT_ID) {
    if (event.op_code < INVALID_OP_CODE) {
      checksum += (event.id<<8 | (event.op_code&0x00FF));
      SERIAL_ECHOLNPAIR("eid+op: chk: ", hex_word(checksum));
    }
    else if (size > 0) {
      // No independent op_code, but have data field
      checksum += (event.id<<8 | event.data[0]);
      start = 1;
      SERIAL_ECHOLNPAIR("eid+d0: chk: ", hex_word(checksum));
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

  if ((size - start) % 2)
    checksum += event.data[size - 1];

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  SERIAL_ECHOLNPAIR("checksum for eid: ", hex_word(event.id), ", opc: ", hex_word(event.op_code), ": ", hex_word(checksum));

  return (uint16_t)checksum;
}


/* Send event to peer host
 * here we have local buffer of PDU header for reentrant
 */
ErrCode Host::Send(Event_t &event) {
  int i = 0;
  uint8_t pdu_header[PDU_HEADER_SIZE + 2];
  uint16_t tmp = 0;

  BaseType_t ret = pdPASS;

  pdu_header[i++] = SOF_H;
  pdu_header[i++] = SOF_L;

  if (event.op_code < INVALID_OP_CODE)
    tmp = event.length + 2;
  else
    tmp = event.length + 1;
  pdu_header[i++] = (uint8_t)(tmp>>8);
  pdu_header[i++] = (uint8_t)(tmp & 0x00FF);

  pdu_header[i++] = PROTOCOL_SM2_VER;

  pdu_header[i++] = pdu_header[PDU_IDX_DATA_LEN_H] ^ pdu_header[PDU_IDX_DATA_LEN_L];

  tmp = CalcChecksum(event);

  pdu_header[i++] = (uint8_t)(tmp>>8);
  pdu_header[i++] = (uint8_t)(tmp & 0x00FF);

  pdu_header[i++] = (uint8_t)event.id;

  if (event.op_code < INVALID_OP_CODE)
    pdu_header[i++] = (uint8_t)event.op_code;

  // lock the uart, there will be more than one writer
  if (task_started_)
    ret = xSemaphoreTake(mlock_uart_, configTICK_RATE_HZ/100);
  if (ret != pdPASS) {
    SERIAL_ECHOLN("failed to get HMI uart lock!");
    return E_BUSY;
  }

  for (int j = 0; j < i; j++)
    HMISERIAL.write(pdu_header[j]);

  for (int j = 0; j < event.length; j++)
    HMISERIAL.write(event.data[j]);

  if (task_started_)
    xSemaphoreGive(mlock_uart_);

  return E_SUCCESS;
}

