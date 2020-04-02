#include "host.h"
#include "snap_dbg.h"

#include "../Marlin.h"
#include "event_handler.h"

Host hmi;


void Host::Init() {
  HMISERIAL.begin(115200);

  inited_ = true;
}


void Host::SwapBytesSeq(uint8_t *dst, uint8_t *src, uint16_t size) {
  while (size-- > 0) {
    *dst++ = *(src + size);
  }
}


/* checkout event from UART RX ring buffer
 * Note that we may call this function many times
 * for one complete event
 */
int Host::CheckoutCmd(uint8_t *cmd) {
  static uint16_t  length = 0;

  int       c = -1;
  uint8_t   length_h, length_l;
  uint16_t  checksum = 0;

  switch (sta_checkout_cmd_) {
  case STA_CHK_CMD_NONE:
    // no enough bytes for one command
    if (HMISERIAL.available() < COMMAND_PACKET_MIN_SIZE)
      return -E_NO_RESRC;

    // ok, we have enough bytes, now find SOF
    // if no SOF in all available bytes, return with same state
    do {
      c = HMISERIAL.read();
      if (c != -1) {
        if (c == SOF_H) {
          if (HMISERIAL.read() == SOF_L) {
            sta_checkout_cmd_ = STA_CHK_CMD_GOT_SOF;
            break;
          }
          else {
            continue;
          }
        }
      }
    } while (c != -1);


  case STA_CHK_CMD_GOT_SOF:
    // arrive here, the state maybe still STA_CHK_CMD_NONE, it doesn't matter
    // follow statement will make it return
    if (HMISERIAL.available() < (COMMAND_PACKET_MIN_SIZE - COMMAND_SOF_SIZE))
      return -E_NO_RESRC;

    // arrive here, we got SOF, and also have enough bytes for one command
    // so just start actually check if there is a avalible command, I mean if all checksums
    // are right.
    length_h = (uint8_t)HMISERIAL.read();
    length_l = (uint8_t)HMISERIAL.read();

    // skip version field
    HMISERIAL.read();

    // read checksum for length
    c = HMISERIAL.read();
    if ((uint8_t)c != (length_h^length_l)) {
      // All right, Y we got uncorrect checksum for length!
      // exit with reseting state, find SOF again in next loop
      sta_checkout_cmd_ = STA_CHK_CMD_NONE;
      return -E_FAILURE;
    }

    // now we know the length, just check if we have correct available
    // bytes in UART ring buffer
    length = (uint16_t)(length_h<<8 | length_l);
    sta_checkout_cmd_ = STA_CHK_CMD_GOT_LENGTH;


  case STA_CHK_CMD_GOT_LENGTH:
    // data checksum has 2 bytes, so add 2 to length then compare it with
    // available bytes in UART RX buffser
    if (HMISERIAL.available() < (length + 2)) {
      // jsut return to wait eoungh bytes in UART RX buffer
      return -E_NO_RESRC;
    }

    c = HMISERIAL.read();
    checksum = (uint16_t)(c<<8 | HMISERIAL.read());

    // read all data field in UART RX buffer
    for (int i = 0; i < length; i++) {
      cmd[i] = HMISERIAL.read();
    }

    // calc checksum of data
    if (CalcChecksum(cmd, length) != checksum) {
      SNAP_DEBUG_CMD_CHECKSUM_ERROR(true);
      LOG_E("uncorrect checksum for data!\n");
      sta_checkout_cmd_ = STA_CHK_CMD_NONE;
      length = 0;
      return -E_FAILURE;
    }

    // now we get an available command
    return length;

  default:
    break;
  }

  return -E_FAILURE;
}


uint16_t Host::CalcChecksum(uint8_t *buffer, uint16_t size, uint16_t event_id, uint16_t op_code) {
  uint32_t  checksum = 0;
  int       start = 0;

  /* when send out event, we will have a event structure
   * so we will have independent event_id and maybe more one op_code.
   * If yes, need to calculate them into checksum
   */
  if (event_id < INVALID_EVENT_ID) {
    if (op_code < INVALID_OP_CODE) {
      checksum += (event_id<<8 | (op_code&0x00FF));
    }
    else if (size > 0) {
      // No independent op_code, but have data field
      checksum += (event_id<<8 | buffer[0]);
      start = 1;
    }
    else {
      // just event_id, no op_code, and no data field
      // actually it should not arrive here!!!
      checksum += event_id;
    }
  }

  for (int j = start; j < (size - 1); j = j + 2)
    checksum += (uint32_t)(buffer[j] << 8 | buffer[j + 1]);

  if (size % 2)
    checksum += buffer[size - 1];

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  return (uint16_t)checksum;
}


uint16_t Host::CalcChecksum(Event_t &e) {
  CalcChecksum(e.data, e.length, e.id, e.op_code);
}


/* Send event to peer host
 * here we have local buffer of PDU header for reentrant
 */
ErrCode Host::Send(Event_t &event) {
  int i = 0;
  uint8_t pdu_header[PDU_HEADER_SIZE + 2];
  uint16_t checksum = 0;

  BaseType_t ret;

  pdu_header[i++] = SOF_H;
  pdu_header[i++] = SOF_L;

  pdu_header[i++] = (uint8_t)(event.length>>8);
  pdu_header[i++] = (uint8_t)(event.length & 0x00FF);

  pdu_header[i++] = PROTOCOL_SM2_VER;

  pdu_header[i++] = pdu_header[PDU_IDX_DATA_LEN_H] ^ pdu_header[PDU_IDX_DATA_LEN_L];

  checksum = CalcChecksum(event);

  pdu_header[i++] = (uint8_t)(checksum>>8);
  pdu_header[i++] = (uint8_t)(checksum & 0x00FF);

  pdu_header[i++] = event.id;

  if (event.op_code < INVALID_OP_CODE)
    pdu_header[i++] = event.op_code;

  // lock the uart, there will be more than one writer
  ret = xSemaphoreTake(mlock_uart_, configTICK_RATE_HZ/100);
  if (ret != pdPASS) {
    SERIAL_ECHOLN("failed to get HMI uart lock!");
    return E_BUSY;
  }

  for (int j = 0; j < i; j++)
    HMISERIAL.write(pdu_header[j]);

  for (int j = 0; j < event.length; j++)
    HMISERIAL.write(event.data[j]);

  xSemaphoreGive(mlock_uart_);

  return E_SUCCESS;
}

