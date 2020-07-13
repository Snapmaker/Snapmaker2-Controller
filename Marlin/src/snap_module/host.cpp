#include "host.h"
#include "snap_dbg.h"

#include "../Marlin.h"
#include "event_handler.h"

#include "../libs/hex_print_routines.h"

#define HOST_DEBUG  0

#define LOG_HEAD  "HOST: "

void Host::Init(HardwareSerial *serial, uint8_t prio) {
  serial->begin(115200);

  nvic_irq_set_priority(serial->c_dev()->irq_num, prio);

  serial_ = serial;

  mlock_uart_ = xSemaphoreCreateMutex();
  configASSERT(mlock_uart_);
}


/* checkout event from UART RX ring buffer
 * Note that we may call this function many times
 * for one complete event
 */
ErrCode Host::CheckoutCmd(uint8_t *cmd, uint16_t *size) {
  int       c = -1;
  int       i;
  uint16_t  timeout = 0;
  uint16_t  length = 0;
  uint16_t  calc_chk = 0;
  uint16_t  recv_chk = 0;
  ErrCode   ret = E_FAILURE;

  // no enough bytes for one command
  if (serial_->available() < COMMAND_PACKET_MIN_SIZE)
    return E_NO_RESRC;

  // ok, we have enough bytes, now find SOF
  // if no SOF in all available bytes, return with same state
  do {
    c = serial_->read();
      if (c == SOF_H) {
        if (serial_->read() == SOF_L) {
          break;
        }
        else {
          continue;
        }
      }
  } while (c != -1);

  if (c == -1) {
    // not found SOF
    return E_NO_SOF;
  }

  // if it doesn't have enough bytes in UART buffer for header, just return
  while (serial_->available() < (COMMAND_PACKET_MIN_SIZE - COMMAND_SOF_SIZE)) {
    vTaskDelay(portTICK_PERIOD_MS);
    if (++timeout > TIMEOUT_FOR_HEADER) {
      SERIAL_ECHOLN(LOG_HEAD "timeout to wait for PDU header");
      return E_NO_HEADER;
    }
  }

  // arrive here, we got SOF, and also have enough bytes for one command header
  // so just start actually check if we got correct header.
  // read all of header
  for (i = 2; i < (PDU_HEADER_SIZE); i++) {
    timeout = 0;
    do {
      c = serial_->read();
      if (c != -1)
        break;
      else {
        vTaskDelay(portTICK_PERIOD_MS);
        if (++timeout > TIMEOUT_FOR_NEXT_BYTE) {
          SERIAL_ECHOLNPAIR(LOG_HEAD "not enough bytes for header, just got ", i);
          return E_NO_HEADER;
        }
      }
    } while (1);
    pdu_header_[i] = (uint8_t)c;
  }

  // now we know the length, just check if we have correct data length
  length = (uint16_t)(pdu_header_[PDU_IDX_DATA_LEN_H]<<8 | pdu_header_[PDU_IDX_DATA_LEN_L]);
  // 1. check if we got normal length
  if (length > INVALID_DATA_LENGTH) {
    SERIAL_ECHOLNPAIR(LOG_HEAD "length out of range, recv: ", hex_word(length));
    return E_INVALID_DATA_LENGTH;
  }

  // 2. verify the length with checksum
  c = pdu_header_[PDU_IDX_DATA_LEN_H]^pdu_header_[PDU_IDX_DATA_LEN_L];
  if (pdu_header_[PDU_IDX_LEN_CHK] != (uint8_t)c) {
    SERIAL_ECHOLNPAIR(LOG_HEAD "length checksum error, recv: ", hex_byte(pdu_header_[PDU_IDX_LEN_CHK]), "calc: ", hex_byte((uint8_t)c));
    return E_INVALID_DATA_LENGTH;
  }

  // ok, got correct length, checkout the command checksum then switch state
  recv_chk = (uint16_t)(pdu_header_[PDU_IDX_CHKSUM_H]<<8 | pdu_header_[PDU_IDX_CHKSUM_L]);

  timeout = 0;
  while (serial_->available() < length) {
    vTaskDelay(DELAY_FOR_DATA);
    if (++timeout > TIMEOUT_FOR_DATA) {
      SERIAL_ECHOLNPAIR(LOG_HEAD "not enough bytes for data: ", length);
      return E_NO_DATA;
    }
  }

  // read all data field in UART RX buffer
  for (int i = 0; i < length; i++) {
    timeout = 0;
    do {
      c = serial_->read();
      if (c != -1)
        break;
      else {
        vTaskDelay(portTICK_PERIOD_MS);
        if (++timeout > TIMEOUT_FOR_NEXT_BYTE) {
          SERIAL_ECHOLN(LOG_HEAD "timeout wait for next byte of data");
          return E_NO_DATA;
        }
      }
    } while (1);
    cmd[i] = (uint8_t)c;
  }

  calc_chk = CalcChecksum(cmd, length);

  // calc checksum of data
  if (calc_chk != recv_chk) {
    SNAP_DEBUG_CMD_CHECKSUM_ERROR(true);
    SERIAL_ECHOLNPAIR(LOG_HEAD "uncorrect calc checksum: ", hex_word(calc_chk), ", recv chksum: ", hex_word(recv_chk));
    if (length > 0) {
      SERIAL_ECHO(LOG_HEAD "content:");
      for (int i = 0; i < length; i++) {
        SERIAL_ECHOPAIR(" ", hex_byte(cmd[i]));
      }
      SERIAL_EOL();
      SERIAL_EOL();
    }
    return E_INVALID_DATA;
  }

  // now we get an available command
  *size = length;

  return E_SUCCESS;
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

#if HOST_DEBUG
  SERIAL_EOL();
  SERIAL_ECHOLNPAIR(LOG_HEAD "send eid: ", hex_word(event.id), ", opc: ", hex_word(event.op_code), ", len: ", event.length);

  SERIAL_ECHO(LOG_HEAD "content:");
  for (int i = 0; i < event.length; i++) {
    SERIAL_ECHOPAIR(" ", hex_byte(event.data[i]));
  }
  SERIAL_EOL();
#endif

  /* when send out event, we will have a event structure
   * so we will have independent event_id and maybe more one op_code.
   * If yes, need to calculate them into checksum
   */
  if (event.id < INVALID_EVENT_ID) {
    if (event.op_code < INVALID_OP_CODE) {
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

#if HOST_DEBUG
  SERIAL_ECHOLNPAIR(LOG_HEAD "checnksum: ", hex_word(checksum));
  SERIAL_EOL();
#endif

  return (uint16_t)checksum;
}


/* Send event to peer host
 * here we have local buffer of PDU header for reentrant
 */
ErrCode Host::Send(Event_t &event) {
  int i = 0;
  int j;
  uint8_t pdu_header[PDU_HEADER_SIZE + 2];
  uint16_t tmp = 0;

  BaseType_t ret = pdFAIL;

  pdu_header[i++] = SOF_H;
  pdu_header[i++] = SOF_L;

  if (event.op_code < INVALID_OP_CODE)
    tmp = event.length + 2;
  else
    tmp = event.length + 1;
  pdu_header[i++] = (uint8_t)(tmp>>8);
  pdu_header[i++] = (uint8_t)(tmp & 0x00FF);

  pdu_header[i++] = PROTOCOL_SM2_VER;

  pdu_header[i++] = (uint8_t)(pdu_header[PDU_IDX_DATA_LEN_H] ^ pdu_header[PDU_IDX_DATA_LEN_L]);

  tmp = CalcChecksum(event);

  pdu_header[i++] = (uint8_t)(tmp>>8);
  pdu_header[i++] = (uint8_t)(tmp & 0x00FF);

  pdu_header[i++] = (uint8_t)event.id;

  if (event.op_code < INVALID_OP_CODE)
    pdu_header[i++] = (uint8_t)event.op_code;

  // lock the uart, there will be more than one writer
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    ret = xSemaphoreTake(mlock_uart_, configTICK_RATE_HZ/100);
    if (ret != pdPASS) {
      SERIAL_ECHOLN(LOG_HEAD "failed to get HMI uart lock!");
      return E_BUSY;
    }
  }

  for (j = 0; j < i; j++)
    serial_->write(pdu_header[j]);

  for (int j = 0; j < event.length; j++)
    serial_->write(event.data[j]);

  if (ret == pdPASS)
    xSemaphoreGive(mlock_uart_);

  return E_SUCCESS;
}


void Host::Flush() {
  serial_->flush();
}
