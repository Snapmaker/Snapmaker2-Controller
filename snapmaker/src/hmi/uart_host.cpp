#include <libmaple/usart.h>
#include "src/core/serial.h"

#include "../common/debug.h"
#include "uart_host.h"

#define HOST_DEBUG  0

#define LOG_HEAD  "HOST: "

void UartHost::Init(HardwareSerial *serial, uint8_t interrupt_prio) {
  uint8_t *buffer;

  struct usart_dev* dev = serial->c_dev();

  buffer = (uint8_t *)pvPortMalloc(1024);
  configASSERT(buffer);

  cmd_buffer_.Init(1024, buffer);

  serial->begin(115200);

  nvic_irq_set_priority(dev->irq_num, interrupt_prio);

  serial_ = serial;

  mlock_uart_ = xSemaphoreCreateMutex();
  configASSERT(mlock_uart_);
}


/* checkout event from UART RX ring buffer
 * Note that we may call this function many times
 * for one complete event
 */
ErrCode UartHost::CheckoutCmd(uint8_t *cmd, uint16_t &length) {
  int c = -1;

  for (;;) {
    c = serial_->read();
    if (c == -1)
      break;

    cmd_buffer_.InsertOne((uint8_t)c);
  }

  return sstp_.Parse(cmd_buffer_, cmd, length);
}


void UartHost::FlushOutput() {
  serial_->flush();
}


void UartHost::FlushInput() {
  while (serial_->read() != -1);
  cmd_buffer_.Reset();
}


/* Send SSTP event to Screen
 * to save memory, we don't use Packege() of SSTP,
 * because we need to provide another buffer to save output
 */
ErrCode UartHost::Send(SSTP_Event_t &event) {
  int i = 0;
  int j;

  BaseType_t ret = pdFAIL;

  uint16_t tmp_u16 = 0;
  uint8_t  pdu_header[SSTP_PDU_HEADER_SIZE + 2];

  pdu_header[i++] = SSTP_PDU_SOF_H;
  pdu_header[i++] = SSTP_PDU_SOF_L;

  if (event.op_code < SSTP_INVALID_OP_CODE)
    tmp_u16 = event.length + 2;
  else
    tmp_u16 = event.length + 1;
  pdu_header[i++] = (uint8_t)(tmp_u16>>8);
  pdu_header[i++] = (uint8_t)(tmp_u16 & 0x00FF);

  pdu_header[i++] = SSTP_PDU_VERSION;

  pdu_header[i++] = (uint8_t)(pdu_header[SSTP_PDU_IDX_DATA_LEN_H] ^ pdu_header[SSTP_PDU_IDX_DATA_LEN_L]);

  tmp_u16 = sstp_.CalcChecksum(event);

  pdu_header[i++] = (uint8_t)(tmp_u16>>8);
  pdu_header[i++] = (uint8_t)(tmp_u16 & 0x00FF);

  pdu_header[i++] = (uint8_t)event.id;

  if (event.op_code < SSTP_INVALID_OP_CODE)
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

  for (j = 0; j < event.length; j++)
    serial_->write(event.data[j]);

  if (ret == pdPASS)
    xSemaphoreGive(mlock_uart_);

  return E_SUCCESS;
}