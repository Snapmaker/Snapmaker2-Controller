#ifndef SNAPMAKER_UART_HOST_H_
#define SNAPMAKER_UART_HOST_H_

#include <stdio.h>
#include <libmaple/ring_buffer.h>
#include "MapleFreeRTOS1030.h"

#include "error.h"
#include "protocol_sstp.h"

#include "../utils/ring_buffer.h"

class HardwareSerial;

class UartHost {

public:
  void Init(HardwareSerial *serial, uint8_t interrupt_prio);

  ErrCode CheckoutCmd(uint8_t *cmd, uint16_t &length);

  ErrCode Send(SSTP_Event_t &e);

  void Flush();

private:

private:
  HardwareSerial *serial_;

  ProtocolSSTP sstp_;

  RingBuffer<uint8_t> cmd_buffer_;

  // lock for HMI uart
  SemaphoreHandle_t mlock_uart_ = NULL;
};

#endif  // #ifndef SNAPMAKER_UART_HOST_H_
