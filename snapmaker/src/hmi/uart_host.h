#ifndef SNAPMAKER_UART_HOST_H_
#define SNAPMAKER_UART_HOST_H_

#include <stdio.h>

#include <HardwareSerial.h>
#include "MapleFreeRTOS1030.h"

#include "../common/error.h"
#include "../common/protocol_sstp.h"

#include "../utils/ring_buffer.h"

class UartHost {

public:
  void Init(HardwareSerial *serial, uint8_t interrupt_prio);

  ErrCode CheckoutCmd(uint8_t *cmd, uint16_t &length);

  ErrCode Send(SSTP_Event_t &e);

  void FlushOutput();
  void FlushInput();

private:
  HardwareSerial *serial_;

  ProtocolSSTP sstp_;

  RingBuffer<uint8_t> cmd_buffer_;
};

#endif  // #ifndef SNAPMAKER_UART_HOST_H_
