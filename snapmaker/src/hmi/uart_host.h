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
  ring_buffer    *rb_;

  ProtocolSSTP sstp_;

  // lock for HMI uart
  SemaphoreHandle_t mlock_uart_ = NULL;
};

#endif  // #ifndef SNAPMAKER_UART_HOST_H_
