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
#include "can_channel.h"
#include "../common/config.h"
#include "../common/debug.h"

#include "src/inc/MarlinConfig.h"
#include HAL_PATH(src/HAL, HAL_can_STM32F1.h)

CanChannel can;

ErrCode CanChannel::Init(CANIrqCallback_t irq_cb) {
  void *tmp = NULL;

  tmp = pvPortMalloc(CAN_MAC_QUEUE_SIZE * 4);
  if (!tmp) {
    return E_NO_MEM;
  }
  mac_id_.Init((int32_t)CAN_MAC_QUEUE_SIZE, (uint32_t *)tmp);

  tmp = pvPortMalloc(CAN_EXT_CMD_QUEUE_SIZE);
  if (!tmp) {
    return E_NO_MEM;
  }
  ext_cmd_.Init((int32_t)CAN_EXT_CMD_QUEUE_SIZE, (uint8_t *)tmp);

  std_cmd_w_ = 0;
  std_cmd_r_ = 0;
  std_cmd_in_q_ = 0;

  for (int i = 0; i < CAN_CH_MAX; i++) {
    lock_[i] = xSemaphoreCreateMutex();
    configASSERT(lock_[i]);
  }

  irq_cb_ = irq_cb;

  CanInit();

  return E_SUCCESS;
}


ErrCode CanChannel::Write(CanPacket_t &packet) {
  BaseType_t ret_lock = pdFAIL;
  uint32_t   ret_send = 0;

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    ret_lock = xSemaphoreTake(lock_[packet.ch], portMAX_DELAY);

  switch (packet.ft) {
  case CAN_FRAME_STD_DATA:
    if (packet.length > 8) {
      if (ret_lock == pdPASS)
        xSemaphoreGive(lock_[packet.ch]);

      return E_PARAM;
    }

    ret_send = CanSendPacked(packet.id, IDTYPE_STDID, packet.ch + 1, FRAME_DATA, packet.length, packet.data);

    break;

  case CAN_FRAME_EXT_DATA:
    for (int32_t  i = 0; i < packet.length; i += 8) {
      if (packet.length - i > 8)
        ret_send = CanSendPacked(packet.id, IDTYPE_EXTID, packet.ch + 1, FRAME_DATA, 8, packet.data + i);
      else
        ret_send = CanSendPacked(packet.id, IDTYPE_EXTID, packet.ch + 1, FRAME_DATA, packet.length - i, packet.data + i);
    }
    break;

  case CAN_FRAME_EXT_REMOTE:
    ret_send = CanSendPacked(packet.id, IDTYPE_EXTID, packet.ch + 1, FRAME_REMOTE, 0, 0);
    break;

  case CAN_FRAME_STD_REMOTE:
    ret_send = CanSendPacked(packet.id, IDTYPE_STDID, packet.ch + 1, FRAME_REMOTE, 0, 0);
    break;

  default:
    break;
  }

  if (ret_lock == pdPASS) {
    xSemaphoreGive(lock_[packet.ch]);
  }

  if (!ret_send) {
    //LOG_I("[CH%u:0x%X] send ok\n", packet.ch + 1, packet.id);
    return E_SUCCESS;
  }
  else {
    //LOG_I("[CH%u:0x%X] failed to send can packet: 0x%X\n", packet.ch + 1, packet.id, ret_send);
    return E_FAILURE;
  }
}


int32_t CanChannel::Available(CanFrameType ft) {
  switch (ft) {
  case CAN_FRAME_STD_DATA:
    return CAN_STD_CMD_QUEUE_SIZE - std_cmd_in_q_;

  case CAN_FRAME_EXT_DATA:
    return ext_cmd_.Available();

  case CAN_FRAME_EXT_REMOTE:
    return mac_id_.Available();

  default:
    break;
  }

  return 0;
}


int32_t CanChannel::Read(CanFrameType ft, uint8_t *buffer, int32_t l) {
  int32_t i = 0;

  uint8_t *tmp_pu8;

  if (!buffer) {
    return -E_PARAM;
  }

  switch (ft) {
  case CAN_FRAME_STD_DATA:
    if (std_cmd_in_q_ == 0)
      return 0;

    tmp_pu8 = (uint8_t *)&std_cmd_[std_cmd_r_];

    for (i = 0; i < CAN_STD_CMD_ELEMENT_SIZE; i++) {
      buffer[i] = tmp_pu8[i];
    }

    if (++std_cmd_r_ >= CAN_STD_CMD_QUEUE_SIZE)
      std_cmd_r_ = 0;

    std_cmd_in_q_--;

    return CAN_STD_CMD_ELEMENT_SIZE;

  case CAN_FRAME_EXT_DATA:
    return ext_cmd_.RemoveMulti(buffer, l);

  case CAN_FRAME_EXT_REMOTE:
    return mac_id_.RemoveMulti((uint32_t *)buffer, l);

  default:
    break;
  }

  return 0;
}


void CanChannel::Irq(CanChannelNumber ch,  uint8_t fifo_index) {
  int32_t i = 0;
  uint8_t filter_index = 0;

  uint32_t  can_id;
  uint8_t   id_type;
  uint8_t   frame_type;
  uint8_t   length;

  CanStdDataFrame_t std_data_frame;

  // read data
  switch (ch) {
  case CAN_CH_1:
    filter_index = Canbus1ParseData(&can_id, &id_type, &frame_type, std_data_frame.data, &length, fifo_index);
    break;

  case CAN_CH_2:
    filter_index = Canbus2ParseData(&can_id, &id_type, &frame_type, std_data_frame.data, &length, fifo_index);
    break;

  default:
    return;
  }

  // standard data frame
  if (fifo_index == 0) {
    if (id_type == IDTYPE_STDID && !filter_index) {
      std_data_frame.id.val = (uint16_t)can_id;
      std_data_frame.id.bits.length = length & 0x1F;

      // check if we have callback for this message id
      // if callback return true, indicates message is handled
      if (irq_cb_ && irq_cb_(std_data_frame)) {
        return;
      }

      // if no callback, enqueue the data just received
      if (std_cmd_in_q_ < CAN_STD_CMD_QUEUE_SIZE) {
        // save data field
        for (i = 0; i < length; i++) {
          std_cmd_[std_cmd_w_] = std_data_frame;
          if (++std_cmd_w_ >= CAN_STD_CMD_QUEUE_SIZE)
            std_cmd_w_ = 0;
          std_cmd_in_q_++;
        }
      }
    }

    return;
  }

  // first bit indicates main controller or modules
  can_id &= 0xFFFFFFFE;

  // extended data frame
  if (id_type == IDTYPE_EXTID && filter_index) {
    // if (std_data_frame.data[0] == 0xAA &&
    //     std_data_frame.data[1] == 0x55) {
    //   // to tell upper level the sender, put can_id in following
    //   ext_cmd_.InsertMulti((uint8_t *)&can_id, 4);
    // }

    ext_cmd_.InsertMulti(std_data_frame.data, length);

    return;
  }

  // extended remote frame
  if (id_type == IDTYPE_EXTID && !filter_index) {
    // low 29 bits is actual module MAC
    // we add channel number in the bit[29], to tell upper level

    // bit[29] == 0 indicates CAN1
    // bit[29] == 1 indicates CAN2
    if (ch == CAN_CH_1)
      can_id &= ~(1<<29);
    else
      can_id |= (1<<29);

    mac_id_.InsertOne(can_id);
    return;
  }
}


extern "C"
{

void __irq_can1_tx(void) {
}

void __irq_can1_rx0(void) {
  can.Irq(CAN_CH_1, 0);
}

void __irq_can1_rx1(void) {
  can.Irq(CAN_CH_1, 1);
}

void __irq_can1_sce(void) {
}

void __irq_can2_tx(void) {
}

void __irq_can2_rx0(void) {
  can.Irq(CAN_CH_2, 0);
}

void __irq_can2_rx1(void) {
  can.Irq(CAN_CH_2, 1);
}

void __irq_can2_sce(void) {
}

}