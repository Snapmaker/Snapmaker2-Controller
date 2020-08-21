#include "can_channel.h"
#include "../common/config.h"

#include MARLIN_HAL(HAL_can_STM32F1.h)

CanChannel can;

ErrCode CanChannel::Init(CANIrqCallback_t irq_cb) {
  void *tmp = NULL;

  tmp = pvPortMalloc(CAN_MAC_QUEUE_SIZE * 4);
  if (!tmp) {
    return E_NO_MEM;
  }
  mac_id_.Init(CAN_MAC_QUEUE_SIZE, (uint32_t *)tmp);

  tmp = pvPortMalloc(CAN_EXT_CMD_QUEUE_SIZE);
  if (!tmp) {
    return E_NO_MEM;
  }
  ext_cmd_.Init(CAN_EXT_CMD_QUEUE_SIZE, (uint8_t *)tmp);

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
  bool       ret_send = false;

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    ret_lock = xSemaphoreTake(lock_[packet.ch], portMAX_DELAY);

  switch (packet.ft) {
  case CAN_FRAME_STD_DATA:
    if (packet.length > 8) {
      if (ret_lock == pdPASS)
        xSemaphoreGive(lock_[packet.ch]);

      return E_PARAM;
    }

    ret_send = CanSendPacked(packet.id, IDTYPE_STDID, packet.ch, FRAME_DATA, packet.length, packet.data);

    break;

  case CAN_FRAME_EXT_DATA:
    for (int32_t  i = 0; i < packet.length; i += 8) {
      if (packet.length - i > 8)
        ret_send = CanSendPacked(packet.id, IDTYPE_EXTID, packet.ch, FRAME_DATA, 8, packet.data + i);
      else
        ret_send = CanSendPacked(packet.id, IDTYPE_EXTID, packet.ch, FRAME_DATA, packet.length - i, packet.data + i);
    }
    break;

  case CAN_FRAME_EXT_REMOTE:
    ret_send = CanSendPacked(packet.id, IDTYPE_EXTID, packet.ch, FRAME_REMOTE, 0, 0);
    break;

  default:
    break;
  }

  if (ret_lock == pdPASS) {
    xSemaphoreGive(lock_[packet.ch]);
  }

  if (ret_send)
    return E_SUCCESS;
  else
    return E_FAILURE;
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
  int32_t ret;

  uint8_t *tmp_pu8;

  if (!buffer) {
    return -E_PARAM;
  }

  switch (ft) {
  case CAN_FRAME_STD_DATA:
    if (std_cmd_in_q_ == 0)
      return 0;

    tmp_pu8 = (uint8_t *)&std_cmd_[std_cmd_in_q_];

    for (i = 0; i < CAN_STD_CMD_ELEMENT_SIZE; i++) {
      buffer[i] = tmp_pu8[i];
    }

    if (++std_cmd_r_ >= CAN_STD_CMD_QUEUE_SIZE)
      std_cmd_r_ = 0;

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


int32_t CanChannel::Peek(CanFrameType ft, uint8_t *buffer, int32_t l) {
  int32_t i = 0;
  int32_t ret;

  if (!buffer) {
    return -E_PARAM;
  }

  switch (ft) {
  case CAN_FRAME_EXT_DATA:
    return ext_cmd_.PeekMulti(buffer, l);

  case CAN_FRAME_EXT_REMOTE:
    return mac_id_.PeekMulti((uint32_t *)buffer, l);

  default:
    break;
  }

  return 0;
}


void CanChannel::Irq(CanChannelNumber ch,  uint8_t fifo_index) {
  int32_t i = 0;
  bool is_remote_frame;

  uint32_t  can_id;
  uint8_t   id_type;
  uint8_t   frame_type;
  uint8_t   length;

  CanStdDataFrame_t std_data_frame;

  // read data
  switch (ch) {
  case CAN_CH_1:
    is_remote_frame = Canbus1ParseData(&can_id, &id_type, &frame_type, std_data_frame.data, &length, fifo_index);
    break;

  case CAN_CH_2:
    is_remote_frame = Canbus2ParseData(&can_id, &id_type, &frame_type, std_data_frame.data, &length, fifo_index);
    break;

  default:
    break;
  }

  // standard data frame
  if (id_type == IDTYPE_STDID && !is_remote_frame) {
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
      }
    }

    return;
  }

  // extended data frame
  if (id_type == IDTYPE_EXTID && !is_remote_frame) {
    ext_cmd_.InsertMulti(std_data_frame.data, length);
    // to tell upper level the sender, put can_id in following
    ext_cmd_.InsertMulti((uint8_t *)&can_id, 4);
    return;
  }

  // extended remote frame
  if (id_type == IDTYPE_EXTID && is_remote_frame) {
    // low 29 bits is actual module MAC
    // we add channel number in the high 2 bits, to tell upper level
    can_id |= (ch<<30);
    mac_id_.InsertOne(can_id);
    return;
  }
}


extern "C"
{

void __irq_can1_tx(void) {
}

void __irq_can1_rx0(void) {
  CanChannel::Irq(CAN_CH_1, 0);
}

void __irq_can1_rx1(void) {
  CanChannel::Irq(CAN_CH_1, 1);
}

void __irq_can1_sce(void) {
}

void __irq_can2_tx(void) {
}

void __irq_can2_rx0(void) {
  CanChannel::Irq(CAN_CH_2, 0);
}

void __irq_can2_rx1(void) {
  CanChannel::Irq(CAN_CH_2, 1);
}

void __irq_can2_sce(void) {
}

}