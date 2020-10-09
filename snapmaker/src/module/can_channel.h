#ifndef SNAPMAKER_CAN_CHANNEL_H_
#define SNAPMAKER_CAN_CHANNEL_H_

#include "MapleFreeRTOS1030.h"

#include "../common/error.h"
#include "../utils/ring_buffer.h"

#define CAN_MAC_QUEUE_SIZE        16

#define CAN_STD_CMD_QUEUE_SIZE    10
#define CAN_STD_CMD_ELEMENT_SIZE  10

#define CAN_EXT_CMD_QUEUE_SIZE    1024


enum CanFrameType {
  CAN_FRAME_STD,
  CAN_FRAME_EXT,

  CAN_FRAME_STD_DATA,
  CAN_FRAME_EXT_DATA,
  CAN_FRAME_EXT_REMOTE,

  CAN_FRAME_INVALID
};


enum CanChannelNumber {
  CAN_CH_1,
  CAN_CH_2,

  CAN_CH_MAX
};

typedef struct CanPacket {
  CanChannelNumber  ch;
  CanFrameType      ft;
  uint32_t          id;

  uint16_t          length;
  uint8_t           *data;
} CanPacket_t;


typedef struct CanStdDataFrame {
  union {
    struct {
      uint16_t  msg_id:   9;  // message id
      uint16_t  reserved: 1;  // reserved, main controller fill 0, module fill 1
      uint16_t  type:     1;  // indicates type of this frame, 0 = request, 1 = ack
      uint16_t  length:   5;  // indicates length of data field: 1 - 8
    } bits;

    uint16_t val;
  } id;               // ID field of CAN standard data frame

  uint8_t   data[8];  // data filed of CAN standard data frame
} CanStdDataFrame_t;


typedef bool (*CANIrqCallback_t)(CanStdDataFrame_t &cmd);

class CanChannel {
  public:
    ErrCode Init(CANIrqCallback_t irq_cb);

    ErrCode Write(CanPacket_t &packet);

    int32_t Read(CanFrameType ft, uint8_t *pdu, int32_t l);
    int32_t Peek(CanFrameType ft, uint8_t *pdu, int32_t l);

    int32_t Available(CanFrameType ft);

    void Irq(CanChannelNumber ch, uint8_t fifo_index);

    RingBuffer<uint8_t> &ext_cmd() { return ext_cmd_; }

  private:
    RingBuffer<uint32_t> mac_id_;
    RingBuffer<uint8_t> ext_cmd_;

    CanStdDataFrame_t std_cmd_[CAN_STD_CMD_QUEUE_SIZE];
    uint8_t std_cmd_r_;
    uint8_t std_cmd_w_;
    uint8_t std_cmd_in_q_;

    CANIrqCallback_t irq_cb_;

    SemaphoreHandle_t lock_[CAN_CH_MAX];
};

extern CanChannel can;

#endif  // #define CAN_CHANNEL_H_
