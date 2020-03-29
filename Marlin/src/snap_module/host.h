#ifndef HMI_HOST_H_
#define HMI_HOST_H_

#include <stdio.h>
#include <libmaple/ring_buffer.h>

#include "error.h"
#include "snap_cmd.h"

#include "MapleFreeRTOS1030.h"


// buffer using to checkout command from UART buffer
#define HMI_RECV_BUFFER_SIZE  1024

#define COMMAND_PACKET_MIN_SIZE   9
#define COMMAND_SOF_SIZE          2

enum StateCheckoutCMD: uint8_t {
  STA_CHK_CMD_NONE = 0,
  STA_CHK_CMD_GOT_SOF,
  STA_CHK_CMD_GOT_LENGTH,
};


#define INVALID_EVENT_ID  0x100
#define INVALID_OP_CODE   0xFF

typedef struct {
  uint8_t id;
  uint8_t op_code;

  uint16_t length;

  uint8_t *data;
} Event_t;


class Host {

public:
  void Init();
  int CheckoutCmd(uint8_t *cmd);

  ErrCode Send(Event_t &e);

  /* local host is little ending, but big ending in PDU(protocol data unit)
   * so need to swap the bytes sequence when read / write PDU.
   * ToPDUBytes() & ToLocalBytes() just wrap the SwapBytesSeq()
   */
  void inline ToPDUBytes(uint8_t *dst, uint8_t *src, uint16_t size) {
    SwapBytesSeq(dst, src, size);
  }
  void inline ToLocalBytes(uint8_t *dst, uint8_t *src, uint16_t size) {
    SwapBytesSeq(dst, src, size);
  }

private:
  void inline SwapBytesSeq(uint8_t *dst, uint8_t *src, uint16_t size);

  uint16_t CalcChecksum(Event_t &e);
  uint16_t CalcChecksum(uint8_t *buffer, uint16_t size, uint16_t event_id=0xFFFF, uint16_t op_code=0xFFFF);

private:
  bool inited_ = false;

  StateCheckoutCMD      sta_checkout_cmd_ = STA_CHK_CMD_NONE;

  // lock for HMI uart
  SemaphoreHandle_t mlock_uart_ = NULL;

};

extern Host hmi;

#endif  //#ifndef HMI_HOST_H_
