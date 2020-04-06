#ifndef HMI_HOST_H_
#define HMI_HOST_H_

#include <stdio.h>
#include <libmaple/ring_buffer.h>

#include "error.h"
#include "snap_cmd.h"

#include "MapleFreeRTOS1030.h"


// buffer using to checkout command from UART buffer
#define HMI_RECV_BUFFER_SIZE  1024

#define COMMAND_PACKET_MIN_SIZE   8
#define COMMAND_SOF_SIZE          2

// convert lcoal word to the bytes in protocol data unit
#define WORD_TO_PDU_BYTES(buff, var)  do { \
                                        (buff)[0] = (uint8_t)((var)>>24 & 0x000000FF);  \
                                        (buff)[1] = (uint8_t)((var)>>16 & 0x000000FF);  \
                                        (buff)[2] = (uint8_t)((var)>>8 & 0x000000FF);  \
                                        (buff)[3] = (uint8_t)((var) & 0x000000FF);  \
                                      } while (0)

// convert lcoal word to the bytes in protocol data unit, and moving index forward
#define WORD_TO_PDU_BYTES_INDEX_MOVE(buff, var, index)  do { \
                                                          (buff)[index++] = (uint8_t)((var)>>24 & 0x000000FF);  \
                                                          (buff)[index++] = (uint8_t)((var)>>16 & 0x000000FF);  \
                                                          (buff)[index++] = (uint8_t)((var)>>8 & 0x000000FF);  \
                                                          (buff)[index++] = (uint8_t)((var) & 0x000000FF);  \
                                                        } while (0)

// convert bytes in protocol unit to local word
#define PDU_TO_LOCAL_WORD(var, buff)   do { \
                                                var = (typeof(var))((buff)[0]<<24 | (buff)[1]<<16 | (buff)[2]<<8 | (buff)[3]); \
                                              } while (0)

// convert local half word to bytes in PDU
#define HWORD_TO_PDU_BYTES_INDE_MOVE(buff, var, index)  do { \
                                                    (buff)[index++] = (uint8_t)((var)>>8 & 0x00FF);  \
                                                    (buff)[index++] = (uint8_t)((var) & 0x00FF);  \
                                                  } while (0)

// convert bytes in protocol unit to local half word
#define PDU_TO_LOCAL_HALF_WORD(var, buff)  do { \
                                                    var = (typeof(var))((buff)[0]<<8 | (buff)[1]); \
                                                  } while (0)


enum StateCheckoutCMD: uint8_t {
  STA_CHK_CMD_NONE = 0,
  STA_CHK_CMD_GOT_SOF,
  STA_CHK_CMD_GOT_CHECKSUM,
};


#define INVALID_EVENT_ID  ((uint16_t)0x100)
#define INVALID_OP_CODE   ((uint16_t)0x100)
typedef struct {
  uint16_t id;
  uint16_t op_code;

  uint16_t length;

  uint8_t *data;
} Event_t;


class Host {

public:
  void Init();
  ErrCode CheckoutCmd(uint8_t *cmd, uint16_t *size);

  ErrCode Send(Event_t &e);

  /* local host is little ending, but big ending in PDU(protocol data unit)
   * so need to swap the bytes sequence when read / write PDU.
   * ToPDUBytes() & ToLocalBytes() just wrap the SwapBytesSeq()
   */
  // void inline ToPDUBytes(uint8_t *dst, uint8_t *src, uint16_t size) {
  //   SwapBytesSeq(dst, src, size);
  // }
  // void inline ToLocalBytes(uint8_t *dst, uint8_t *src, uint16_t size) {
  //   SwapBytesSeq(dst, src, size);
  // }

private:
  // void inline SwapBytesSeq(uint8_t *dst, uint8_t *src, uint16_t size) {
  //   while (--size) {
  //     *dst++ = *(src + size);
  //     if (!size)
  //       break;
  //   }
  // }

  uint16_t CalcChecksum(Event_t &e);
  uint16_t CalcChecksum(uint8_t *buffer, uint16_t size);

private:
  bool task_started_ = false;

  // parameters for checkout command
  StateCheckoutCMD  sta_checkout_cmd_ = STA_CHK_CMD_NONE;
  uint8_t  volatile timeout_ = 0;
  uint16_t volatile length_ = 0;
  uint16_t volatile checksum_ = 0;

  // lock for HMI uart
  SemaphoreHandle_t mlock_uart_ = NULL;

};

extern Host hmi;

#endif  //#ifndef HMI_HOST_H_
