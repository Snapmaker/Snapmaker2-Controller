#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)
#ifndef _CANBUS_H_
#define _CANBUS_H_

#define EXECUTER_CAN  1
#define LINEAR_MODULE_CAN 1
#define EXTERN_CAN    2
#define WHEEL_CAN     2

#define CAN_IDS_BC   0x500
#define CAN_IDS_TEMP_CONTROL  0x501
#define CAN_IDS_FAN 0x502
#define CAN_IDS_SWTICH 0x503
#define CAN_IDS_LASER 0x504
#define CAN_IDS_DCMOTOR  0x505
#define CAN_IDS_LIGHT  0x506
#define CAN_IDS_JODPANEL 0x507
#define CAN_IDS_CAMERA 0x508

typedef struct
{
  uint32_t ID;
  uint8_t FrameType;
  uint8_t Data[8];
}strCanData;

class CanBus
{
public:
  CanBus(){}
  void Init();
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t *pData, uint32_t Len);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t *pData, uint32_t Len, uint32_t *Err);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t Data0);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t Data0, uint8_t Data1);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t Data0, uint8_t Data1, uint8_t Data2);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4, uint8_t Data5);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4, uint8_t Data5, uint8_t Data6);
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4, uint8_t Data5, uint8_t Data6, uint8_t Data7);
  bool WaitReply(uint8_t CanNum, uint32_t ID, uint8_t *pData, uint32_t Len, millis_t timeout);
  void CheckReplay(uint8_t CanNum, uint32_t ID);

public:
  uint32_t TestBits;

private:
  millis_t waittick;
  uint8_t RequestReplyCAN;
  uint32_t RequestReplyID;
  bool RequestReplied;
};

extern CanBus CanBusControlor;

#endif //def _CANBUS_H_
#endif //ENABLE CANBUS_SUPPORT