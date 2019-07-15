#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)
#ifndef _CANBUS_H_
#define _CANBUS_H_

#include "CanDefines.h"

class CanBus
{
public:
  CanBus(){}
  void Init();
  bool SendData(uint8_t CanNum, uint32_t ID, uint8_t *pData, int16_t Len);
  //bool SendData(uint8_t CanNum, uint32_t ID, uint8_t *pData, int16_t Len, uint32_t *Err);
  bool SendLongData(uint8_t CanNum, uint32_t ID, uint8_t *pData, int16_t Len);
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
  uint16_t ProcessLongPacks(uint8_t *pBuff, uint16_t MaxLen);

public:
  static uint32_t CurCommunicationID;
  static uint8_t ReadRingBuff[2048];
  static uint8_t ProcessBuff[524];
  static uint16_t ReadHead;
  static uint16_t ReadTail;

public:
  millis_t waittick;
  uint8_t RequestReplyCAN;
  uint32_t RequestReplyID;
  bool RequestReplied;
  uint32_t ModuleMacList[32];
  uint8_t ModuleCount;
  uint32_t ExtendModuleMacList[64];
  uint8_t ExtendModuleCount;
};

extern CanBus CanBusControlor;

#endif //def _CANBUS_H_
#endif //ENABLE CANBUS_SUPPORT