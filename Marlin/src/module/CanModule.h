#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)
#ifndef _CANMODULE_H_
#define _CANMODULE_H_

class CanModule
{
public:
  CanModule(){}
  void Init(void);
  void CollectPlugModules(void);
  void PrepareLinearModules(void);
  void PrepareExecuterModules(void);
  bool Update(uint8_t CanNum, uint32_t ID, char *Version);
  void EraseUpdatePack(void);
  bool LoadUpdatePack(uint16_t Packindex, uint8_t *pData);
  bool LoadUpdateInfo(char *Version, uint16_t *StartID, uint16_t *EndID, uint32_t *Flag);
  bool UpdateModule(uint8_t CanNum, uint32_t ID, char *Version, uint32_t Flag);
  void UpdateProcess(void);
  uint8_t GetUpdateStatus() {return UpdateStatus;}
  
  int UpdateEndstops(uint8_t *pBuff);
  int UpdateTemperature(uint8_t *pBuff);
  int UpdateCNCRPM(uint8_t *pBuff);
  int SetFunctionValue(uint8_t CanNum, uint16_t FuncID, uint8_t *pBuff);
  int SetFunctionValue(uint8_t CanNum, uint16_t FuncID, uint8_t *pBuff, uint8_t Len);
public:
  uint32_t ExecuterID[6];
  uint32_t LinearModuleID[9];
  uint16_t LinearModuleMsgID[9];
  uint16_t LinearModuleAxis[9];
  uint16_t LinearModuleLength[9];
  uint8_t ExecuterCount;
  uint8_t LinearModuleCount;
  uint32_t Endstop;
  uint32_t tmpEndstopBits;
  uint8_t UpdateStatus;
  uint16_t MsgIDTable[512];

private:
  uint32_t MacIDofFuncID[128];
  uint16_t FuncIDList[128];
  uint16_t FuncIDPriority[128];
  uint16_t MsgIDCount;
  uint8_t SendBuff[256];
  uint8_t RecvBuff[256];
};

extern CanModule CanModules;

#endif //def _CANMODULE_H_
#endif //ENABLE CANBUS_SUPPORT