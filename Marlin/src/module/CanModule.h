#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)
#ifndef _CANMODULE_H_
#define _CANMODULE_H_

#define MAX_CAN_AXES  9

class CanModule
{
public:
  CanModule(){}
  void Init(void);
  void CollectPlugModules(void);
  void PrepareLinearModules(void);
  // void PrepareExecuterModules(void);
  // void PrepareExtendModules(void);
  void PrepareRestModules(void);
  bool Update(uint8_t CanNum, uint32_t ID, char *Version);
  void EraseUpdatePack(void);
  bool LoadUpdatePack(uint16_t Packindex, uint8_t *pData);
  bool LoadUpdateInfo(char *Version, uint16_t *StartID, uint16_t *EndID, uint32_t *Flag);
  bool UpdateModule(uint8_t CanNum, uint32_t ID, char *Version, uint32_t Flag);
  void UpdateProcess(void);
  uint8_t GetUpdateStatus() {return UpdateStatus;}
  bool GetFirmwareVersion(uint8_t CanNum, uint32 MacID, char* pVersion);
  void EnumFirmwareVersion(bool ReportToScreen, bool ReportToPC);
  
  int UpdateEndstops(uint8_t *pBuff);
  int UpdateTemperature(uint8_t *pBuff);
  int UpdateCNCRPM(uint8_t *pBuff);
  int SetFunctionValue(uint8_t CanNum, uint16_t FuncID, uint8_t *pBuff);
  int SetFunctionValue(uint8_t CanNum, uint16_t FuncID, uint8_t *pBuff, uint8_t Len);
  uint16_t SearchModule(uint16_t ModuleTypeID);
  void GetAxesLength();
  void GetAxesLead();
  bool SetAxesLength(uint32_t ID, uint16_t Length);
  bool SetAxesLead(uint32_t ID, float Lead);
  uint16_t GetLinearModuleLength( uint8_t Index ) { return LinearModuleLength[Index]; }
  float GetLinearModuleLead( uint8_t Index ) { return LinearModuleT[Index]; }
  bool SetMacID(uint32_t OldMacID, uint32_t NewMacID);
  static uint8_t GetMachineSizeType() { return machine_size_type; };

  void UpdateEndstops();
public:
  uint32_t ExecuterID[6];
  uint32_t LinearModuleID[MAX_CAN_AXES];
  uint8_t ExecuterCount;
  uint8_t LinearModuleCount;
  uint32_t Endstop;
  uint32_t PeriphSwitch;
  uint32_t tmpEndstopBits;
  uint8_t UpdateStatus;
  uint16_t MsgIDTable[512];

private:
  uint32_t MacIDofFuncID[256];
  uint16_t FuncIDList[256];
  uint16_t FuncIDPriority[256];
  uint16_t MsgIDCount;
  
  uint32_t MacIDofFuncID_CAN2[128];
  uint16_t FuncIDList_CAN2[128];
  uint16_t FuncIDPriority_CAN2[128];
  uint16_t MsgIDCount_CAN2;
  uint32_t MacIDofFuncID_CAN1[128];
  uint16_t FuncIDList_CAN1[128];
  uint16_t FuncIDPriority_CAN1[128];
  uint16_t MsgIDCount_CAN1;
  uint16_t LinearModuleLength[MAX_CAN_AXES];
  float LinearModuleT[MAX_CAN_AXES];
  uint8_t LinearModuleMark[MAX_CAN_AXES];
  uint8_t SendBuff[256];
  uint8_t RecvBuff[256];
  static uint8_t machine_size_type;
};

extern CanModule CanModules;

#endif //def _CANMODULE_H_
#endif //ENABLE CANBUS_SUPPORT