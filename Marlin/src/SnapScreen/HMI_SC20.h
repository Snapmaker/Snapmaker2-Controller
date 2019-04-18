#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(HMI_SC20)
#ifndef _HMI_SC20_H_
#define _HMI_SC20_H_

class HMI_SC20
{
public:
  HMI_SC20(){};
  short GetCommand(unsigned char *pBuff);
  void Process();
  void SC20NopProcess(void);
  void SC20ChangePage(unsigned char page);
  void SC20SendGcode(char *GCode, uint8_t EventID);

  void SendBreakPointLine();
  void SendChDirResult(uint8_t Result);
  void SendMachineStatusChange(uint8_t Status, uint8_t Result);
  void SendMachineFaultFlag();
  void SendBreakPointData();
  void SendMachineStatus();
  void SendCurrentUDiskPath(uint8_t Result);
  void SendInitUdisk(uint8_t Result);
  uint8_t SendDirItems(uint16_t Offset);
  void SendSpecialData();
  void SendStartPrintReack(uint8_t Result);
  void BuffFlush(void);
  void SettingReack(uint8_t OP_ID, uint8_t Result);
  void SendMachineSize();
  void SendLaserFocus(uint8_t OpCode);
  void SendProgressPercent(uint8_t Percent);
  void SendPowerPanicResume(uint8_t OpCode, uint8_t Result);
  void SendFaultClearReack();
  void MovementRequestReack(uint8_t OP_ID, uint8_t Result);
  void SendUpdatePackRequest(uint16_t PackRequested);
  void SendUpdateCompleteReack(uint16_t Resultl);
  void SendStartUpdateReack(uint8_t Result);
  void UpdateComplete(void);
  void UpdatePackProcess(uint8_t * pBuff, uint16_t DataLen);
  void StartUpdate(void);

private:
  uint8_t CalibrateMethod;
  uint8_t HalfAutoCalibrateState;
  uint8_t HMICommandSave;
};




#endif  //ndef _HMI_SC20_H_

#endif  //ENABLED(HMI_SC20)