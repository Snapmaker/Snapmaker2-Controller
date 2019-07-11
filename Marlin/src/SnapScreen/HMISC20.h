#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(HMI_SC20W)
#ifndef _HMI_SC20_H_
#define _HMI_SC20_H_

// event ID

// gcode from PC
#define EID_GCODE_REQ         1
#define EID_GCODE_RESP        2
// gcode from file
#define EID_FILE_GCODE_REQ    3
#define EID_FILE_GCODE_RESP   4
// file operation
#define EID_FILE_OP_REQ       5
#define EID_FILE_OP_RESP      6
// status query
#define EID_STATUS_REQ        7
#define EID_STATUS_RESP       8
// settings operation
#define EID_SETTING_REQ       9
#define EID_SETTING_RESP      0xa
// movement command
#define EID_MOVEMENT_REQ      0xb
#define EID_MOVEMENT_RESP     0xc
// laser&camera opration
#define EID_LAS_CAM_OP_REQ    0xd
#define EID_LAS_CAM_OP_RESP   0xe
// upgrade command
#define EID_UPGRADE_REQ       0xa9
#define EID_UPGRADE_RESP      0xaa
// Add-on operation
#define EID_ADDON_OP_REQ      0x11
#define EID_ADDON_OP_RESP     0x12


class HMI_SC20
{
public:
  HMI_SC20(){};
  void PollingCommand();
  void SendGcode(char *GCode, uint8_t EventID);
  void SendChDirResult(uint8_t Result);
  void SendMachineStatusChange(uint8_t Status, uint8_t Result);
  void SendMachineFaultFlag();
  void SendBreakPointData();
  void SendMachineStatus();
  
  void SendStartPrintReack(uint8_t Result);
  void BuffFlush(void);
  void SendHalfCalibratePoint(uint8_t Opcode, uint8_t Index);
  void SendMachineSize();
  void SendLaserFocus(uint8_t OpCode, float Height);
  void SendProgressPercent(uint8_t Percent);
  void SendPowerPanicResume(uint8_t OpCode, uint8_t Result);
  void SendFaultClearReack();
  void MovementRequestReack(uint8_t OP_ID, uint8_t Result);
  void SendUpdatePackRequest(uint16_t PackRequested);
  void SendUpdateCompleteReack(uint16_t Resultl);
  void SendStartUpdateReack(uint8_t Result);
  bool UpdateDownloadComplete(void);
  void SendUpdateComplete(uint8_t Type);
  void SendUpdateStatus(uint8_t Status);
  void UpdatePackProcess(uint8_t * pBuff, uint16_t DataLen);
  void StartUpdate(void);
  void RequestFirmwareVersion(void);
  void CheckFirmwareVersion(char *pNewVersion);

  #if ENABLED(SDSUPPORT)
   void SendCurrentUDiskPath(uint8_t Result);
   void SendInitUdisk(uint8_t Result);
   uint8_t SendDirItems(uint16_t Offset);
   void SendSpecialData();
  #endif
private:
  void HmiWriteData(char *pData, uint16_t len);
  short GetCommand(unsigned char *pBuff);
  uint8_t HalfAutoCalibrate();
  uint8_t ManualCalibrateStart();
  void ResizeMachine(char *pBuff);
  void EnterLaserFocusSetting();
  void SendWifiIP(uint8_t OpCode, uint8_t Result, char * SSID, char * PWD, char * IP);
  void PackedProtocal(char *pData, uint16_t len);
  void SendGeneralReack(uint8_t EventID, uint8_t OpCode, uint8_t Result);

public:
  uint8_t HmiRequestStatus;

private:
  uint8_t CalibrateMethod;
  uint8_t HalfAutoCalibrateState;
  uint8_t HMICommandSave;
  uint8_t ReadBuff[1024];
  uint16_t ReadTail;
  uint16_t ReadHead;
  uint32_t UpdateDataSize;
  uint8_t UpdateInProgress;
  uint16_t UpdatePackRequest;
  uint8_t CalibrateXIndeX[9]={0, 1, 2, 2, 2, 1, 0, 0, 1};
  uint8_t CalibrateXIndeY[9]={0, 0, 0, 1, 2, 2, 2, 1, 1};
  //调平点索引
  uint8_t PointIndex;
  float MeshPointZ[9];
  uint16_t ZHomeOffsetIndex;
  char BuildinWifiIP[16];
  char SSID[32];
  char Password[32];
};




#endif  //ndef _HMI_SC20_H_

#endif  //ENABLED(HMI_SC20)