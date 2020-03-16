#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(HMI_SC20W)
#ifndef _HMI_SC20_H_
#define _HMI_SC20_H_

#include "Screen.h"

class HMI_SC20
{
public:
  HMI_SC20(){};
  void PollingCommand(bool nested);
  void HandleOneCommand(bool reject_sync_write);
  void SendEvent(uint8_t EventID, char *buffer, uint16_t len);
  void SendMachineStatusChange(uint8_t Status, uint8_t Result);
  void SendMachineFaultFlag(uint32_t flag = 0);
  void SendBreakPointData();
  void SendMachineStatus();
  
  void SendStartPrintReack(uint8_t Result);
  void BuffFlush(void);
  void SendHalfCalibratePoint(uint8_t Opcode, uint8_t Index);
  void SendMachineSize();
  void SendLaserFocus(uint8_t OpCode, float Height);
  void MovementRequestReack(uint8_t OP_ID, uint8_t Result);
  void SendUpdatePackRequest(uint16_t PackRequested);
  bool UpdateDownloadComplete(void);
  void SendUpdateComplete(uint8_t Type);
  void SendUpdateStatus(uint8_t Status);
  void UpdatePackProcess(uint8_t * pBuff, uint16_t DataLen);
  void StartUpdate(void);
  void RequestFirmwareVersion(void);
  void ReportModuleFirmwareVersion(uint32_t ID, char *pVersion);
  void ReportLinearLength();
  void ReportLinearLead();
  void ReportLinearModuleMacID(void);
  void CheckFirmwareVersion(char *pNewVersion);
  void PackedProtocal(char *pData, uint16_t len);
  void SetFeedrate(float f) { last_feedrate = f; }

  #if ENABLED(SDSUPPORT)
    void SendCurrentUDiskPath(uint8_t Result);
    void SendInitUdisk(uint8_t Result);
    uint8_t SendDirItems(uint16_t Offset);
    void SendSpecialData();
  #endif
private:
  void HmiWriteData(char *pData, uint16_t len);
  short GetCommand(unsigned char *pBuff);
  uint8_t HalfAutoCalibrate(bool fast_leveling);
  uint8_t ManualCalibrateStart();
  void SendWifiIP(uint8_t OpCode, uint8_t Result, char * SSID, char * PWD, char * IP);
  void SendBluetoothName(uint8_t OpCode, uint8_t Result, char * Name);
  void SendBluetoothMac(uint8_t OpCode, uint8_t Result, uint8_t * Mac);
  void SendGeneralReack(uint8_t EventID, uint8_t OpCode, uint8_t Result);
  void LaserCoarseCalibrate(float X, float Y, float Z);
  void DrawLaserCalibrateShape();
  bool DrawLaserRuler(float StartX, float StartY, float StartZ, float Z_Increase, uint8_t Count);
  void MovementProcess(float X, float Y, float Z, float speed, uint8_t Option);
  void MoveE(float extrude_len, float extrude_speed, float retract_len, float retract_speed, uint8_t Option);

public:
  HMIReq RequestStatus;

private:
  uint8_t CalibrateMethod;
  uint8_t HalfAutoCalibrateState;
  uint8_t HMICommandSave;
  uint16_t ReadTail;
  uint16_t ReadHead;
  uint32_t UpdateDataSize;
  uint8_t UpdateInProgress;
  uint16_t UpdatePackRequest;
  uint8_t CalibrateIndeX[9]={0, 1, 2, 2, 2, 1, 0, 0, 1};
  uint8_t CalibrateIndeY[9]={0, 0, 0, 1, 2, 2, 2, 1, 1};
  //调平点索引
  uint8_t PointIndex;
  float MeshPointZ[25];
  uint16_t ZHomeOffsetIndex;
  char BuildinWifiIP[16];
  char SSID[32];
  char Password[32];
  char bluetooth_name[32];
  uint8_t bluetooth_mac[6];
  uint8_t next_cmd_idx;
  bool is_handling_cmd;
  float last_feedrate;

  uint32_t current_line;
};




#endif  //ndef _HMI_SC20_H_

#endif  //ENABLED(HMI_SC20)