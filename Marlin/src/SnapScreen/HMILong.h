#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(HMI_LONG)
#ifndef _HMI_LONG_H_
#define _HMI_LONG_H_

#include "HMILongDefines.h"

#define FILE_PER_PAGE (6)

//检测指令缓冲是否为空
#define CMD_BUFF_EMPTY()	(commands_in_queue>0?false:true)
//U  盘枚举成功
//#define IS_UDISK_INSERTED DiskInserted

class HMILong
{
  public:
  HMILong(){};
  void BuffFlush(void);
  short HmiGetCommand(unsigned char *pBuff);
  void Show(void);
  void ChangePage(uint8_t Page);
  void ShowFileName(uint8_t Index, char *FileName);
  void ShowTemperature(uint8_t Index, int CurrentTemp, int TargetTemp);
  void ShowProgressBar(uint8_t Value);
  void SetControlEnable(bool Enable);
  void SetSettingEnable(bool Enable);
  void PauseEnable(bool Enable);
  void StopEnable(bool Enable);
  void SetTFIcon(bool Enable);
  void ShowConfirmFileName(char *FileName);
  void ShowPrintFileName(char *FileName);
  void ZOffsetLable(float Value);
  void ShowChangeFilamentTamp();
  void ShowLaserPower(uint8_t Power);
  void ShowPrintTime(void);
  void ChangePrinterStatus(uint8_t Status);
  void ChangeLaserCNCStatus(uint8_t Status);
  void SetAboutLabels(void);
  void PollingCommand(void);

  private:
  void HmiWriteData(char *pData, uint16_t len);
  short HmiGetCommand(void);
  public:
  uint8_t HmiRequestStatus;

  private:
  const uint32_t TemperatureLable[3] = {LABEL_TEMP_HEART0, LABEL_TEMP_BED};
  const uint32_t FileLable[6] = {LABEL_FILE1, LABEL_FILE2, LABEL_FILE3, LABEL_FILE4, LABEL_FILE5, LABEL_FILE6};
  long UIKey;
  bool StepperSync;
  uint8_t PointIndex;
  float MeshPointZ[4];

  //指令暂存缓冲，正确解析之后，指令存放在这里
  char tmpBuff[256];

  uint8_t ReadBuff[256];
  uint16_t ReadTail;
  uint16_t ReadHead;
  uint8_t CurrentHMIPage;
  uint16_t FileCounts;
  uint8_t FilePages;
  uint8_t CurFilePage;
  char CardFileNameSelected[13];
  char tmpCardFileNameSelected[13];
  char PrintFileNameSelected[13];
  uint16_t FileIndex;
  char CardFileName[FILE_PER_PAGE][80];
  char PrintFileName[FILE_PER_PAGE][13];
  //SD  插入状态
  bool SdInsert = false;
};

#endif
#endif // ENABLED(HMI_LONG)