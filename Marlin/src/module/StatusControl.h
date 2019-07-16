#pragma once

#include "../inc/MarlinConfig.h"
#include "../snap_module/error.h"

#ifndef _STATUS_CONTROL_H_
#define _STATUS_CONTROL_H_

#define TRIGGLE_STAT_IDLE   0
#define TRIGGLE_STAT_PAUSE  1
#define TRIGGLE_STAT_STOP   2
#define TRIGGLE_STAT_RESUME 3

#define FAULT_FLAG_HEATER0	          1
#define FAULT_FLAG_BED		            (1<<1)
#define FAULT_FLAG_LOAD		            (1<<2)
#define FAULT_FLAG_FILAMENT	          (1<<3)
#define FAULT_FLAG_HEATFAIL	          (1<<4)
#define FAULT_FLAG_FILAMENT_SENSOR	  (1<<5)
#define FAULT_FLAG_POWERPANIC	        (1<<6)
#define FAULT_FLAG_LASER_EEPROM	      (1<<7)
#define FAULT_FLAG_INVALID_PPD        (1<<8)    /* invalid power panic data */

//终止触发源
typedef enum
{
	NoneStop = 0,
	//脱机打印结束停止
	EndPrint = 1,
	//人机停止
	ManualStop,
	//断电停止
	PowerPanicStop
}StopPrintType;

//暂停触发源
typedef enum
{
	NonePause = 0,
	//物料错误
	FilamentFaultPause = 1,
	//人机停止
	ManualPause,
	//门中断
	DoorOpenPause,
	//执行头丢失
	ExecuterLostPause,
	//继续
	ContinuePrint
}PausePrintType;


class StatusControl
{
public:
  StatusControl(){};
  void Init();
  uint8_t GetCurrentPrinterStatus();
  uint32_t GetSystemFault();
  void SetCurrentPrinterStatus(uint8_t newstatus);
  uint8_t GetPeriphDeviceStatus();
  void ClearSystemFaultBit(uint32_t BitsToClear);
  void SetSystemFaultBit(uint32_t BitsToClear);
  bool PauseTriggle(PausePrintType type);
  bool StopTriggle(StopPrintType type);
  void PauseProcess();
  void StopProcess();
  ErrCode PauseResume();

private:
  void InterruptAllCommand();
  void inline resume_3dp(void);
  void inline resume_cnc(void);
  void inline resume_laser(void);
  
public:
  uint8_t CurrentStatus = STAT_IDLE;
  uint32_t PeriphDeviceStatus;

private:
  uint8_t TriggleStat;
  PausePrintType PauseType;
  StopPrintType StopType;
  uint32_t FaultFlag;
};

extern StatusControl SystemStatus;

#endif //def _STATUS_CONTROL_H_
