#pragma once

#include "../inc/MarlinConfig.h"

#ifndef _POWER_PANIC_H_
#define _POWER_PANIC_H_

typedef enum
{
	GCODE_SOURCE_UDISK = 0,
	GCODE_SOURCE_SCREEN,
	GCODE_SOURCE_PC
}GCodeSources;

#define PP_FILE_NAME_LEN  270
#define PP_FAN_COUNT      4
#define PP_HEATER         4

// delay for debounce, uint: ms, for now we use 10ms
#define POWERPANIC_DEBOUNCE	10
typedef struct
{
	//校验
	uint32_t CheckSum;
	//挤出头温度
	float HeaterTamp[PP_HEATER];
	//速度
	float PrintFeedRate;
	//空跑速度
	float TravelFeedRate;
	//热床温度
	float BedTamp;
	//坐标,  计数器反算的坐标
	float PositionData[NUM_AXIS];
	//文件位置
	int FilePosition;
	//时间记录
	uint32_t accumulator;
	//风扇速度
	uint8_t FanSpeed[PP_FAN_COUNT];
	//数据有效标志
	uint8_t Valid;
	//机器类型
	uint8_t MachineType;
	//打印数据源
	uint8_t GCodeSource;
  //激活的挤出头
  uint8_t active_extruder;
	//文件名
	char FileName[PP_FILE_NAME_LEN];
}strPowerPanicSave;


class PowerPanic
{
public:
  PowerPanic(){};
  void Init(void);
  void WriteFlash(void);
  void ClearPowerPanicData(void);
  void MaskPowerPanicData(void);
  int  SaveEnv(void);
  bool PowerPanicResumeWork(uint8_t *Err);
  void SaveCmdLine(uint32_t l);
  void TurnOffPower(void);

public:
  strPowerPanicSave Data;

private:
  uint32_t WriteIndex;
  strPowerPanicSave tmpPowerPanicData;

  int Load(void);
};

extern PowerPanic PowerPanicData;

#endif //def _STATUS_CONTROL_H_
