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


typedef struct
{
	//校验
	uint32_t CheckSum;
	//挤出头温度
	float HeaterTamp[4];
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
	uint8_t FanSpeed[4];
	//数据有效标志
	uint8_t Valid;
	//机器类型
	uint8_t MachineType;
	//打印数据源
	uint8_t GCodeSource;
	//标置Z  轴是否移动
	uint8_t ZMove;
	//文件名
	char FileName[270];
}strPowerPanicSave;


class PowerPanic
{
public:
  PowerPanic(){};
  bool Load();
  void Restore();
  void ClearPowerPanicData(void);
  void MaskPowerPanicData(void);
  bool PowerPanicResumeWork(uint8_t *Err);
  
public:
  strPowerPanicSave Data;

private:
  uint32_t WriteIndex;
  strPowerPanicSave tmpPowerPanicData;
  
};

extern PowerPanic PowerPanicData;

#endif //def _STATUS_CONTROL_H_
