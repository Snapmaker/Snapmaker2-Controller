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
	// checksum of this section
	uint32_t CheckSum;
	// temperature of extrucders
	float HeaterTamp[PP_HEATER];
	// speed of work
	float PrintFeedRate;
	// speed of travel
	float TravelFeedRate;
	// CNC power
	float cnc_power;
	// laser Power
	float laser_percent;
	uint16_t laser_pwm;
	// target temperature of heat bed
	float BedTamp;
	// position of stepper on last move
	float PositionData[NUM_AXIS];
	// line number of last gcode
	int FilePosition;
	// 
	uint32_t accumulator;
	// fans' speed
	uint8_t FanSpeed[PP_FAN_COUNT];
	// if this section is valid
	uint8_t Valid;
	// working machineType when power-loss
	uint8_t MachineType;
	// Gcode source
	uint8_t GCodeSource;
  // active extruder
  uint8_t active_extruder;
#if (BOARD_VER == BOARD_SNAPMAKER1)
	// file name
	char FileName[PP_FILE_NAME_LEN];
#endif
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
