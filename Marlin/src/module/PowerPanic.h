#pragma once

#include "../inc/MarlinConfig.h"
#include "../snap_module/error.h"

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
typedef struct __attribute__((aligned (4)))
{
	// checksum of this section
	uint32_t CheckSum;
	// temperature of extrucders
	int16_t HeaterTamp[PP_HEATER];
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
	int16_t BedTamp;
	// position of stepper on last move
	float PositionData[NUM_AXIS];
	// position shift between home offset and workspace offset
	float position_shift[XYZ];
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
  int8_t active_coordinate_system;
} strPowerPanicSave;


class PowerPanic
{
public:
  PowerPanic(){};
  void Init(void);
  void WriteFlash(void);
  void ClearPowerPanicData(void);
  void MaskPowerPanicData(void);
  int  SaveEnv(void);
  ErrCode ResumeWork();
  void SaveCmdLine(uint32_t l);
  void TurnOffPower(void);
	void TurnOffPowerISR(void);
	void Reset();

	uint32_t LastLine() { return last_line; }

public:
  strPowerPanicSave Data;
  strPowerPanicSave pre_data_;

private:
  uint32_t WriteIndex;
	uint32_t last_line;

  int Load(void);

	void Resume3DP();
	void ResumeCNC();
	void ResumeLaser();

	void RestoreWorkspace();
};

extern PowerPanic powerpanic;

#endif //def _STATUS_CONTROL_H_
