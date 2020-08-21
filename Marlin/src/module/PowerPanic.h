#pragma once

#include "../inc/MarlinConfig.h"
#include "../snap_module/error.h"
#include "../snap_module/quickstop_service.h"

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
#define POWERPANIC_DEBOUNCE	6
typedef struct __attribute__((aligned (4)))
{
	// checksum of this section
	uint32_t CheckSum;
	// temperature of extrucders
	int16_t HeaterTemp[PP_HEATER];
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

	bool axis_relative_modes[XYZE];
	bool axes_relative_mode;

	int16_t feedrate_percentage;
	float   live_z_offset;
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

  /*
  * when a block is output ended, save it's line number
  * this function is called by stepper isr()
  * when powerloss happened, no need to record line num.
  */
  void FORCE_INLINE SaveCmdLine(uint32_t l) {
	if (l != INVALID_CMD_LINE)
		last_line_ = l;
  }

  void TurnOffPower(QuickStopState sta);
	void Reset(void);
	void Check(void);

	uint32_t LastLine() { return last_line_; }

public:
  strPowerPanicSave Data;
  strPowerPanicSave pre_data_;

private:
  uint32_t WriteIndex;
  uint32_t last_line_;
	millis_t last_powerloss_;

  bool enabled_;

  int Load(void);

	void Resume3DP();
	void ResumeCNC();
	void ResumeLaser();

	void RestoreWorkspace();
};

extern PowerPanic powerpanic;

#endif //def _STATUS_CONTROL_H_
