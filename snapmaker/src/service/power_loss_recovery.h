/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SNAPMAKER_POWER_LOSS_RECOVERY_H_
#define SNAPMAKER_POWER_LOSS_RECOVERY_H_

#include "../common/config.h"
#include "../common/error.h"

#include "quick_stop.h"

typedef enum {
	GCODE_SOURCE_UDISK = 0,
	GCODE_SOURCE_SCREEN,
	GCODE_SOURCE_PC
} GCodeSources;

#define PP_FILE_NAME_LEN  270
#define PP_FAN_COUNT      2
#define PP_HEATER         1

// delay for debounce, uint: ms, for now we use 10ms
#define POWERPANIC_DEBOUNCE	10
typedef struct __attribute__((aligned (4))) {
	// checksum of this section
	uint32_t CheckSum;
	// temperature of extrucders
	int16_t HeaterTemp[PP_HEATER];
	// speed of work
	float PrintFeedRate;
	// speed of travel
	float TravelFeedRate;
	// CNC power
	uint8_t cnc_power;
	// laser Power
	float laser_percent;
	uint16_t laser_pwm;
	// target temperature of heat bed
	int16_t BedTamp;
	// position of stepper on last move
	float PositionData[NUM_AXIS];
	// position shift between home offset and workspace offset
	float position_shift[XN];
	// line number of last gcode
	int FilePosition;
	//
	uint32_t accumulator;
	// fans' speed
	uint8_t FanSpeed[PP_FAN_COUNT];
	// if this section is valid
	uint8_t Valid;
	// working toolead when power-loss
	uint8_t toolhead;
	// Gcode source
	uint8_t GCodeSource;
  // active extruder
  uint8_t active_extruder;
#if (BOARD_VER == BOARD_SNAPMAKER1)
	// file name
	char FileName[PP_FILE_NAME_LEN];
#endif
  int8_t active_coordinate_system;

	bool axis_relative_modes[X_TO_E];
	bool axes_relative_mode;

	int16_t feedrate_percentage;
	float   live_z_offset;
} PowerLossRecoveryData_t;


class PowerLossRecovery {
	public:
    PowerLossRecovery(){};
    void Init(void);
    void WriteFlash(void);
    void ClearPowerPanicData(void);
    void MaskPowerPanicData(void);
    int  SaveEnv(void);
    ErrCode ResumeWork();
	void enable(bool onoff);
	bool enable() {return enabled_;}
    /*
    * when a block is output ended, save it's line number
    * this function is called by stepper isr()
    * when powerloss happened, no need to record line num.
    */
    void FORCE_INLINE SaveCmdLine(uint32_t l) {
      if (l != INVALID_CMD_LINE)
        last_line_ = l;
    }

	void Reset(void);
	void Check(void);

      uint32_t LastLine() { return last_line_; }

	public:
	PowerLossRecoveryData_t cur_data_;
	PowerLossRecoveryData_t pre_data_;

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

extern PowerLossRecovery pl_recovery;

#endif //def _STATUS_CONTROL_H_
