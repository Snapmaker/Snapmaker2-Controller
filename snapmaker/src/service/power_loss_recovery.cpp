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
#include <EEPROM.h>

#include "../common/debug.h"

#include "../module/linear.h"
#include "../module/enclosure.h"
#include "../module/toolhead_3dp.h"
#include "../module/toolhead_cnc.h"
#include "../module/toolhead_laser.h"

#include "power_loss_recovery.h"
#include "quick_stop.h"
#include "bed_level.h"
#include "system.h"

#include "src/Marlin.h"
#include "src/gcode/gcode.h"
#include "src/gcode/parser.h"
#include "src/module/configuration_store.h"
#include "src/module/printcounter.h"
#include "src/module/stepper.h"
#include "src/module/temperature.h"
#include "src/module/planner.h"
#include "src/module/motion.h"
#include "src/feature/runout.h"
#include "src/feature/bedlevel/bedlevel.h"

#include HAL_PATH(src/HAL, HAL.h)


#define FLASH_PAGE_SIZE				2048
#define RECORD_SIZE (sizeof(PowerLossRecoveryData_t) + 8)  // size of one record, prefix 8 bytes
#define FLASH_RECORD_PAGES		(MARLIN_POWERPANIC_SIZE / 2048)
#define RECORD_COUNT_PER_PAGE	(FLASH_PAGE_SIZE / (RECORD_SIZE))

#define RECORD_START_ADDR(index) ((index / RECORD_COUNT_PER_PAGE) * 2048 + (index % RECORD_COUNT_PER_PAGE) * RECORD_SIZE + FLASH_MARLIN_POWERPANIC)

PowerLossRecovery pl_recovery;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  extern float saved_g0_feedrate_mm_s;
  extern float saved_g1_feedrate_mm_s;
#endif

 /**
 * need to initialize the power detect pin
 */
void PowerLossRecovery::Init(void) {
  int ret;

  SET_INPUT(POWER_DETECT_PIN);

  if (READ(POWER_DETECT_PIN) == POWER_LOSS_STATE) {
    LOG_E("PL: power-loss signal triggerred!\n");
    enabled_ = false;
    systemservice.SetSystemFaultBit(FAULT_FLAG_POWER_DETECT_ERR);
  }

  ret = Load();

  // if data is invalid, tell others
  switch (ret)
  {
  case 0:
    // got power panic data
		if (ModuleBase::toolhead() == pre_data_.toolhead) {
			systemservice.ThrowException(EHOST_MC, ETYPE_POWER_LOSS);

			if (pre_data_.live_z_offset != 0) {
				LOG_I("PL: changed live z: %.3f\n", pre_data_.live_z_offset);
				levelservice.live_z_offset(pre_data_.live_z_offset);
				settings.save();
			}

			SERIAL_ECHOLN("PL: Got available data!");
		}
		else {
			MaskPowerPanicData();
		}
    break;
  case 1:
    // data read from flash is invalid
    SERIAL_ECHOLNPGM("PL: Unavailable data!");
    break;

  default:
    // do nothing for other results such as 2 = no power panic data
    SERIAL_ECHOLNPGM("PL: No data!");
    break;
  }
}


 /**
 *Load power panic data from flash
 *return:
 * 0 = success to get data
 * 1 = check failed
 * 2 = no data
 */
int PowerLossRecovery::Load(void)
{
  uint8_t *pSrcBuff, *pDstBuff;
	uint32_t i;
	uint32_t tmpIndex;
	uint32_t addr;
	uint32_t TotalCount;
	uint32_t Flag;
  int ret = 0;

	// size of one record, prefix 8 bytes

	// total records which can be saved
	TotalCount = RECORD_COUNT_PER_PAGE * FLASH_RECORD_PAGES;
  tmpIndex = 0;

	// find the first free block
	for (i = 0; i < TotalCount; i++) {
		// start addr for every record
		addr = RECORD_START_ADDR(i);

		// read the start flag
		Flag = *((uint32_t*)addr);

		if (Flag == 0xffffffff) {
			// tmpIndex point to a possible Non-free block, if i==0, it will ponit to the last block
			tmpIndex = (i + TotalCount - 1) % TotalCount;
			break;
		}
	}

	LOG_I("PL: first free block index: %d\n", tmpIndex);

	// try to find a non-free block
	for (i = 0; i < TotalCount; i++)
	{
		// start address of one block
		addr = RECORD_START_ADDR(tmpIndex);
		Flag = *((uint32_t*)addr);

		// if its start flag is not 0xffffffff, it is a non-free block
		if (Flag != 0xffffffff)
			break;

		// index tmpIndex forward until finding a non-free block or i reach TotalCount
		// if i reach TotalCount, it indicates all block is free
		tmpIndex = (tmpIndex + TotalCount - 1) % TotalCount;
	}

	LOG_I("PL: first non-free block index: %d\n", tmpIndex);

	// check the last value of flag
	if(Flag == 0xffffffff) {
		// arrive here we know the flash area is empty, never recording any power-loss data
		// make index to be 0
		WriteIndex = 0;
		// make flag to be invalid
		pre_data_.Valid = 0;
		// return value
		ret = 2;

		LOG_I("PL: no any data\n");
	}
	else {
		// arrive here we may have avalible power-loss data

		// check firstly whether this block is masked
		addr = RECORD_START_ADDR(tmpIndex) + 4;
		Flag = *((uint32_t*)addr);
		if(Flag != 0x5555) {
			// alright, this block has been masked by Screen
			LOG_I("PL: data has been masked\n");
			// make data to be invalid
			pre_data_.Valid = 0;
			ret = 1;
		}
		else {
			// Good, it seems we have new power-loss data, have a look at whether it is available
			// read the data to buffer
			addr = RECORD_START_ADDR(tmpIndex) + 8;
			pSrcBuff = (uint8_t*)addr;
			pDstBuff = (uint8_t*)&pre_data_;
			for (i = 0; i < sizeof(PowerLossRecoveryData_t); i++)
				*pDstBuff++ = *pSrcBuff++;

			// calculate checksum
			uint32_t Checksum;
			uint32_t tmpChecksum;
			tmpChecksum = pre_data_.CheckSum;
			pre_data_.CheckSum = 0;
			Checksum = 0;
			pSrcBuff = (uint8_t*)&pre_data_;
			for (i = 0; i < sizeof(PowerLossRecoveryData_t); i++)
				Checksum += pSrcBuff[i];
			//Checksum = Checksum - (uint8_t)(cur_data_.CheckSum >> 24) - (uint8_t)(cur_data_.CheckSum >> 16) - (uint8_t)(cur_data_.CheckSum >> 8) -
			// (uint8_t)(cur_data_.CheckSum);

			if (Checksum != tmpChecksum) {
				// shit! uncorrent checksum, flash was damaged?
				LOG_E("PL: Error checksum[0x%08x] for power-loss data, should be [0x%08x]\n", tmpChecksum, Checksum);

				// anyway, we mask this block
				FLASH_Unlock();
				addr = RECORD_START_ADDR(tmpIndex) + 4;
				FLASH_ProgramWord(addr, 0);
				FLASH_Lock();
				// make data to be invalid
				pre_data_.Valid = 0;
				ret = 1;
			}
			else {
				// correct checksum
				pre_data_.Valid = 1;
			}
		}

		// make write index to point next block
		WriteIndex = (tmpIndex + 1) % TotalCount;
		LOG_I("PL: next write index: %u\n", WriteIndex);

        // NOTE that: Used to calculate whether to clear the next pages.
        // The next page needs to be emptied in advance,
        // otherwise the empty block will not be found when the machine restarts after the last block is used.
		tmpIndex = (WriteIndex + 1) % TotalCount;
		// check if need to erase flash page
		if (((tmpIndex) % RECORD_COUNT_PER_PAGE) == 0)
		{
			// NOTE that: when WriteIndex point to the 2nd or 3rd pages firstly, this will be executed at every power-on.
			// Because its previous block is not free, that is to say, the start flag is not 0xffffffff
			// Though this may be executed many times, the flash is only erased when it is not empty.
			// So it's no need to check if we need to erase flash at every power-on. Just do it.
			FLASH_Unlock();
			addr = RECORD_START_ADDR(tmpIndex);
			FLASH_ErasePage(addr);
			FLASH_Lock();

			LOG_I("PL: erased next page, addr: 0x%X\n", addr);
		}
	}

  return ret;
}

 /**
 * save the power panic data to flash
 */
void PowerLossRecovery::WriteFlash(void)
{
  uint32_t addr;
	uint32_t u32data;
	uint8_t *pBuff;

  pBuff = (uint8_t *)&cur_data_;

	//写一半标识
	addr = RECORD_START_ADDR(WriteIndex);
	u32data = 0x5555;
	FLASH_Unlock();
	FLASH_ProgramWord(addr, u32data);

	//地址
	addr = RECORD_START_ADDR(WriteIndex) + 8;

	for(uint32_t i=0;i<sizeof(PowerLossRecoveryData_t);i=i+4)
	{
		u32data = (pBuff[i + 3] << 24) | (pBuff[i + 2] << 16) | (pBuff[i + 1] << 8) | pBuff[i];
		FLASH_ProgramWord(addr, u32data);
		addr = addr + 4;
	}

	//写完整标识
	addr = RECORD_START_ADDR(WriteIndex) + 4;
	u32data = 0x5555;
	FLASH_ProgramWord(addr, u32data);
	FLASH_Lock();
}

 /**
 *Clear all power panic data
 */
void PowerLossRecovery::ClearPowerPanicData(void)
{
  uint32_t addr;

	addr = FLASH_MARLIN_POWERPANIC;
	FLASH_Unlock();
	for(int i=0; i<FLASH_RECORD_PAGES; i++)
	{
		FLASH_ErasePage(addr);
		addr += 2048;
	}
	FLASH_Lock();
}

 /**
 *discard power panic data
 */
void PowerLossRecovery::MaskPowerPanicData(void)
{
	uint32_t TotalCount;
	uint32_t addr;
	uint16_t preIndex;

	//记录总空间
	TotalCount = RECORD_COUNT_PER_PAGE * FLASH_RECORD_PAGES;

	preIndex = (WriteIndex + TotalCount - 1) % TotalCount;

	//计算地址
	addr = RECORD_START_ADDR(preIndex) + 4;
	FLASH_Unlock();
	FLASH_ProgramWord(addr, 0);
	FLASH_Lock();
}

/*
 * save current working status to power panic data
 * return:
 *    0  - sucess
 *    -1 - failed
 */
int PowerLossRecovery::SaveEnv(void) {
  int     i = 0;
  uint8_t *pBuff;

	LOOP_XN(idx) cur_data_.position_shift[idx] = position_shift[idx];

	cur_data_.FilePosition = last_line_;

	cur_data_.axes_relative_mode = relative_mode;

	LOOP_X_TO_E(idx) cur_data_.axis_relative_modes[idx] = gcode.axis_relative_modes[idx];

  cur_data_.toolhead = ModuleBase::toolhead();

  cur_data_.PrintFeedRate = saved_g1_feedrate_mm_s;
  cur_data_.TravelFeedRate = saved_g0_feedrate_mm_s;
	cur_data_.feedrate_percentage = feedrate_percentage;

	// if live z offset was changed when working, record it
	if (levelservice.live_z_offset_updated())
		cur_data_.live_z_offset = levelservice.live_z_offset();
	else
		cur_data_.live_z_offset = 0;

  cur_data_.accumulator = print_job_timer.duration();

  // if power loss, we have record the position to cur_data_.PositionData[]
	// NOTE that we save native position for XYZ
	for (i=0; i<NUM_AXIS; i++) {
		cur_data_.PositionData[i] = current_position[i];
	}

	switch (ModuleBase::toolhead())
	{
	case MODULE_TOOLHEAD_CNC:
		cur_data_.cnc_power = cnc.power();
		break;

	case MODULE_TOOLHEAD_LASER:
	case MODULE_TOOLHEAD_LASER_10W:
		cur_data_.laser_percent = laser->power();
		cur_data_.laser_pwm = laser->tim_pwm();
	    laser->TurnOff();
	break;

  case MODULE_TOOLHEAD_3DP:
    for (i = 0; i < PP_FAN_COUNT; i++)
      cur_data_.FanSpeed[i] = printer1->fan_speed(i);
    // extruders' temperature
    HOTEND_LOOP() cur_data_.HeaterTemp[e] = thermalManager.temp_hotend[e].target;
    // heated bed
    cur_data_.BedTamp = thermalManager.temp_bed.target;
    break;

	default:
		break;
	}

#if (MOTHERBOARD == BOARD_SNAPMAKER1)
  if (cur_data_.GCodeSource == GCODE_SOURCE_UDISK) {
  }
  else {
    // 0xff will reduce the write times for the flash
    memset((void *)cur_data_.FileName, 0xFF, PP_FILE_NAME_LEN);
    cur_data_.FileName[0] = 0;
	}
#elif (MOTHERBOARD == BOARD_SNAPMAKER_2_0)
	if (systemservice.GetWorkingPort() == WORKING_PORT_SC) {
		cur_data_.GCodeSource = GCODE_SOURCE_SCREEN;
	}
	else {
		cur_data_.GCodeSource = GCODE_SOURCE_PC;
	}
#endif

  cur_data_.active_coordinate_system = gcode.active_coordinate_system;

  cur_data_.Valid = 1;

	// checksum need to be calculate at the end,
	// when all data will not be changed again
	pBuff = (uint8_t*)&cur_data_;
	cur_data_.CheckSum = 0;
	for(i = 0; i < (int)sizeof(PowerLossRecoveryData_t); i++)
		cur_data_.CheckSum += pBuff[i];

  return 0;
}

void PowerLossRecovery::Resume3DP() {
	// enable hotend
	if(pre_data_.BedTamp > BED_MAXTEMP - 15) {
		LOG_W("recorded bed temp [%f] is larger than %f, limited it.\n",
						pre_data_.BedTamp, BED_MAXTEMP - 15);
		pre_data_.BedTamp = BED_MAXTEMP - 15;
	}

	if (pre_data_.HeaterTemp[0] > HEATER_0_MAXTEMP - 15) {
		LOG_W("recorded hotend temp [%f] is larger than %f, limited it.\n",
						pre_data_.BedTamp, HEATER_0_MAXTEMP - 15);
		pre_data_.HeaterTemp[0] = HEATER_0_MAXTEMP - 15;
	}

	if (pre_data_.HeaterTemp[0] < 180)
		pre_data_.HeaterTemp[0] = 180;

	/* when recover 3DP from power-loss, maybe the filament is freezing because
	 * nozzle has been cooling. If we raise Z in this condition, the model will
	 * be pulled up, sometimes it will break the model.
	 * So we heating the hotend to 150 celsius degree before raising Z
	 */
	thermalManager.setTargetBed(pre_data_.BedTamp);
	thermalManager.setTargetHotend(pre_data_.HeaterTemp[0], 0);

  	while (thermalManager.degHotend(0) < 150) idle();

	RestoreWorkspace();

	// waiting temperature reach target
	thermalManager.wait_for_bed(true);
	thermalManager.wait_for_hotend(0, true);

  	// recover FAN speed after heating to save time
	for (int i = 0; i < PP_FAN_COUNT; i++) {
		printer1->SetFan(i, pre_data_.FanSpeed[i]);
	}

	current_position[E_AXIS] += 20;
	line_to_current_position(5);
	planner.synchronize();

	// try to cut out filament
	current_position[E_AXIS] -= 6;
	line_to_current_position(50);
	planner.synchronize();

	// E axis will be recovered in ResumeOver() using  cur_data_.PositionData[]
	// So put it in cur_data_.PositionData[] in advance
	cur_data_.PositionData[E_AXIS] = pre_data_.PositionData[E_AXIS];

	// move to target X Y
	move_to_limited_xy(pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS], 50);
	planner.synchronize();

	// move to target Z
	move_to_limited_z(pre_data_.PositionData[Z_AXIS], 30);
	planner.synchronize();
}

void PowerLossRecovery::ResumeCNC() {
	// for CNC recover form power-loss, we need to raise Z firstly.
	// because the drill bit maybe is in the workpiece
	// and we need to keep CNC motor running when raising Z
	cnc.SetOutput(pre_data_.cnc_power);

	relative_mode = true;
	process_cmd_imd("G28 Z");
	relative_mode = false;

	cnc.SetOutput(0);

	// homing and restore workspace
	RestoreWorkspace();

	// move to target X Y
	move_to_limited_xy(pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS], 50);
	planner.synchronize();

	// enable CNC motor
	cnc.SetOutput(pre_data_.cnc_power);
	LOG_I("Restore CNC power: %.2f\n", pre_data_.cnc_power);

	// move to target Z
	move_to_limited_z(pre_data_.PositionData[Z_AXIS] + 15, 30);
	move_to_limited_z(pre_data_.PositionData[Z_AXIS], 10);
	planner.synchronize();
}


void PowerLossRecovery::ResumeLaser() {
	// make sure laser is disable
	laser->TurnOff();

	// homing and restore workspace
	RestoreWorkspace();

	// move to target X Y
	move_to_limited_xy(pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS], 50);
	planner.synchronize();

	// move to target Z
	move_to_limited_z(pre_data_.PositionData[Z_AXIS], 30);
	planner.synchronize();

	// Because we open laser when receive first command after resuming,
	// and there will check if cur_data_.laser_pwm is larger than 0
	// So we recover the value to them
	cur_data_.laser_pwm = pre_data_.laser_pwm;

	// just change laser power but not enable output
	laser->SetPower(pre_data_.laser_percent);
}


void PowerLossRecovery::RestoreWorkspace() {
	feedrate_percentage = pre_data_.feedrate_percentage;

	// home first
	process_cmd_imd("G28");

	planner.synchronize();

	LOG_I("position shift:\n");
	LOG_I("X: %.2f, Y: %.2f, Z: %.2f, B: %.2f\n", pre_data_.position_shift[0], pre_data_.position_shift[1],
													pre_data_.position_shift[2],pre_data_.position_shift[3]);

	LOOP_XN(i) {
		position_shift[i] = pre_data_.position_shift[i];
		update_workspace_offset((AxisEnum)i);
	}
  gcode.active_coordinate_system = pre_data_.active_coordinate_system;
}
/**
 *Resume work after power panic if exist valid power panic data
 *return :true is resume success, or else false
 */
ErrCode PowerLossRecovery::ResumeWork() {
	if (action_ban & ACTION_BAN_NO_WORKING) {
    LOG_E("System Fault! Now cannot start working!\n");
    return E_NO_WORKING;
	}

	if (pre_data_.Valid == 0) {
		LOG_E("previous power-loss data is invalid!\n");
		return E_NO_RESRC;
	}

	LOG_I("restore point: X:%.2f, Y: %.2f, Z: %.2f, B: &.2f, E: %.2f)\n", pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS],
			pre_data_.PositionData[Z_AXIS], pre_data_.PositionData[B_AXIS], pre_data_.PositionData[E_AXIS]);

	switch (pre_data_.toolhead) {
	case MODULE_TOOLHEAD_3DP:
		if (runout.is_filament_runout()) {
			LOG_E("trigger RESTORE: failed, filament runout\n");
			systemservice.SetSystemFaultBit(FAULT_FLAG_FILAMENT);
			process_cmd_imd("G28 Z");
			return E_NO_FILAMENT;
		}

		LOG_I("previous target temp: hotend: %d, bed: %d\n", pre_data_.HeaterTemp[0], pre_data_.BedTamp);

		Resume3DP();
		break;

	case MODULE_TOOLHEAD_CNC:
		if (enclosure.DoorOpened()) {
			LOG_E("trigger RESTORE: failed, door is open\n");
			return E_DOOR_OPENED;
		}

		LOG_I("previous CNC power is %.2f\n", pre_data_.cnc_power);
		if (pre_data_.cnc_power < 50) {
			LOG_I("previous power is less than 50%, set to 50%");
			pre_data_.cnc_power = 50;
		}

		ResumeCNC();
		break;

	case MODULE_TOOLHEAD_LASER:
	case MODULE_TOOLHEAD_LASER_10W:
		if (enclosure.DoorOpened()) {
			LOG_E("trigger RESTORE: failed, door is open\n");
			return E_DOOR_OPENED;
		}

		LOG_I("previous recorded target Laser power is %.2f\n", pre_data_.laser_percent);
		LOG_I("previous recorded target laser PWM is 0x%x\n", pre_data_.laser_pwm);

		ResumeLaser();
		break;

	default:
		LOG_W("invalid machine type saved in power-loss: %d\n", pre_data_.toolhead);
		return E_FAILURE;
		break;
	}

	current_position[B_AXIS] = pre_data_.PositionData[B_AXIS];
	sync_plan_position();

	// resume stopwatch
	print_job_timer.start();
	Stopwatch::resume(pre_data_.accumulator);

	// restore speed for G0 G1
	saved_g1_feedrate_mm_s = pre_data_.PrintFeedRate;
	saved_g0_feedrate_mm_s = pre_data_.TravelFeedRate;

	LOOP_X_TO_E(idx) gcode.axis_relative_modes[idx] = pre_data_.axis_relative_modes[idx];
	relative_mode = pre_data_.axes_relative_mode;

	return E_SUCCESS;
}


/*
 * reset the power-loss data, generally called in starting work
 */
void PowerLossRecovery::Reset() {
	int i;
	int size = sizeof(PowerLossRecoveryData_t);
	char *ptr = (char *)&cur_data_;

	for (i=0; i<size; i++) {
		*ptr++ = 0;
	}
}

void PowerLossRecovery::enable(bool onoff) {
	enabled_ = onoff;
}

void PowerLossRecovery::Check(void) {
  uint8_t powerstat = READ(POWER_DETECT_PIN);

  if (!enabled_)
    return;

  // debounce for power loss, will delay 10ms for responce
  if (powerstat != POWER_LOSS_STATE) {
    last_powerloss_ = millis();
    return;
  }

  if ((millis() - last_powerloss_) >= POWERPANIC_DEBOUNCE) {
    quickstop.Trigger(QS_SOURCE_POWER_LOSS, true);
  }
}

