#include "../inc/MarlinConfig.h"
#include HAL_PATH(../HAL, HAL.h)

#include "../Marlin.h"
#include "temperature.h"
#include "planner.h"
#include "executermanager.h"
#include "motion.h"
#include "../gcode/gcode.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../module/configuration_store.h"
#include "../gcode/parser.h"
#include "../SnapScreen/Screen.h"
#include "periphdevice.h"
#include <EEPROM.h>
#include "printcounter.h"
#include "StatusControl.h"
#include "stepper.h"
#include "../snap_module/snap_dbg.h"
#include "../feature/runout.h"

#include "PowerPanic.h"
#include "ExecuterManager.h"

#define FLASH_PAGE_SIZE	2048
#define FLASH_RECORD_PAGES	(MARLIN_POWERPANIC_SIZE / 2048)
#define RECORD_COUNT_PER_PAGE	(FLASH_PAGE_SIZE / (sizeof(strPowerPanicSave) + 8))

PowerPanic powerpanic;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  extern float saved_g0_feedrate_mm_s;
  extern float saved_g1_feedrate_mm_s;
#endif

 /**
 * need to initialize the power detect pin
 */
void PowerPanic::Init(void) {
  int ret;

	SET_INPUT(POWER_DETECT_PIN);

  ret = Load();

  // if data is invalid, tell others
  switch (ret)
  {
  case 0:
    // got power panic data
		if (ExecuterHead.MachineType == pre_data_.MachineType) {
			SystemStatus.ThrowException(EHOST_MC, ETYPE_POWER_LOSS);
			SERIAL_ECHOLNPGM("Got power panic data!");
		}
		else {
			MaskPowerPanicData();
		}
    break;
  case 1:
    // data read from flash is invalid
    SERIAL_ECHOLNPGM("invalid power panic data!");
    break;

  default:
    // do nothing for other results such as 2 = no power panic data
    SERIAL_ECHOLNPGM("No power panic data!");
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
int PowerPanic::Load(void)
{
  uint8_t *pSrcBuff, *pDstBuff;
	uint32_t i;
	uint32_t tmpIndex;
	uint32_t addr;
	uint32_t RecordSize;
	uint32_t TotalCount;
	uint32_t Flag;
  int ret = 0;

	// size of one record, prefix 8 bytes
	RecordSize = (sizeof(strPowerPanicSave) + 8);

	// total records which can be saved
	TotalCount = RECORD_COUNT_PER_PAGE * FLASH_RECORD_PAGES;
  tmpIndex = 0;

	// find the first free block
	for (i = 0; i < TotalCount; i++) {
		// start addr for every record
		addr = (i / RECORD_COUNT_PER_PAGE) * 2048 + (i % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC;

		// read the start flag
		Flag = *((uint32_t*)addr);

		if (Flag == 0xffffffff) {
			// tmpIndex point to a possible Non-free block, if i==0, it will ponit to the last block
			tmpIndex = (i + TotalCount - 1) % TotalCount;
			break;
		}
	}

	// try to find a non-free block
	for (i = 0; i < TotalCount; i++)
	{
		// start address of one block
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC;
		Flag = *((uint32_t*)addr);

		// if its start flag is not 0xffffffff, it is a non-free block
		if (Flag != 0xffffffff)
			break;

		// index tmpIndex forward until finding a non-free block or i reach TotalCount
		// if i reach TotalCount, it indicates all block is free
		tmpIndex = (tmpIndex + TotalCount - 1) % TotalCount;
	}

	// check the last value of flag
	if(Flag == 0xffffffff) {
		// arrive here we know the flash area is empty, never recording any power-loss data
		// make index to be 0
		WriteIndex = 0;
		// make flag to be invalid
		pre_data_.Valid = 0;
		// return value
		ret = 2;
	}
	else {
		// arrive here we may have avalible power-loss data

		// check firstly whether this block is masked
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 4 + FLASH_MARLIN_POWERPANIC;
		Flag = *((uint32_t*)addr);
		if(Flag != 0x5555) {
			// alright, this block has been masked by Screen

			// make data to be invalid
			pre_data_.Valid = 0;
			ret = 1;
		}
		else {
			// Good, it seems we have new power-loss data, have a look at whether it is available
			// read the data to buffer
			addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 8 + FLASH_MARLIN_POWERPANIC;
			pSrcBuff = (uint8_t*)addr;
			pDstBuff = (uint8_t*)&pre_data_;
			for (i = 0; i < sizeof(strPowerPanicSave); i++)
				*pDstBuff++ = *pSrcBuff++;

			// calculate checksum
			uint32_t Checksum;
			uint32_t tmpChecksum;
			tmpChecksum = pre_data_.CheckSum;
			pre_data_.CheckSum = 0;
			Checksum = 0;
			pSrcBuff = (uint8_t*)&pre_data_;
			for (i = 0; i < sizeof(strPowerPanicSave); i++)
				Checksum += pSrcBuff[i];
			//Checksum = Checksum - (uint8_t)(Data.CheckSum >> 24) - (uint8_t)(Data.CheckSum >> 16) - (uint8_t)(Data.CheckSum >> 8) -
			// (uint8_t)(Data.CheckSum);

			if (Checksum != tmpChecksum) {
				// shit! uncorrent checksum, flash was damaged?
				LOG_E("Error checksum[0x%08x] for power-loss data, should be [0x%08x]\n", tmpChecksum, Checksum);

				// anyway, we mask this block
				FLASH_Unlock();
				addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 4 + FLASH_MARLIN_POWERPANIC;
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
		// check if need to erase flash page
		if ((WriteIndex % RECORD_COUNT_PER_PAGE) == 0)
		{
			// NOTE that: when WriteIndex point to the 2nd or 3rd pages firstly, this will be executed at every power-on.
			// Because its previous block is not free, that is to say, the start flag is not 0xffffffff
			// Though this may be executed many times, the flash is only erased when it is not empty.
			// So it's no need to check if we need to erase flash at every power-on. Just do it.
			FLASH_Unlock();
			addr = (WriteIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC;
			FLASH_ErasePage(addr);
			FLASH_Lock();
		}
	}

  return ret;
}

 /**
 * save the power panic data to flash
 */
void PowerPanic::WriteFlash(void)
{
  uint32_t addr;
	uint32_t RecordSize;
	uint32_t u32data;
	uint8_t *pBuff;

  //记录大小
  RecordSize = (sizeof(strPowerPanicSave) + 8);
  pBuff = (uint8_t *)&Data;

	//写一半标识
	addr = (WriteIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC;
	u32data = 0x5555;
	FLASH_Unlock();
	FLASH_ProgramWord(addr, u32data);

	//地址
	addr = (WriteIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 8 + FLASH_MARLIN_POWERPANIC;

	for(uint32_t i=0;i<sizeof(strPowerPanicSave);i=i+4)
	{
		u32data = (pBuff[i + 3] << 24) | (pBuff[i + 2] << 16) | (pBuff[i + 1] << 8) | pBuff[i];
		FLASH_ProgramWord(addr, u32data);
		addr = addr + 4;
	}

	//写完整标识
	addr = (WriteIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC + 4;
	u32data = 0x5555;
	FLASH_ProgramWord(addr, u32data);
	FLASH_Lock();
}

 /**
 *Clear all power panic data
 */
void PowerPanic::ClearPowerPanicData(void)
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
void PowerPanic::MaskPowerPanicData(void)
{
	uint32_t RecordSize;
	uint32_t TotalCount;
	uint32_t addr;
	uint16_t preIndex;

	//记录大小
	RecordSize = (sizeof(strPowerPanicSave) + 8);
	//记录总空间
	TotalCount = RECORD_COUNT_PER_PAGE * FLASH_RECORD_PAGES;

	preIndex = (WriteIndex + TotalCount - 1) % TotalCount;

	//计算地址
	addr = (preIndex / RECORD_COUNT_PER_PAGE) * 2048 + (preIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC + 4;
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
int PowerPanic::SaveEnv(void) {
  int     i = 0;
  uint8_t *pBuff;

  // extruders' temperature
	HOTEND_LOOP() Data.HeaterTamp[e] = thermalManager.temp_hotend[e].target;

	LOOP_XYZ(i) Data.position_shift[i] = position_shift[i];

	Data.FilePosition = last_line;

  // heated bed
  Data.BedTamp = thermalManager.temp_bed.target;

  Data.MachineType = ExecuterHead.MachineType;

  Data.PrintFeedRate = saved_g1_feedrate_mm_s;
  Data.TravelFeedRate = saved_g0_feedrate_mm_s;

  Data.accumulator = print_job_timer.duration();

  for (i = 0; i < PP_FAN_COUNT; i++)
    Data.FanSpeed[i] = ExecuterHead.GetFanSpeed(i);

  // if power loss, we have record the position to Data.PositionData[]
	// NOTE that we save logical position for XYZ
	for (i=0; i<NUM_AXIS; i++) {
		Data.PositionData[i] = (i == E_AXIS) ?
				current_position[i] : NATIVE_TO_LOGICAL(current_position[i], i);
	}

#if (BOARD_VER == BOARD_SNAPMAKER1)
  if (Data.GCodeSource == GCODE_SOURCE_UDISK) {
  }
  else {
    // 0xff will reduce the write times for the flash
    memset((void *)Data.FileName, 0xFF, PP_FILE_NAME_LEN);
    Data.FileName[0] = 0;
	}
#elif (BOARD_VER == BOARD_SNAPMAKER2_V2)
	if (SystemStatus.GetWorkingPort() == WORKING_PORT_SC) {
		Data.GCodeSource = GCODE_SOURCE_SCREEN;
	}
	else {
		Data.GCodeSource = GCODE_SOURCE_PC;
	}
#endif

  Data.Valid = 1;

	switch (ExecuterHead.MachineType)
	{
	case MACHINE_TYPE_CNC:
		Data.cnc_power = ExecuterHead.CNC.GetPower();
		break;

	case MACHINE_TYPE_LASER:
		Data.laser_percent = ExecuterHead.Laser.GetPowerPercent();
		Data.laser_pwm = ExecuterHead.Laser.GetTimPwm();
		ExecuterHead.Laser.Off();
	break;

	default:
		break;
	}
  Data.active_coordinate_system = gcode.active_coordinate_system;
	// checksum need to be calculate at the end,
	// when all data will not be changed again
	pBuff = (uint8_t*)&Data;
	Data.CheckSum = 0;
	for(i = 0; i < sizeof(strPowerPanicSave); i++)
		Data.CheckSum += pBuff[i];

  return 0;
}

void PowerPanic::Resume3DP() {

	for (int i = 0; i < PP_FAN_COUNT; i++) {
		ExecuterHead.SetFan(i, pre_data_.FanSpeed[i]);
	}

	// enable hotend
	if(pre_data_.BedTamp > BED_MAXTEMP - 15) {
		LOG_W("recorded bed temp [%f] is larger than %f, limited it.\n",
						pre_data_.BedTamp, BED_MAXTEMP - 15);
		pre_data_.BedTamp = BED_MAXTEMP - 15;
	}

	if (pre_data_.HeaterTamp[0] > HEATER_0_MAXTEMP - 15) {
		LOG_W("recorded hotend temp [%f] is larger than %f, limited it.\n",
						pre_data_.BedTamp, HEATER_0_MAXTEMP - 15);
		pre_data_.HeaterTamp[0] = HEATER_0_MAXTEMP - 15;
	}

	RestoreWorkspace();

	// for now, just care 1 hotend
	thermalManager.setTargetHotend(pre_data_.HeaterTamp[0], 0);
	thermalManager.setTargetBed(pre_data_.BedTamp);

	// waiting temperature reach target
	thermalManager.wait_for_bed(true);
	thermalManager.wait_for_hotend(true);

	// pre-extrude
	relative_mode = true;

	current_position[E_AXIS] += 30;
	line_to_current_position(10);
	planner.synchronize();

	// try to cut out filament
	current_position[E_AXIS] -= 6;
	line_to_current_position(30);

	// pre-extrude
	current_position[E_AXIS] += 6;
	line_to_current_position(10);
	planner.synchronize();

	// absolute mode
	relative_mode = false;

	// set E to previous position
	current_position[E_AXIS] = pre_data_.PositionData[E_AXIS];
	planner.set_e_position_mm(current_position[E_AXIS]);

	// move to target X Y
	do_blocking_move_to_logical_xy(pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS], 50);
	planner.synchronize();

	// move to target Z
	do_blocking_move_to_logical_z(pre_data_.PositionData[Z_AXIS], 30);
	planner.synchronize();
}

void PowerPanic::ResumeCNC() {
	// for CNC recover form power-loss, we need to raise Z firstly.
	// because the drill bit maybe is in the workpiece
	// and we need to keep CNC motor running when raising Z
	ExecuterHead.CNC.SetPower(pre_data_.cnc_power);

	relative_mode = true;
	process_cmd_imd("G28 Z");
	relative_mode = false;

	ExecuterHead.CNC.SetPower(0);

	// homing and restore workspace
	RestoreWorkspace();

	// move to target X Y
	do_blocking_move_to_logical_xy(pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS], 50);
	planner.synchronize();

	// enable CNC motor
	ExecuterHead.CNC.SetPower(pre_data_.cnc_power);
	LOG_I("Restore CNC power: %.2f\n", pre_data_.cnc_power);

	// move to target Z
	do_blocking_move_to_logical_z(pre_data_.PositionData[Z_AXIS] + 15, 30);
	do_blocking_move_to_logical_z(pre_data_.PositionData[Z_AXIS], 10);
	planner.synchronize();
}


void PowerPanic::ResumeLaser() {
	// make sure laser is disable
	ExecuterHead.Laser.Off();

	// homing and restore workspace
	RestoreWorkspace();

	// move to target X Y
	do_blocking_move_to_logical_xy(pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS], 50);
	planner.synchronize();

	// move to target Z
	do_blocking_move_to_logical_z(pre_data_.PositionData[Z_AXIS], 30);
	planner.synchronize();

	// Because we open laser when receive first command after resuming,
	// and there will check if Data.laser_pwm is larger than 0
	// So we recover the value to them
	Data.laser_pwm = pre_data_.laser_pwm;

	// just change laser power but not enable output
	ExecuterHead.Laser.ChangePower(pre_data_.laser_percent);
}


void PowerPanic::RestoreWorkspace() {
	// home first
	process_cmd_imd("G28");

	planner.synchronize();

	LOG_I("position shift:\n");
	LOG_I("X: %.2f, Y: %.2f, Z: %.2f\n", pre_data_.position_shift[0],
						pre_data_.position_shift[1], pre_data_.position_shift[2]);

	LOOP_XYZ(i) {
		position_shift[i] = pre_data_.position_shift[i];
		update_workspace_offset((AxisEnum)i);
	}
  gcode.active_coordinate_system = pre_data_.active_coordinate_system;
}
/**
 *Resume work after power panic if exist valid power panic data
 *return :true is resume success, or else false
 */
ErrCode PowerPanic::ResumeWork() {
	if (action_ban & ACTION_BAN_NO_WORKING) {
    LOG_E("System Fault! Now cannot start working!\n");
    return E_NO_WORKING;
	}

	if (pre_data_.Valid == 0) {
		LOG_E("previous power-loss data is invalid!\n");
		return E_NO_RESRC;
	}

	LOG_I("restore point: X:%.2f, Y: %.2f, Z: %.2f, E: %.2f)\n", pre_data_.PositionData[X_AXIS],
			pre_data_.PositionData[Y_AXIS], pre_data_.PositionData[Z_AXIS], pre_data_.PositionData[E_AXIS]);

	switch (pre_data_.MachineType) {
	case MACHINE_TYPE_3DPRINT:
		if (runout.is_filament_runout()) {
			LOG_E("trigger RESTORE: failed, filament runout\n");
			SystemStatus.SetSystemFaultBit(FAULT_FLAG_FILAMENT);
			process_cmd_imd("G28 Z");
			return E_NO_FILAMENT;
		}

		LOG_I("previous target temp: hotend: %d, bed: %d\n", pre_data_.HeaterTamp[0], pre_data_.BedTamp);

		Resume3DP();
		break;

	case MACHINE_TYPE_CNC:
		if (Periph.IsDoorOpened()) {
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

	case MACHINE_TYPE_LASER:
		if (Periph.IsDoorOpened()) {
			LOG_E("trigger RESTORE: failed, door is open\n");
			return E_DOOR_OPENED;
		}

		LOG_I("previous recorded target Laser power is %.2f\n", pre_data_.laser_percent);
		LOG_I("previous recorded target laser PWM is 0x%x\n", pre_data_.laser_pwm);

		ResumeLaser();
		break;

	default:
		LOG_W("invalid machine type saved in power-loss: %d\n", pre_data_.MachineType);
		return E_FAILURE;
		break;
	}

	// resume stopwatch
	print_job_timer.start();
	Stopwatch::resume(pre_data_.accumulator);

	// restore speed for G0 G1
	saved_g1_feedrate_mm_s = pre_data_.PrintFeedRate;
	saved_g0_feedrate_mm_s = pre_data_.TravelFeedRate;

	return E_SUCCESS;
}

/*
 * disable other unused peripherals in ISR
 */
void PowerPanic::TurnOffPowerISR(void) {
  // these 2 statement will disable power supply for
  // HMI, BED, and all addones except steppers
	disable_power_domain(POWER_DOMAIN_0 | POWER_DOMAIN_2);
}

/*
 * disable other unused peripherals after ISR
 */
void PowerPanic::TurnOffPower(void) {
	BreathLightClose();

	// disble timer except the stepper's
	rcc_clk_disable(TEMP_TIMER_DEV->clk_id);
	rcc_clk_disable(TIMER7->clk_id);

	// disalbe ADC
	rcc_clk_disable(ADC1->clk_id);
	rcc_clk_disable(ADC2->clk_id);

	//disble DMA
	rcc_clk_disable(DMA1->clk_id);
	rcc_clk_disable(DMA2->clk_id);

	// disable other unnecessary soc peripherals
	// disable usart
	//rcc_clk_disable(MSerial1.c_dev()->clk_id);
	//rcc_clk_disable(MSerial2.c_dev()->clk_id);
	//rcc_clk_disable(MSerial3.c_dev()->clk_id);

#if ENABLED(EXECUTER_CANBUS_SUPPORT)
  // turn off hot end and FAN
	// if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
	// 	ExecuterHead.SetTemperature(0, 0);
	// }
#endif
}

/*
 * when a block is output ended, save it's line number
 * this function is called by stepper isr()
 * when powerloss happened, no need to record line num.
 */
void PowerPanic::SaveCmdLine(uint32_t l) {
	if (l != INVALID_CMD_LINE)
		last_line = l;
}

/*
 * reset the power-loss data, generally called in starting work
 */
void PowerPanic::Reset() {
	int i;
	int size = sizeof(strPowerPanicSave);
	char *ptr = (char *)&Data;

	for (i=0; i<size; i++) {
		*ptr++ = 0;
	}
}
