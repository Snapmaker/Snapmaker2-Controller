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
#include "../sd/cardreader.h"
#include "printcounter.h"
#include "StatusControl.h"
#include "stepper.h"
#include "../snap_module/snap_dbg.h"
#include "../feature/runout.h"

#include "PowerPanic.h"

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
    SystemStatus.SetSystemFaultBit(FAULT_FLAG_POWERPANIC);
    SERIAL_ECHOLNPGM("Got power panic data!");
    break;
  case 1:
    // data read from flash is invalid
    SystemStatus.SetSystemFaultBit(FAULT_FLAG_INVALID_PPD);
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

	//记录大小
	RecordSize = (sizeof(strPowerPanicSave) + 8);
	//记录总空间
	TotalCount = RECORD_COUNT_PER_PAGE * FLASH_RECORD_PAGES;
  tmpIndex = 0;

	//查找空块
	for(i=0;i<TotalCount;i++)
	{
		//计算地址
		addr = (i / RECORD_COUNT_PER_PAGE) * 2048 + (i % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC;
		//读取标置
		Flag = *((uint32_t*)addr);
		//空块
		if(Flag == 0xffffffff)
		{
			//
			tmpIndex = (i + TotalCount - 1) % TotalCount;
			break;
		}
	}

	//查找最后一个非空块
	for(i=0;i<TotalCount;i++)
	{
		//计算地址
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC;
		Flag = *((uint32_t*)addr);
		//非空块
		if(Flag != 0xffffffff)
			break;
		//
		tmpIndex = (tmpIndex + TotalCount - 1) % TotalCount;
	}

	//读取标置位
	Flag = *((uint32_t*)addr);
	//Flash  是空的，表示没有记录
	if(Flag == 0xffffffff)
	{
		//当前位置作为写入位置
		WriteIndex = 0;
		//标置无效
		pre_data_.Valid = 0;
		//标志断电错误标置
		ret = 2;
	}
	else
	{
		//地址
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 8 + FLASH_MARLIN_POWERPANIC;
		//读取数据
		pSrcBuff = (uint8_t*)addr;
		pDstBuff = (uint8_t*)&pre_data_;
		for(uint32_t i=0;i<sizeof(strPowerPanicSave);i++)
			*pDstBuff++ = *pSrcBuff++;

		//校验
		uint32_t Checksum;
		uint32_t tmpChecksum;
		tmpChecksum = pre_data_.CheckSum;
		pre_data_.CheckSum = 0;
		Checksum = 0;
		pSrcBuff = (uint8_t*)&pre_data_;
		for(uint32_t i=0;i<sizeof(strPowerPanicSave);i++)
			Checksum += pSrcBuff[i];
		//Checksum = Checksum - (uint8_t)(Data.CheckSum >> 24) - (uint8_t)(Data.CheckSum >> 16) - (uint8_t)(Data.CheckSum >> 8) -
		// (uint8_t)(Data.CheckSum);

		//校验失败
		if(Checksum != tmpChecksum)
		{
			//清除标志
			FLASH_Unlock();
			addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC;
			FLASH_ProgramWord(addr, 0);
			FLASH_Lock();
			//标置无效
			pre_data_.Valid = 0;
			//标志断电错误标置
			ret = 1;
		}
		//校验成功
		else
		{
			//标置有效
			pre_data_.Valid = 1;
		}

		//地址
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 4 + FLASH_MARLIN_POWERPANIC;
		//读取标置位
		Flag = *((uint32_t*)addr);
		//完整标置无效
		if(Flag != 0x5555)
		{
			pre_data_.Valid = 0;
			//标志断电错误标置
			ret = 1;
		}

		//下一个位置作为写入位置
		WriteIndex = (tmpIndex + 1) % TotalCount;
		//首地址
		if((WriteIndex % RECORD_COUNT_PER_PAGE) == 0)
		{
			//控除FLASH
			FLASH_Unlock();
			addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_MARLIN_POWERPANIC;
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
    Data.FanSpeed[i] = Periph.GetFanSpeed(i);

  // if power loss, we have record the position to Data.PositionData[]
	for (i=0; i<NUM_AXIS; i++) {
		Data.PositionData[i] = (i == E_AXIS) ?
				current_position[i] : NATIVE_TO_LOGICAL(current_position[i], i);
	}

#if (BOARD_VER == BOARD_SNAPMAKER1)
  if (Data.GCodeSource == GCODE_SOURCE_UDISK)
    strcpy(Data.FileName, card.filename);
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

	pBuff = (uint8_t*)&Data;
	Data.CheckSum = 0;
	for(i = 0; i < sizeof(strPowerPanicSave); i++)
		Data.CheckSum += pBuff[i];

	switch (ExecuterHead.MachineType)
	{
	case MACHINE_TYPE_CNC:
		Data.cnc_power = ExecuterHead.CNC.GetRPM();
		break;

	case MACHINE_TYPE_LASER:
		Data.laser_percent = ExecuterHead.Laser.GetPower();
		Data.laser_pwm = ExecuterHead.Laser.GetTimPwm();
	break;

	default:
		break;
	}

  return 0;
}

void PowerPanic::Resume3DP() {
	char tmpBuff[32];

	for (int i = 0; i < PP_FAN_COUNT; i++) {
		sprintf(tmpBuff, "M106 P%d S%d", i, pre_data_.FanSpeed[i]);
		process_cmd_imd(tmpBuff);
	}

	// enable hotend
	if(pre_data_.BedTamp > 130) {
		LOG_W("recorded bed temp [%f] is larger than 150, limited it.\n",
						pre_data_.BedTamp);
		pre_data_.BedTamp = 130;
	}

	if (pre_data_.HeaterTamp[0] > 285) {
		LOG_W("recorded hotend temp [%f] is larger than 285, limited it.\n",
						pre_data_.BedTamp);
		pre_data_.HeaterTamp[0] = 285;
	}

	// for now, just care 1 hotend
	sprintf(tmpBuff, "M104 S%0.2f", pre_data_.HeaterTamp[0]);
	process_cmd_imd(tmpBuff);

	// set heated bed temperature
	sprintf(tmpBuff, "M140 S%0.2f", pre_data_.BedTamp);
	process_cmd_imd(tmpBuff);

	RestoreWorkspace();

	// absolut mode
	relative_mode = false;

	// enable leveling
	// because the recorded position is logical position
	set_bed_leveling_enabled(true);

	// waiting temperature reach target
	sprintf(tmpBuff, "M190 S%0.2f", pre_data_.BedTamp);
	process_cmd_imd(tmpBuff);

	sprintf(tmpBuff, "M109 S%0.2f", pre_data_.HeaterTamp[0]);
	process_cmd_imd(tmpBuff);

	// pre-extrude
	relative_mode = true;
	process_cmd_imd("G0 E15 F100");
	planner.synchronize();

	// set E to previous position
	current_position[E_AXIS] = pre_data_.PositionData[E_AXIS];
	planner.set_e_position_mm(current_position[E_AXIS]);

	// try to cut out filament
	process_cmd_imd("G0 E-6.5 F2400");

	// absolute mode
	relative_mode = false;

	// move to target X Y
	sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS]);
	process_cmd_imd(tmpBuff);
	planner.synchronize();

	// move to target Z
	sprintf(tmpBuff, "G0 Z%0.2f F2000", pre_data_.PositionData[Z_AXIS]);
	process_cmd_imd(tmpBuff);
	planner.synchronize();

	// enable runout
	process_cmd_imd("M412 S1");
}

void PowerPanic::ResumeCNC() {
	char tmpBuff[32];

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

	LOG_I("position shift:\n");
	LOG_I("X: %f, Y: %f, Z: %f\n", pre_data_.position_shift[0],
				pre_data_.position_shift[1], pre_data_.position_shift[2]);

	// move to target X Y
	sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS]);
	process_cmd_imd(tmpBuff);
	planner.synchronize();

	// enable CNC motor
	ExecuterHead.CNC.SetPower(pre_data_.cnc_power);

	// move to target Z
	sprintf(tmpBuff, "G0 Z%0.2f F2000", pre_data_.PositionData[Z_AXIS]);
	process_cmd_imd(tmpBuff);
	planner.synchronize();
}


void PowerPanic::ResumeLaser() {
	char tmpBuff[32];

	// enable laser is disable
	ExecuterHead.Laser.SetLaserPower((uint16_t)0);

	// homing and restore workspace
	RestoreWorkspace();

	// move to target X Y
	sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", pre_data_.PositionData[X_AXIS], pre_data_.PositionData[Y_AXIS]);
	process_cmd_imd(tmpBuff);
	planner.synchronize();

	// move to target Z
	sprintf(tmpBuff, "G0 Z%0.2f F2000", pre_data_.PositionData[Z_AXIS]);
	process_cmd_imd(tmpBuff);
	planner.synchronize();
}


void PowerPanic::RestoreWorkspace() {
	// home first
	process_cmd_imd("G28");

	planner.synchronize();

	LOG_I("\nposition shift:\n");
	LOG_I("X: %.2f, Y: %.2f, Z: %.2f\n", pre_data_.position_shift[0],
						pre_data_.position_shift[1], pre_data_.position_shift[2]);

	LOOP_XYZ(i) {
		position_shift[i] = pre_data_.position_shift[i];
		update_workspace_offset((AxisEnum)i);
	}

	// TODO: whether we need to sync plan position??
}
/**
 *Resume work after power panic if exist valid power panic data
 *return :true is resume success, or else false
 */
ErrCode PowerPanic::ResumeWork() {
	if (pre_data_.MachineType != ExecuterHead.MachineType) {
		LOG_E("current[%d] machine is not same as previous[%d]\n",
						ExecuterHead.MachineType, pre_data_.MachineType);
		return E_HARDWARE;
	}

	if (pre_data_.Valid == 0) {
		LOG_E("previous power-loss data is invalid!\n");
		return E_NO_RESRC;
	}

	if (pre_data_.GCodeSource != GCODE_SOURCE_SCREEN) {
		LOG_E("previous Gcode-source is not screen: %d\n", pre_data_.GCodeSource);
		return E_INVALID_STATE;
	}

	if (Periph.IsDoorOpened()) {
		LOG_E("trigger RESTORE: failed, door is open\n");
		return E_INVALID_STATE;
	}

	LOG_I("restore point: X:%.2f, Y: %.2f, Y: %.2f, E: %.2f)\n", pre_data_.PositionData[X_AXIS],
			pre_data_.PositionData[Y_AXIS], pre_data_.PositionData[Z_AXIS], pre_data_.PositionData[E_AXIS]);
	LOG_I("line number: %d\n", pre_data_.FilePosition);

	switch (pre_data_.MachineType) {
	case MACHINE_TYPE_3DPRINT:
		if (pre_data_.HeaterTamp[0] < 185) {
			LOG_E("cannot restore work, previous recorded hotend temperature is less than 185: %f\n",
							pre_data_.HeaterTamp[0]);
			return E_INVALID_STATE;
		}
		if (runout.sensor_state()) {
			LOG_E("trigger RESTORE: failed, filament runout\n");
			return E_HARDWARE;
		}
		Resume3DP();
		break;

	case MACHINE_TYPE_CNC:
		ResumeCNC();
		break;

	case MACHINE_TYPE_LASER:
		ResumeLaser();
		break;

	default:
		LOG_W("invalid machine type saved in power-loss: %d\n", pre_data_.MachineType);
		return E_HARDWARE;
		break;
	}

  Periph.StartDoorCheck();

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

  // disable power of heated bed
  thermalManager.setTargetBed(0);

  // close laser
  if (ExecuterHead.MachineType == MACHINE_TYPE_LASER)
    ExecuterHead.Laser.SetLaserPower((uint16_t)0);

  // these 2 statement will disable power supply for
  // HMI, BED, and all addones
  WRITE(POWER0_SUPPLY_PIN, POWER0_SUPPLY_OFF);
  WRITE(POWER2_SUPPLY_PIN, POWER2_SUPPLY_OFF);
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
