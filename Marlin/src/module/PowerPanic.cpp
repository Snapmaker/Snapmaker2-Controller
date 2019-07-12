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

#include "PowerPanic.h"

#define FLASH_PAGE_SIZE	2048
#define FLASH_RECORD_PAGES	(MARLIN_POWERPANIC_SIZE / 2048)
#define RECORD_COUNT_PER_PAGE	(FLASH_PAGE_SIZE / (sizeof(strPowerPanicSave) + 8))

PowerPanic PowerPanicData;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  extern float saved_g0_feedrate_mm_s;
  extern float saved_g1_feedrate_mm_s;
#endif

#if USE_EXECUTE_COMMANDS_IMMEDIATE
#define process_cmd_imd(str) do{parser.parse(str), _
                                gcode.process_parsed_command( _
                                false \
                                );}while(0)
#else
#define process_cmd_imd(str) do{parser.parse(str), gcode.process_parsed_command( );}while(0)
#endif

 /**
 * need to initialize the power detect pin
 */
void PowerPanic::init(void) {
  int ret;

  restoring = false;
  powerloss = false;
	SET_INPUT(POWER_DETECT_PIN);

  ret = Load();

  // if data is invalid, tell others
  switch (ret)
  {
  case 0:
    // got power panic data
    SystemStatus.SetSystemFaultBit(FAULT_FLAG_POWERPANIC);
    break;
  case 1:
    // data read from flash is invalid
    SystemStatus.SetSystemFaultBit(FAULT_FLAG_INVALID_PPD);
    break;

  default:
    // do nothing for other results such as 2 = no power panic data
    break;
  }

}

 /**
 * check the state of power detect pin
 * if power loss and saved the data successfully, return true, otherwise return false.
 */
bool PowerPanic::check(block_t *blk) {
	if (READ(POWER_DETECT_PIN) != POWER_LOSS_STATE)
		return false;


  powerloss = true;

  turnoffPower();

  /* when we are reading data from flash or resume work, restoring = true
   * for this condition, we just disable power and move to stop point
   */
  if (!restoring)
    return false;

  /* see if there is a block is handling
   * if yes, its file position is we needed
   * if no, need to see if command queue have any command
   */
  if (blk)
    Data.FilePosition = blk->filePos;


  /* if power-loss happened, need to disable all interrupts,
   * then do some critical actions to save data, and then enable
   * ISR to move executer to stop
   */
  DISABLE_ISRS();

  // stop stepper, clean buffer and queue
	stopWorking();

  // for now, we only handle power-loss in follow two conditions
  if (SystemStatus.CurrentStatus != STAT_PAUSE ||
        SystemStatus.CurrentStatus != STAT_RUNNING) {
    ENABLE_ISRS();
    return false;
  }

  // save information to buffer
	saveWork();

  // write buffer to flash
  save();

  // enable interrupt
  ENABLE_ISRS();

  return true;
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

  restoring = true;

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
		Data.Valid = 0;
		//标志断电错误标置
		ret = 2;
	}
	else
	{
		//地址
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 8 + FLASH_MARLIN_POWERPANIC;
		//读取数据
		pSrcBuff = (uint8_t*)addr;
		pDstBuff = (uint8_t*)&Data;
		for(uint32_t i=0;i<sizeof(strPowerPanicSave);i++)
			*pDstBuff++ = *pSrcBuff++;

		//校验
		uint32_t Checksum;
		uint32_t tmpChecksum;
		tmpChecksum = Data.CheckSum;
		Data.CheckSum = 0;
		Checksum = 0;
		pSrcBuff = (uint8_t*)&Data;
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
			Data.Valid = 0;
			//标志断电错误标置
			ret = 1;
		}
		//校验成功
		else
		{
			//标置有效
			Data.Valid = 1;
		}

		//地址
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 4 + FLASH_MARLIN_POWERPANIC;
		//读取标置位
		Flag = *((uint32_t*)addr);
		//完整标置无效
		if(Flag != 0x5555)
		{
			Data.Valid = 0;
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

  restoring = false;

  return ret;
}

 /**
 * save the power panic data to flash
 */
void PowerPanic::save(void)
{
  uint32_t addr;
	uint32_t RecordSize;
	uint32_t checksum;
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
	for(int i=0;i<FLASH_RECORD_PAGES;i++)
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
int PowerPanic::saveWork(void) {
  int     i = 0;
  uint8_t *pBuff;
  if (restoring)
    return -1;

  // extruders' temperature
	HOTEND_LOOP() Data.HeaterTamp[e] = thermalManager.temp_hotend[e].target;

  // heated bed
  Data.BedTamp = thermalManager.temp_bed.target;

  Data.MachineType = ExecuterHead.MachineType;

  Data.PrintFeedRate = saved_g1_feedrate_mm_s;
  Data.TravelFeedRate = saved_g0_feedrate_mm_s;

  Data.accumulator = print_job_timer.duration();

  for (i = 0; i < PP_FAN_COUNT; i++)
    Data.FanSpeed[i] = Periph.GetFanSpeed(i);

  for (i = 0; i < NUM_AXIS; i++)
    Data.PositionData[i] = current_position[i];

  if (Data.GCodeSource == GCODE_SOURCE_SCREEN)
    strcpy(Data.FileName, card.filename);
  else {
    // 0xff will reduce the write times for the flash
    memset((void *)Data.FileName, 0xFF, PP_FILE_NAME_LEN);
    PowerPanicData.Data.FileName[0] = 0;
  }

  Data.Valid = 1;

	pBuff = (uint8_t*)&Data;
	Data.CheckSum = 0;
	for(i = 0; i < sizeof(strPowerPanicSave); i++)
		Data.CheckSum += pBuff[i];

  return 0;
}


// for snapmaker2, we don't have Udisk on MCU
#if 1
 /**
 *Resume work after power panic if exist valid power panic data
 *return :true is resume success, or else false
 */
bool PowerPanic::PowerPanicResumeWork(uint8_t *Err)
{
  restoring = true;

	char tmpBuff[32];

	//暂存文件位置，运动会更新PowerPanicData  中的数据
	tmpPowerPanicData = Data;
  Data.FilePosition = 0;
  Data.Valid = 0;

	//数据有效
	if(tmpPowerPanicData.Valid != 0)
	{
		//power loss when printing from screen
		if(tmpPowerPanicData.GCodeSource == GCODE_SOURCE_SCREEN)
		{
      //发送启动成功
      //HMI_SendPowerPanicResume(0x0c, 0);
      //风扇开启
      for (int i = 0; i < PP_FAN_COUNT; i++) {
        sprintf(tmpBuff, "M106 P%d S%d", i, tmpPowerPanicData.FanSpeed[i]);
        process_cmd_imd(tmpBuff);
      }

      //加热
      if(tmpPowerPanicData.BedTamp > 20)
      {
        sprintf(tmpBuff, "M140 S%0.2f", tmpPowerPanicData.BedTamp);
        process_cmd_imd(tmpBuff);
      }

      //目前只考虑加热头1
      sprintf(tmpBuff, "M104 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
      process_cmd_imd(tmpBuff);

      //X  Y  回原点
      strcpy(tmpBuff, "G28");
      process_cmd_imd(tmpBuff);

      //等待加热
      if(tmpPowerPanicData.BedTamp > 20)
      {
        sprintf(tmpBuff, "M190 S%0.2f", tmpPowerPanicData.BedTamp);
        process_cmd_imd(tmpBuff);
      }

      //目前只考虑加热头1
      sprintf(tmpBuff, "M109 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
      process_cmd_imd(tmpBuff);

      //绝对模式
      relative_mode = false;

      //移动到恢复点
      //调平失效
      set_bed_leveling_enabled(false);
      //移动到续点高度加5mm  位置
      sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS] + 5);
      process_cmd_imd(tmpBuff);
      planner.synchronize();

      //预挤出
      //相对模式
      relative_mode = true;
      strcpy(tmpBuff, "G0 E25 F100");
      process_cmd_imd(tmpBuff);
      planner.synchronize();
      //坐标更新
      current_position[E_AXIS] = tmpPowerPanicData.PositionData[E_AXIS];
      planner.set_e_position_mm(current_position[E_AXIS]);
      //绝对模式
      relative_mode = false;
      //暂停3  秒
      strcpy(tmpBuff, "G4 S3");
      process_cmd_imd(tmpBuff);
      //移动到续点XY  坐标
      sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", tmpPowerPanicData.PositionData[X_AXIS], tmpPowerPanicData.PositionData[Y_AXIS]);
      process_cmd_imd(tmpBuff);
      planner.synchronize();
      //移动到打印高度
      sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS]);
      process_cmd_imd(tmpBuff);
      planner.synchronize();
      //计时器开始工作
      print_job_timer.start();
      Stopwatch::resume(tmpPowerPanicData.accumulator);

      //绝对模式
      relative_mode = false;
      //同步坐标计数器
      //Sync_PCounter();

      //速度还原
      saved_g1_feedrate_mm_s = Data.PrintFeedRate;
      saved_g0_feedrate_mm_s = Data.TravelFeedRate;

      //调平使能
      set_bed_leveling_enabled(true);

      *Err = 0;
      SystemStatus.SetCurrentPrinterStatus(STAT_RUNNING);
      //EnablePowerPanicCheck();
      if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
        Periph.StartDoorCheck();
      #if((HAVE_FILAMENT_SENSOR == 1) || (HAVE_FILAMENT_SWITCH == 1))
      if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
        Periph.StartFilamentCheck();
      #endif
      //清除断电数据
      restoring = false;
      MaskPowerPanicData();
      return 0;
		}
		//联机打印断电
		else
		{
			//发送启动成功
			//HMI_SendPowerPanicResume(0x0b, 0);
			//风扇开启
			#ifdef H_FAN2_PORT
			H_FAN2_PORT->BSRR = H_FAN2_PIN;
			#endif
			//机型是3D  打印
			if(MACHINE_TYPE_3DPRINT == tmpPowerPanicData.MachineType)
			{
				//加热
				if(tmpPowerPanicData.BedTamp > 20)
				{
					sprintf(tmpBuff, "M140 S%0.2f", tmpPowerPanicData.BedTamp);
					process_cmd_imd(tmpBuff);
				}

				//目前只考虑加热头1
				sprintf(tmpBuff, "M104 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
				process_cmd_imd(tmpBuff);
			}

			//X  Y  回原点
			strcpy(tmpBuff, "G28");
			process_cmd_imd(tmpBuff);

			//机型是3D  打印
			if(MACHINE_TYPE_3DPRINT == tmpPowerPanicData.MachineType)
			{
				//等待加热
				if(tmpPowerPanicData.BedTamp > 20)
				{
					sprintf(tmpBuff, "M190 S%0.2f", tmpPowerPanicData.BedTamp);
					process_cmd_imd(tmpBuff);
				}

				//目前只考虑加热头1
				sprintf(tmpBuff, "M109 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
				process_cmd_imd(tmpBuff);
			}

			//绝对模式
			relative_mode = false;

			//移动到恢复点
			//调平失效
			set_bed_leveling_enabled(false);
			//移动到续点高度加5mm  位置
			sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS] + 5);
			process_cmd_imd(tmpBuff);
			planner.synchronize();
			//机型是3D  打印
			if(MACHINE_TYPE_3DPRINT == tmpPowerPanicData.MachineType)
			{
				//预挤出
				//相对模式
				relative_mode = true;
				strcpy(tmpBuff, "G0 E25 F100");
				process_cmd_imd(tmpBuff);
				planner.synchronize();
				//坐标更新
				current_position[E_AXIS] = tmpPowerPanicData.PositionData[E_AXIS];
				planner.set_e_position_mm(current_position[E_AXIS]);
			}
			//绝对模式
			relative_mode = false;
			//暂停3  秒
			strcpy(tmpBuff, "G4 S3");
			process_cmd_imd(tmpBuff);
			//移动到续点XY  坐标
			sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", tmpPowerPanicData.PositionData[X_AXIS], tmpPowerPanicData.PositionData[Y_AXIS]);
			process_cmd_imd(tmpBuff);
			planner.synchronize();
			//移动到打印高度
			sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS]);
			process_cmd_imd(tmpBuff);
			planner.synchronize();

			//绝对模式
			relative_mode = false;
			//同步坐标计数器
			//Sync_PCounter();

			//速度还原
			saved_g1_feedrate_mm_s = Data.PrintFeedRate;
      saved_g0_feedrate_mm_s = Data.TravelFeedRate;

			//调平使能
			set_bed_leveling_enabled(true);

			*Err = 0;
      SystemStatus.SetCurrentPrinterStatus(STAT_RUNNING_ONLINE);
			//EnablePowerPanicCheck();
			if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
				Periph.StartDoorCheck();
			#if((HAVE_FILAMENT_SENSOR == 1) || (HAVE_FILAMENT_SWITCH == 1))
			if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
				Periph.StartFilamentCheck();
			#endif
			//清除断电数据
			MaskPowerPanicData();
      restoring = false;
			return 0;
		}
	}
	/*
	else
	{
		//发送失败
		HMI_SendPowerPanicResume(0x0c, 1);
		if(Err != NULL)
			*Err = 1;
		return false;
	}
	*/
    restoring = false;

    if(Err != NULL)
        *Err = 1;
    return false;
}

#else
 /**
 *Resume work after power panic if exist valid power panic data
 *return :true is resume success, or else false
 */
bool PowerPanic::PowerPanicResumeWork(uint8_t *Err)
{
  restoring = true;

	char tmpBuff[32];

	//暂存文件位置，运动会更新PowerPanicData  中的数据
	tmpPowerPanicData = Data;

	//数据有效
	if(tmpPowerPanicData.Valid != 0)
	{
		//U  盘打印断电
		if(tmpPowerPanicData.GCodeSource == GCODE_SOURCE_UDISK)
		{
      card.initsd();
      //尝试打开文件
      card.openFile(Data.FileName, true);

			//文件成功打开
			if(card.isFileOpen() == true)
			{
				//发送启动成功
				//HMI_SendPowerPanicResume(0x0c, 0);
				//风扇开启
				#ifdef H_FAN2_PORT
				H_FAN2_PORT->BSRR = H_FAN2_PIN;
				#endif
				//加热
				if(tmpPowerPanicData.BedTamp > 20)
				{
					sprintf(tmpBuff, "M140 S%0.2f", tmpPowerPanicData.BedTamp);
					process_cmd_imd(tmpBuff);
				}

				//目前只考虑加热头1
				sprintf(tmpBuff, "M104 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
				process_cmd_imd(tmpBuff);

				//X  Y  回原点
				strcpy(tmpBuff, "G28");
				process_cmd_imd(tmpBuff);

				//等待加热
				if(tmpPowerPanicData.BedTamp > 20)
				{
					sprintf(tmpBuff, "M190 S%0.2f", tmpPowerPanicData.BedTamp);
					process_cmd_imd(tmpBuff);
				}

				//目前只考虑加热头1
				sprintf(tmpBuff, "M109 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
				process_cmd_imd(tmpBuff);

				//绝对模式
				relative_mode = false;

				//移动到恢复点
				//调平失效
				set_bed_leveling_enabled(false);
				//移动到续点高度加5mm  位置
				sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS] + 5);
				process_cmd_imd(tmpBuff);
				planner.synchronize();

				//预挤出
				//相对模式
				relative_mode = true;
				strcpy(tmpBuff, "G0 E25 F100");
        process_cmd_imd(tmpBuff);
				planner.synchronize();
				//坐标更新
				current_position[E_AXIS] = tmpPowerPanicData.PositionData[E_AXIS];
        planner.set_e_position_mm(current_position[E_AXIS]);
				//绝对模式
				relative_mode = false;
				//暂停3  秒
				strcpy(tmpBuff, "G4 S3");
				process_cmd_imd(tmpBuff);
				//移动到续点XY  坐标
				sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", tmpPowerPanicData.PositionData[X_AXIS], tmpPowerPanicData.PositionData[Y_AXIS]);
				process_cmd_imd(tmpBuff);
				planner.synchronize();
				//移动到打印高度
				sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS]);
				process_cmd_imd(tmpBuff);
				planner.synchronize();
				//计时器开始工作
				print_job_timer.start();
        Stopwatch::resume(tmpPowerPanicData.accumulator);

				//文件跳转
				card.setIndex(tmpPowerPanicData.FilePosition);
				//启动打印
				card.startFileprint();
				//绝对模式
				relative_mode = false;
				//同步坐标计数器
				//Sync_PCounter();

				//速度还原
				//SetG0FeedRate(Data.TravelFeedRate);
				//SetG1FeedRate(Data.PrintFeedRate);
				saved_g1_feedrate_mm_s = Data.PrintFeedRate;
        saved_g0_feedrate_mm_s = Data.TravelFeedRate;

				//调平使能
				set_bed_leveling_enabled(true);

				//Gcode  带Z  坐标
				if(tmpPowerPanicData.ZMove == 1)
				{

				}
				//Gcode  不带Z  坐标
				else
				{
					sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS]);
					process_cmd_imd(tmpBuff);
					planner.synchronize();
				}

				*Err = 0;
        SystemStatus.SetCurrentPrinterStatus(STAT_RUNNING);
				//EnablePowerPanicCheck();
				if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
          Periph.StartDoorCheck();
				#if((HAVE_FILAMENT_SENSOR == 1) || (HAVE_FILAMENT_SWITCH == 1))
				if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
					Periph.StartFilamentCheck();
				#endif
				//清除断电数据
        restoring = false;
				MaskPowerPanicData();
				return 0;
			}
			else
			{
        restoring = false;
				//发送失败
				//HMI_SendPowerPanicResume(0x0c, 1);
				if(Err != NULL)
					*Err = 1;
				return false;
			}
		}
		//联机打印断电
		else
		{
			//发送启动成功
			//HMI_SendPowerPanicResume(0x0b, 0);
			//风扇开启
			#ifdef H_FAN2_PORT
			H_FAN2_PORT->BSRR = H_FAN2_PIN;
			#endif
			//机型是3D  打印
			if(MACHINE_TYPE_3DPRINT == tmpPowerPanicData.MachineType)
			{
				//加热
				if(tmpPowerPanicData.BedTamp > 20)
				{
					sprintf(tmpBuff, "M140 S%0.2f", tmpPowerPanicData.BedTamp);
					process_cmd_imd(tmpBuff);
				}

				//目前只考虑加热头1
				sprintf(tmpBuff, "M104 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
				process_cmd_imd(tmpBuff);
			}

			//X  Y  回原点
			strcpy(tmpBuff, "G28");
			process_cmd_imd(tmpBuff);

			//机型是3D  打印
			if(MACHINE_TYPE_3DPRINT == tmpPowerPanicData.MachineType)
			{
				//等待加热
				if(tmpPowerPanicData.BedTamp > 20)
				{
					sprintf(tmpBuff, "M190 S%0.2f", tmpPowerPanicData.BedTamp);
					process_cmd_imd(tmpBuff);
				}

				//目前只考虑加热头1
				sprintf(tmpBuff, "M109 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
				process_cmd_imd(tmpBuff);
			}

			//绝对模式
			relative_mode = false;

			//移动到恢复点
			//调平失效
			set_bed_leveling_enabled(false);
			//移动到续点高度加5mm  位置
			sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS] + 5);
			process_cmd_imd(tmpBuff);
			planner.synchronize();
			//机型是3D  打印
			if(MACHINE_TYPE_3DPRINT == tmpPowerPanicData.MachineType)
			{
				//预挤出
				//相对模式
				relative_mode = true;
				strcpy(tmpBuff, "G0 E25 F100");
				process_cmd_imd(tmpBuff);
				planner.synchronize();
				//坐标更新
				current_position[E_AXIS] = tmpPowerPanicData.PositionData[E_AXIS];
				planner.set_e_position_mm(current_position[E_AXIS]);
			}
			//绝对模式
			relative_mode = false;
			//暂停3  秒
			strcpy(tmpBuff, "G4 S3");
			process_cmd_imd(tmpBuff);
			//移动到续点XY  坐标
			sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", tmpPowerPanicData.PositionData[X_AXIS], tmpPowerPanicData.PositionData[Y_AXIS]);
			process_cmd_imd(tmpBuff);
			planner.synchronize();
			//移动到打印高度
			sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS]);
			process_cmd_imd(tmpBuff);
			planner.synchronize();

			//绝对模式
			relative_mode = false;
			//同步坐标计数器
			//Sync_PCounter();

			//速度还原
			saved_g1_feedrate_mm_s = Data.PrintFeedRate;
      saved_g0_feedrate_mm_s = Data.TravelFeedRate;

			//调平使能
			set_bed_leveling_enabled(true);

			//Gcode  带Z  坐标
			if(tmpPowerPanicData.ZMove == 1)
			{

			}
			//Gcode  不带Z  坐标
			else
			{
				sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS]);
				process_cmd_imd(tmpBuff);
				planner.synchronize();
			}

			*Err = 0;
      SystemStatus.SetCurrentPrinterStatus(STAT_RUNNING_ONLINE);
			//EnablePowerPanicCheck();
			if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
				Periph.StartDoorCheck();
			#if((HAVE_FILAMENT_SENSOR == 1) || (HAVE_FILAMENT_SWITCH == 1))
			if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
				Periph.StartFilamentCheck();
			#endif
			//清除断电数据
			MaskPowerPanicData();
      restoring = false;
			return 0;
		}
	}
	/*
	else
	{
		//发送失败
		HMI_SendPowerPanicResume(0x0c, 1);
		if(Err != NULL)
			*Err = 1;
		return false;
	}
	*/
    restoring = false;

    if(Err != NULL)
        *Err = 1;
    return false;
}
#endif

/*
 * disable other unused peripherals
 */
void PowerPanic::turnoffPower(void) {
  // these 3 statement will disable power supply for
  // HMI, all axes, all addones
  WRITE(POWER0_SUPPLY_PIN, POWER_SUPPLY_OFF);
  WRITE(POWER1_SUPPLY_PIN, POWER_SUPPLY_OFF);
  WRITE(POWER2_SUPPLY_PIN, POWER_SUPPLY_OFF);

  // disable power of heated bed
  WRITE(HEATER_BED_PIN, LOW);

  // disable other unnecessary soc peripherals
  // disable usart
  rcc_clk_disable(MSerial1.c_dev()->clk_id);
  rcc_clk_disable(MSerial2.c_dev()->clk_id);
  rcc_clk_disable(MSerial3.c_dev()->clk_id);

  // disble timer except the stepper's
  rcc_clk_disable(TEMP_TIMER_DEV->clk_id);
  rcc_clk_disable(TIMER7->clk_id);

  // disalbe ADC
  rcc_clk_disable(ADC1->clk_id);
  rcc_clk_disable(ADC2->clk_id);

  //disble DMA
  rcc_clk_disable(DMA1->clk_id);
  rcc_clk_disable(DMA2->clk_id);
}

/*
 * stop current working and recover stepper postion to current_position
 */
void PowerPanic::stopWorking(void) {

  /* NOTE: this is executed in stepper ISR!
   * to avoid race condition, we cannot clear block buffer or command queue
   * here, just delay next block/queue to be performed. Then do it in
   * powerpanic::process()
   */

  // stop stepper, it will abort the current block if it have
  stepper.quick_stop();

  // it will delay the stepper to get next avaible block
  planner.delay_before_delivering = 100;

  // it will delay planner to plan next movement
  planner.cleaning_buffer_counter = 1000;

  // above two variables will be cleared in powerpanic::process()

  // set current stepper position to current_position
  // then we can get current stepper position from current_position[4]
  set_current_from_steppers_for_axis(ALL_AXES);

  // stop stopwatch
  print_job_timer.stop();
}

/*
 * move axes to stop point
 * NOTE: this function is also called by pause/stop process
 */
void PowerPanic::towardStopPoint(void) {

  //切换到绝对坐标模式
  relative_mode = false;

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    if(thermalManager.temp_hotend[0].current > 180) {
        current_position[E_AXIS] -= 4;
        line_to_current_position(40);
        planner.synchronize();
        while(planner.movesplanned()) thermalManager.manage_heater();
      }

      if(all_axes_known != false) {
        if (powerloss)
          do_blocking_move_to_z(current_position[Z_AXIS] + 5, 10);  // if power loss, raise z for 5 mm
        else
          do_blocking_move_to_z(current_position[Z_AXIS] + 30, 10); // else raise z for 30mm
        //X  轴走到限位开关位置
        do_blocking_move_to_x(0, 35);
        //Y  轴走到最大位置
        do_blocking_move_to_xy(current_position[X_AXIS], home_offset[Y_AXIS] + Y_MAX_POS, 30);
      }
      Periph.StopFilamentCheck();
    break;

  case MACHINE_TYPE_CNC:
    //关闭电机
    ExecuterHead.CNC.SetCNCPower(0);

    if (powerloss)
      do_blocking_move_to_z(current_position[Z_AXIS] + 5, 10);
    else
      do_blocking_move_to_z(current_position[Z_AXIS] + 30, 10);
    while(planner.movesplanned())thermalManager.manage_heater();

    //走到工件原点
    do_blocking_move_to_xy(0, 0, 50);
    break;

  case MACHINE_TYPE_LASER:
    if (powerloss)
      Periph.StopDoorCheck();
    else
      //启动检测
      Periph.StartDoorCheck();
    break;

  default:
    break;
  }
}

/*
 * when a block is output ended, save it's line number
 * this function is called by stepper isr()
 *
 */
void PowerPanic::saveCmdLine(uint32_t l) {
  Data.FilePosition = l;
}


void PowerPanic::process(void) {
  if (!powerloss)
    return;

  // enable power for all axes, we need to move executer
  // to a specific position
  WRITE(POWER1_SUPPLY_PIN, POWER_SUPPLY_ON);

  // clear block queue
  planner.block_buffer_nonbusy = planner.block_buffer_planned \
      = planner.block_buffer_head = planner.block_buffer_tail;

  // clear command queue
  clear_command_queue();

  // these two variables will clean the latency in planning commands
  // and outputing blocks
  planner.delay_before_delivering = 0;
  planner.cleaning_buffer_counter = 0;

  towardStopPoint();

  // wait to die
  while (1);
}