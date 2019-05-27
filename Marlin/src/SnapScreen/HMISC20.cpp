#include "../inc/MarlinConfig.h"

#if ENABLED(HMI_SC20W)

#include "../module/stepper.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/printcounter.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../gcode/gcode.h"
#include "../gcode/parser.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../module/configuration_store.h"
#include "../sd/cardreader.h"
#include "../module/ExecuterManager.h"
#include "../module/PowerPanic.h"
#include "../module/StatusControl.h"
#include "../module/LaserExecuter.h"
#include "../module/CNCexecuter.h"
#include "../module/PeriphDevice.h"
#include <EEPROM.h>

#include "HMISC20.h"

extern long pCounter_X,pCounter_Y,pCounter_Z,pCounter_E;

char FileListBuff[1028];


//指令暂存缓冲，正确解析之后，指令存放在这里
static char tmpBuff[1024];

//检测指令缓冲是否为空
#define CMD_BUFF_EMPTY()	(commands_in_queue>0?false:true)
//U  盘枚举成功
#if ENABLED(SDSUPPORT)	
#define IS_UDISK_INSERTED IS_SD_INSERTED()
#else
#define IS_UDISK_INSERTED false
#endif

/**
 *PackedProtocal:Pack up the data in protocal
 */
void HMI_SC20::PackedProtocal(uint8_t *pData, uint16_t len)
{
  uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;

  while(len--)
    tmpBuff[i++] = *pData++;

  //重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/**
 *HmiWriteData:Write datas to the HMI serial port
 *para pData:the pointer to the datas
 *para len:number of the datas to be written
 */
void HMI_SC20::HmiWriteData(char *pData, uint16_t len)
{
  uint8_t c;
	while(len--)
	{
	  c = *pData++;
		HMISERIAL.write(c);
	} 
}

/**
 *SC20屏幕获取指令
 */
short HMI_SC20::GetCommand(unsigned char *pBuff)
{
  int c;
	uint16_t tmplen;
	uint16_t tmphead;
  uint16_t nexthead;
	uint16_t tmptail;
	uint16_t i;
	uint32_t checksum;

	tmphead = ReadHead;
	tmptail = ReadTail;

  while(1)
  {
    nexthead = (tmphead + 1) % sizeof(ReadBuff);
    if(nexthead == tmptail)
      break;
    
    c = HMISERIAL.read();
    if(c == -1)
      break;
    
    ReadBuff[tmphead] = (uint8_t)c;
    tmphead = nexthead;
  }
  ReadHead = tmphead;
	
	//没数据
	if (tmphead == tmptail)
	{
		return (short)-1;
	}

  ReadHead = tmphead;

	tmplen = (uint16_t)((tmphead + sizeof(ReadBuff) - tmptail) % sizeof(ReadBuff));
	
	//数据长度足够
	while(tmplen > 9)
	{
		if(ReadBuff[tmptail] != 0xAA)
		{
			tmptail = (tmptail + 1) % sizeof(ReadBuff);
			tmplen--;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}
		if(ReadBuff[(tmptail + 1) % sizeof(ReadBuff)] != 0x55)
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}
		//读取包长
		uint8_t cmdLen0 = ReadBuff[(tmptail + 2) % sizeof(ReadBuff)];
		uint8_t cmdLen1 = ReadBuff[(tmptail + 3) % sizeof(ReadBuff)];
		uint16_t commandLen = (uint16_t)((cmdLen0 << 8) | cmdLen1);
		//包长效验错误
		if((((commandLen >> 8) & 0xff) ^ (commandLen & 0xff)) != ReadBuff[(tmptail + 5) % sizeof(ReadBuff)])
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}

		//缓冲数据足够
		if(commandLen <= (tmplen - 8))
		{
			//复制数据
			for(i=0;i<(commandLen + 8);i++)
			{
				pBuff[i] = ReadBuff[tmptail];
				tmptail = (tmptail + 1) % sizeof(ReadBuff);
			}

			//更新读指针
			ReadTail = tmptail;

			//校验
			checksum = 0;
			for(i=0;i<(commandLen-1);i=i+2)
				checksum += (pBuff[i + 8] << 8) | pBuff[i + 9];

			//奇偶判断
			if(commandLen % 2)
				checksum += pBuff[commandLen + 8 - 1];

			while(checksum > 0xffff)
				checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

			checksum = ~checksum;
			if((uint16_t)checksum != (uint16_t)((pBuff[6] << 8) | pBuff[7]))
				return (short)-1;
			//_rx_buffer->tail = _rx_buffer->head = 0;
			return commandLen + 8;
		}
		//数据长度不足
		else
		{
			return (short)-1;
		}
		
	}
	return (short)-1;
}

/********************************************************
启动升级
*********************************************************/
void HMI_SC20::StartUpdate(void)
{
  uint32_t Address;
  uint8_t Pages;
  //擦除FLASH
  FLASH_Unlock();

  //擦除升级文件信息
  Pages = UPDATE_CONTENT_INFO_SIZE / 2048;
  Address = FLASH_UPDATE_CONTENT_INFO;
  for(int i=0;i<Pages;i++)
  {
    FLASH_ErasePage(Address);
    Address += 2048;
  }
  //擦除升级内容
  Pages = MARLIN_CODE_SIZE / 2048;
  Address = FLASH_UPDATE_CONTENT;
  for(int i=0;i<128;i++)
  {
    FLASH_ErasePage(Address);
    Address += 2048;
  }
  FLASH_Lock();
  SendStartUpdateReack(0);
  UpdateDataSize = 0;
  UpdateInProgress = 1;
  UpdatePackRequest = 0;
  SendUpdatePackRequest(UpdatePackRequest);
}

/********************************************************
升级包处理
参数    pBuff:数据缓冲区指针
      DataLen:数据长度
*********************************************************/
void HMI_SC20::UpdatePackProcess(uint8_t *pBuff, uint16_t DataLen)
{
  uint32_t Address;
  uint16_t Packindex;
  uint16_t u16Value;
  uint16_t maxpack;
  Packindex = (uint16_t)((pBuff[0] << 8) | pBuff[1]);
  maxpack = (MARLIN_CODE_SIZE + UPDATE_CONTENT_INFO_SIZE) / 512;
  //暂定500包，即250K
  if((Packindex < maxpack) && (UpdateInProgress == 1) && (Packindex == UpdatePackRequest))
  {
    //减去包序号2个字节
    DataLen = DataLen - 2;
    UpdateDataSize = Packindex * 512 + DataLen;
    Address = FLASH_UPDATE_CONTENT_INFO + Packindex * 512;
    if((DataLen % 2) != 0)
      DataLen++;
    FLASH_Unlock();
    for(int i=0;i<DataLen;i=i+2)
    {
      u16Value = ((pBuff[i + 3] << 8) | pBuff[i + 2]);
      FLASH_ProgramHalfWord(Address, u16Value);
      Address = Address + 2;
    }
    FLASH_Lock();
    UpdatePackRequest++;
    SendUpdatePackRequest(UpdatePackRequest);
  }
}

/********************************************************
升级结束应答
*********************************************************/
void HMI_SC20::UpdateComplete(void)
{
  if(UpdateDataSize == 0)   
    SendUpdateCompleteReack(1);
  else
    SendUpdateCompleteReack(0);
}

/********************************************************
平自动调平处理
*********************************************************/
void HMI_SC20::HalfAutoCalibrate()
{
	int j;
  int indexdir;
	int indexx, indexy;
	if(CMD_BUFF_EMPTY() == true)
	{
    //请求执行头的开关状态
    
    //CanRequestIOSwichStatus(0);
		//关闭热床和加热头
		thermalManager.disable_all_heaters();
    strcpy(tmpBuff, "G28");
    parser.parse(tmpBuff);
    gcode.process_next_command();
		while(planner.movesplanned());
		set_bed_leveling_enabled(false);

		//设置Z  轴最大速度
		planner.settings.max_feedrate_mm_s[Z_AXIS] = 50;

		//设置海平面点的坐标
		current_position[Z_AXIS] = Z_MAX_POS;
		sync_plan_position();
    indexx = 0;
    indexy = 0;
    indexdir = 1;
		for(j=0;j<(GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y);j++)
		{
			//Z  轴移动到13mm
			do_blocking_move_to_z(7);

			//X  Y  移动到第i  个调平点
			do_blocking_move_to_xy(_GET_MESH_X(indexx) - 11, _GET_MESH_Y(indexy) - 13, 70.0f);

			Periph.StartLevelingCheck();
      do_blocking_move_to_z(current_position[Z_AXIS] - 11, 1.16f);

			set_current_from_steppers_for_axis(ALL_AXES);
			MeshPointZ[indexy * GRID_MAX_POINTS_X + indexx] = current_position[Z_AXIS];
			Periph.StoplevelingCheck();
			sync_plan_position();
      //获取调平点索引值
			indexx += indexdir;
      if(indexx == GRID_MAX_POINTS_X)
      {
        indexy++;
        indexdir = -1;
      }
      else if(indexx < 0)
      {
        indexy++;
        indexdir = 1;
      }
			//发送进度
			SettingReack(0x03, indexy * GRID_MAX_POINTS_X + indexx + 1);
		}

		//Zoffset
		do_blocking_move_to_z(7, 50);

    do_blocking_move_to_xy(_GET_MESH_X(0) + _GET_MESH_X(GRID_MAX_POINTS_X - 1) / 2.0f, _GET_MESH_Y(0) + _GET_MESH_Y(GRID_MAX_POINTS_Y - 1) / 2.0f, 50.0f);

		//设置Z  轴最大速度
		planner.settings.max_feedrate_mm_s[Z_AXIS] = 20;
		//屏幕锁定
		HMICommandSave = 1;

		//标置自动调平
		CalibrateMethod = 1;
	}
}


/****************************************************
手动调平启动
***************************************************/
void HMI_SC20::ManualCalibrateStart()
{
	int i,j;
	//if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
	if(CMD_BUFF_EMPTY() == true)
	{
		//请求执行头的开关状态
		//CanRequestIOSwichStatus(0);
		//关闭热床和加热头
		thermalManager.disable_all_heaters();
		strcpy(tmpBuff, "G28");
		parser.parse(tmpBuff);
    gcode.process_next_command();
		while(planner.movesplanned());

		set_bed_leveling_enabled(false);

		//绝对坐标模式
    do_blocking_move_to_xy(home_offset[X_AXIS], home_offset[Y_AXIS]);
    do_blocking_move_to_z(0);
		
		//设置海平面点的坐标
		//限位开关在最高处
		if(Z_HOME_DIR > 0)
			current_position[Z_AXIS] = Z_MAX_POS;
		//限位开关在最低处
		else
			current_position[Z_AXIS] = 0;
		sync_plan_position();

		//Z  轴移到20  的位置
		do_blocking_move_to_z(15);

		//初始化
		PointIndex = 99;
		
		//屏幕锁定
		//HMICommandSave = 1;

		for(i=0;i<GRID_MAX_POINTS_Y;i++)
		{
			for(j=0;j<GRID_MAX_POINTS_X;j++)
			{
				MeshPointZ[i * GRID_MAX_POINTS_X + j] = z_values[i][j];
			}
		}

		//标置手动调平
		CalibrateMethod = 2;
	}
}

/****************************************************
机器尺寸重定义
***************************************************/
void HMI_SC20::ResizeMachine(char *pBuff)
{
	uint32_t u32Value;
	int32_t int32Value;
	uint16_t j;

	//长宽高
	j = 0;
	//X
	u32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	X_MAX_POS = u32Value / 1000.0f;
	j = j + 4;
	//Y
	u32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	Y_MAX_POS = u32Value / 1000.0f;
	j = j + 4;
	//Z
	u32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	Z_MAX_POS = u32Value / 1000.0f;
	j = j + 4;

	//回原点方向
	//X
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		X_HOME_DIR = -1;
	else
		X_HOME_DIR = 1;
	j = j + 4;
	
	//Y
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		Y_HOME_DIR = -1;
	else
		Y_HOME_DIR = 1;
	j = j + 4;
	
	//Z
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		Z_HOME_DIR = -1;
	else
		Z_HOME_DIR = 1;
  j = j + 4;

	//电机方向
	//X
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		X_DIR = false;
	else
		X_DIR = true;
	j = j + 4;
	
	//Y
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		Y_DIR = false;
	else
		Y_DIR = true;
	j = j + 4;
	
	//Z
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		Z_DIR = false;
	else
		Z_DIR = true;
  j = j + 4;

	//E
	E_DIR = true;

	//Offset
	//X
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	home_offset[X_AXIS] = int32Value / 1000.0f;
	j = j + 4;

	//Y
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	home_offset[Y_AXIS] = int32Value / 1000.0f;
	j = j + 4;

	//Z
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	home_offset[Z_AXIS] = int32Value / 1000.0f;
	j = j + 4;

  #if ENABLED(SW_MACHINE_SIZE)
    UpdateMachineDefines();
  #endif

  do_blocking_move_to(current_position[X_AXIS] + 0.05f, current_position[Y_AXIS] + 0.05f, current_position[Z_AXIS] + 0.05f, 16);
  do_blocking_move_to(current_position[X_AXIS] - 0.05f, current_position[Y_AXIS] - 0.05f, current_position[Z_AXIS] - 0.05f, 16);

	//保存数据
	settings.save();
}

/****************************************************
激光焦点设置启动
***************************************************/
void HMI_SC20::EnterLaserFocusSetting()
{
	char strCmd[50];

	//回原点
	strcpy(strCmd, "G28 Z");
	parser.parse(tmpBuff);
  gcode.process_next_command();
	//走到特定高度
	do_blocking_move_to_z(20.0f);
	//微光
	ExecuterHead.Laser.SetLaserPower(0.5f);
}

void HMI_SC20::PollingCommand(void)
{
	uint8_t CurStatus;
	uint32_t ID;
	float fCenterX, fCenterY; 
	int32_t int32Value;
	uint8_t eventId;
	short i;
	uint16_t j;
	uint16_t cmdLen;
	i = GetCommand((unsigned char*)tmpBuff);

	if(i == (short)-1)
	{
	}
	//屏幕协议
	else
	{
		cmdLen = (tmpBuff[2] << 8) | tmpBuff[3];
		
		eventId = tmpBuff[8];
		
		//上位指令调试
		if(eventId == 0x01)
		{
		  Periph.EncloseLedOn();
			//指令尾补0
			j = cmdLen + 8;
			tmpBuff[j] = 0;
			Screen_enqueue_and_echo_commands(&tmpBuff[13], 0xffffffff, 0x02);	
		}
		//GCode  打印
		else if(eventId == 0x03)
		{	  
			Periph.EncloseLedOn();
			//获取当前状态
			CurStatus = SystemStatus.GetCurrentPrinterStatus();
			//上位启动打印
			if((CurStatus== STAT_RUNNING_ONLINE) || (CurStatus== STAT_PAUSE_ONLINE))
			{
				//激光头
				if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
				{
					//行号
					ID = (tmpBuff[9] << 24) | (tmpBuff[10] << 16) | (tmpBuff[11] << 8) | tmpBuff[12];
					//指令尾补0
					j = cmdLen + 8;
					tmpBuff[j] = 0;

					if(Periph.GetDoorCheckFlag() == true)
					{
						//门已关闭
						if(Periph.IsDoorOpened() == false)
						{
							Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
						}
						else
						{
							SystemStatus.PauseTriggle(ManualPause);
							SystemStatus.SetCurrentPrinterStatus(STAT_PAUSE_ONLINE);
						}
					}
					else
          {     
					  Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
          }
				}
				//非激光头
				else
				{
					//行号
					ID = (tmpBuff[9] << 24) | (tmpBuff[10] << 16) | (tmpBuff[11] << 8) | tmpBuff[12];
					//指令尾补0
					j = tmpBuff[3] + 8;
					tmpBuff[j] = 0;
					Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
				}
			}
		}
    #if(0)
		//文件操作
		else if(eventId == 0x05)
		{
			Periph.ShieldLedOn();
			//控制字
			switch(tmpBuff[9])
			{
				//重新挂载
				case 0:
					card.initsd();
					if(card.isDetected())
						SendInitUdisk(0);
					else
						SendInitUdisk(1);
					//清除项目显示偏移量
					ListFileOffset = 0;
				break;

				//获取当前目录路径
				case 1:
					if(card.getWorkDirPath() == true)
						SendCurrentUDiskPath(0);
					else
						SendCurrentUDiskPath(1);
				break;

				//进入目录
				case 2:
					ID = 0;
					FileListBuff[ID] = 0;
					//清除项目显示偏移量
					ListFileOffset = 0;
					if(card.getWorkDirPath() == true)
					{
						for(int i=10;i<263;i++)
						{
							FileListBuff[ID++] = tmpBuff[i];
							if(tmpBuff[i] == 0)
								break;
						}
						if(card.chdir(FileListBuff) == 0)
						{
							SendChDirResult(0);
						}
						else
						{
							SendChDirResult(1);
						}
					}
					else
					{
						SendChDirResult(1);
					}
				break;

				//返回上一级
				case 3:
				break;

				//获取当前目录的内容
				case 4:
					ListFileOffset += SendDirItems(ListFileOffset);
				break;

				//获取文件特殊内容
				case 5:
					//待机状态下可操作
					if(SystemStatus.GetCurrentPrinterStatus() == STAT_IDLE)
					{
						ID = 0;
						FileListBuff[ID] = 0;
						if(card.getWorkDirPath() == true)
						{
							for(int i=10;i<263;i++)
							{
								FileListBuff[ID++] = tmpBuff[i];
								if(tmpBuff[i] == 0)
									break;
							}
						}
						SendSpecialData();
					}
				break;

				//启动U  盘打印
				case 6:
					//获取当前状态
					CurStatus = SystemStatus.GetCurrentPrinterStatus();
					//待机状态下可操作
					if(CurStatus == STAT_IDLE)
					{
						ID = 0;
						FileListBuff[ID] = 0;
						tmpBuff[cmdLen + 8] = 0;
						if(card.getWorkDirPath() == true)
						{
							for(int i=10;i<263;i++)
							{
								FileListBuff[ID++] = tmpBuff[i];
								if(tmpBuff[i] == 0)
									break;
							}
						}
						card.StartSelectPrint(FileListBuff);
						//文件打开成功
						if(card.isFileOpen() == false)
						{
							//发送文件打开失败
							SendStartPrintReack(1);
							break;
						}
						PowerPanicData.FilePosition = 0;
						PowerPanicData.accumulator = 0;
						PowerPanicData.HeaterTamp[0] = 0;
						PowerPanicData.BedTamp = 0;
						PowerPanicData.PositionData[0] = 0;
						PowerPanicData.PositionData[1] = 0;
						PowerPanicData.PositionData[2] = 0;
						PowerPanicData.PositionData[3] = 0;
						PowerPanicData.GCodeSource = GCODE_SOURCE_UDISK;
						PowerPanicData.MachineType = ExecuterHead.MachineType;
						SendStartPrintReack(0);

						//使能检测
						EnablePowerPanicCheck();
						//激光功能
						if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
						{
							//门检测开启
							if(Periph.GetDoorCheckFlag() == true)
							{
								//门已关闭
								if(READ(DOOR_PIN) == true)
								{
									SystemStatus.PauseTriggle(ManualPause);
									SystemStatus.SetCurrentPrinterStatus(STAT_PAUSE);
								}
							}
							//启动门开关检测
							Periph.StartDoorCheck();
						}
							
						//屏幕锁定
						HMICommandSave = 1;
					}
				break;
			}
		}
    #endif
		//状态上报
		else if(eventId == 0x07)
		{
			uint8_t StatuID;
			StatuID = tmpBuff[9];
			//获取当前状态
			CurStatus = SystemStatus.GetCurrentPrinterStatus();
			//查询状态
			if(StatuID == 0x01)
			{
				SendMachineStatus();
			}
			//查询异常
			else if(StatuID == 0x02)
			{
				SendMachineFaultFlag();
			}
			//联机打印
			else if(StatuID == 0x03)
			{
				//待机状态中
				if(CurStatus == STAT_IDLE)
				{
					//设置打印状态
					SystemStatus.SetCurrentPrinterStatus(STAT_RUNNING_ONLINE);
					
					//激光执行头
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
					{
					  
						//Z  未知坐标
						if(axes_homed(Z_AXIS) == false)
						{
							//Z  轴回原点
							strcpy(tmpBuff, "G28 Z");
							parser.parse(tmpBuff);
              gcode.process_next_command();
						}
            //走到工件坐标
            do_blocking_move_to_xy(0, 0);
					}
					
					PowerPanicData.Data.FilePosition = 0;
					PowerPanicData.Data.accumulator = 0;
					PowerPanicData.Data.HeaterTamp[0] = 0;
					PowerPanicData.Data.BedTamp = 0;
					PowerPanicData.Data.PositionData[0] = 0;
					PowerPanicData.Data.PositionData[1] = 0;
					PowerPanicData.Data.PositionData[2] = 0;
					PowerPanicData.Data.PositionData[3] = 0;
					PowerPanicData.Data.GCodeSource = GCODE_SOURCE_SCREEN;
					PowerPanicData.Data.MachineType = ExecuterHead.MachineType;
					//使能断电检测
					//EnablePowerPanicCheck();
					//使能检测
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
						Periph.StartFilamentCheck();
					SendMachineStatusChange(0x03, 0);
					//屏幕锁定
					HMICommandSave = 1;
				}
			}
			//暂停
			else if(StatuID == 0x04)
			{
				//获取当前状态
				CurStatus = SystemStatus.GetCurrentPrinterStatus();
				if(CurStatus == STAT_RUNNING)
				{
					HmiRequestStatus = STAT_PAUSE;
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
						ExecuterHead.Laser.SetLaserPower(0.0f);
					SystemStatus.PauseTriggle(ManualPause);
				}
				else if(CurStatus == STAT_RUNNING_ONLINE)
				{
					HmiRequestStatus = STAT_PAUSE_ONLINE;
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
						ExecuterHead.Laser.SetLaserPower(0.0f);
					SystemStatus.PauseTriggle(ManualPause);
				}
			}
			//继续
			else if(StatuID == 0x05)
			{
				//获取当前状态
				CurStatus = SystemStatus.GetCurrentPrinterStatus();
				//U  盘打印
				if(CurStatus ==  STAT_PAUSE)
				{
					//激光执行头
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
					{
						//关闭检测
						Periph.StopDoorCheck();
						//门开关检测使能
						if(Periph.GetDoorCheckFlag() == true)
						{
							//门已关闭
							if(Periph.IsDoorOpened() == false)
							{
								//启动打印
								SystemStatus.PauseResume();
							}
							//开启检测
							Periph.StartDoorCheck();
						}
						//禁能门开关检测
						else
						{
							SystemStatus.PauseResume();
						}
					}
					else
					{
						SystemStatus.PauseResume();
					}
					//清除故障标志
					SystemStatus.ClearSystemFaultBit(FAULT_FLAG_FILAMENT);
					HmiRequestStatus = STAT_RUNNING;
				}
				//连机打印
				else if(CurStatus == STAT_PAUSE_ONLINE)
				{
					SystemStatus.PauseResume();
					//清除故障标志
					SystemStatus.ClearSystemFaultBit(FAULT_FLAG_FILAMENT);
					HmiRequestStatus = STAT_RUNNING_ONLINE;
					
				}
			}
			
			//停止
			else if(StatuID == 0x06)
			{
				//不在待机状态
				if(CurStatus != STAT_IDLE)
				{
					//上位机停止分2  种，立即停止
					SystemStatus.StopTriggle(ManualStop);
					//清除断电有效标置
					PowerPanicData.Data.Valid = 0;
				}
			}
			//打印结束
			else if(StatuID == 0x07)
			{
				//不在待机状态
				if(CurStatus != STAT_IDLE)
				{	
					//触发停止
					SystemStatus.StopTriggle(EndPrint);
					//清除断电有效标置
					PowerPanicData.Data.Valid = 0;
				}
			}
			//请求最近行号
			else if(StatuID == 0x08)
			{
				SendBreakPointData();
			}
			//请求打印进度
			else if(StatuID == 0x09)
			{
				//文件打开
				if(card.isFileOpen() == true)
					//更新最后进度
					card.LastPercent = card.percentDone();
				//发送进度
				SendProgressPercent(card.LastPercent);
			}
			//清除断电续打数据
			else if(StatuID == 0x0a)
			{
				//清除标置
				SystemStatus.ClearSystemFaultBit(0xffffffff);
				//断电数据有效
				if(PowerPanicData.Data.Valid == 1)
				{
					//清除Flash  有效位
					PowerPanicData.MaskPowerPanicData();
				}
				//标志断电续打无效
				PowerPanicData.Data.Valid = 0;
				//应答
				SendFaultClearReack();
			}
			//联机续打
			else if(StatuID == 0x0b)
			{
				//获取当前状态
				CurStatus = SystemStatus.GetCurrentPrinterStatus();
				//激光执行头
				if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
				{
					//外罩门检测开启
					if(Periph.GetDoorCheckFlag() == true)
					{
						//门已关闭且处于待机状态
						if((Periph.IsDoorOpened() == false) && (CurStatus == STAT_IDLE))
						{
							//启动打印
							PowerPanicData.PowerPanicResumeWork(NULL);
							//开启检测
							Periph.StartDoorCheck();
						}
						//门未关闭
						else
						{
							//发送失败
							SendPowerPanicResume(0x0b, 1);
						}
					}
					//不检测外罩门，待机状态
					else if(CurStatus == STAT_IDLE)
					{
						//启动打印
						PowerPanicData.PowerPanicResumeWork(NULL);
					}
					//不检测外罩 门，非待机状态
					else
					{
						//发送失败
						SendPowerPanicResume(0x0b, 1);
					}
				}
				//CNC  或3D  打印
				else
				{
					//处于待机状态
					if(CurStatus == STAT_IDLE)
					{
						//切换状态
						SystemStatus.SetCurrentPrinterStatus(STAT_PAUSE_ONLINE);
						//启动断电续打
						PowerPanicData.PowerPanicResumeWork(NULL);
					}
					//非待机状态
					else
					{
						//发送失败
						SendPowerPanicResume(0x0b, 1);
					}
				}
			}
			//U  盘续打
			else if(StatuID == 0x0c)
			{
				//获取当前状态
				CurStatus = SystemStatus.GetCurrentPrinterStatus();
				//激光执行头
				if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
				{
					//外罩门检测开启
					if(Periph.GetDoorCheckFlag() == true)
					{
						//门已关闭且处于待机状态
						if((Periph.IsDoorOpened() == false) && (CurStatus == STAT_IDLE))
						{
							//启动打印
							PowerPanicData.PowerPanicResumeWork(NULL);
							//开启检测
							Periph.StartDoorCheck();
						}
						//门未关闭或不处于待机状态
						else
						{
							//发送失败
							SendPowerPanicResume(0x0c, 1);
						}
					}
					//不检测外罩门，待机状态
					else if(CurStatus == STAT_IDLE)
					{
						//启动打印
						PowerPanicData.PowerPanicResumeWork(NULL);
					}
					//不检测外罩 门，非待机状态
					else
					{
						//发送失败
						SendPowerPanicResume(0x0c, 1);
					}
				}
				//CNC  或3D  打印
				else
				{
					//处于待机状态
					if(CurStatus == STAT_IDLE)
					{
						//启动断电续打
						PowerPanicData.PowerPanicResumeWork(NULL);
					}
					//非待机状态
					else
					{
						//发送失败
						SendPowerPanicResume(0x0c, 1);
					}
				}
			}
		}
		//操作指令
		else if(eventId == 0x09)
		{
			switch(tmpBuff[9])
			{
				//设置尺寸
				case 1:
					ResizeMachine(&tmpBuff[10]);

					//应答
					SettingReack(0x01, 0);
				break;

				//开启自动调平
				case 2:
					//3D  打印并开启调平传感器
					//if((MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType) && (HeadProbeEnable == true))
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
					{
						HalfAutoCalibrate();
						//应答
						SettingReack(0x02, 0);
					}
					else
					{
						SettingReack(0x02, 1);
					}
				break;

				//开启手动调平
				case 4:
					//3D  打印开启
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
					{
						ManualCalibrateStart();
						//应答
						SettingReack(0x04, 0);
					}
					else
					{
						SettingReack(0x04, 1);
					}
				break;

				//移动到调平点
				case 5:
					if((tmpBuff[10] < 10) && (tmpBuff[10] > 0))
					{
						//有效索引
						if(PointIndex < 10)
						{
							//保存数据
							MeshPointZ[PointIndex] = current_position[Z_AXIS];
						}
						//更新点索引
						PointIndex = tmpBuff[10] - 1;
						do_blocking_move_to_z(current_position[Z_AXIS] + 5, 30);						
						do_blocking_move_to_xy(_GET_MESH_X(PointIndex % GRID_MAX_POINTS_X), _GET_MESH_Y(PointIndex / GRID_MAX_POINTS_Y), 60.0f);
            
						//使能调平传感器
						if(Periph.LevelingSensorValid() == true)
						{
						  //启动探头触发
						  Periph.StartLevelingCheck();
							//走到-5  位置
							do_blocking_move_to_z(-5.0, 0.2);
							
							//关闭探头触发
							Periph.StoplevelingCheck();
							//更新Z  轴坐标
							set_current_from_steppers_for_axis(ALL_AXES);
							sync_plan_position();
						}
						//禁能调平传感器
						else
						{
							do_blocking_move_to_z(current_position[Z_AXIS] - 5, 0.2);
						}
						
						//应答
						SettingReack(0x05, 0);
					}
				break;

				//Z  轴移动
				case 6:
					int32Value = (tmpBuff[10] << 24) | (tmpBuff[11] << 16) | (tmpBuff[12] << 8) | (tmpBuff[13]);
          do_blocking_move_to_z(current_position[Z_AXIS] + int32Value, 20.0f);
					//应答
					SettingReack(0x06, 0);
				break;

				//保存调平点
				case 7:
					if(CMD_BUFF_EMPTY() == true)
					{
						//自动调平方式
						if(CalibrateMethod == 1)
						{
							//设置调平值
							for(i=0;i<GRID_MAX_POINTS_Y;i++)
							{
								for(j=0;j<GRID_MAX_POINTS_X;j++)
                {        
                  sprintf(tmpBuff, "G29 W I0 J0 Z%0.3f", MeshPointZ[i * GRID_MAX_POINTS_X + j]);
                  parser.parse(tmpBuff);
                  gcode.process_parsed_command();
                }
							}
							//保存数据
							settings.save();
						}
						//手动调平
						else if(CalibrateMethod == 2)
						{
							if(PointIndex != 99)
							{
								MeshPointZ[PointIndex] = current_position[Z_AXIS];
                //设置调平值
              	for(i=0;i<GRID_MAX_POINTS_Y;i++)
              	{
                  for(j=0;j<GRID_MAX_POINTS_X;j++)
                  {
                    sprintf(tmpBuff, "G29 W I0 J0 Z%0.3f", MeshPointZ[i * GRID_MAX_POINTS_X + j]);
                    parser.parse(tmpBuff);
                    gcode.process_parsed_command();
                  }
              	}
								//保存数据
								settings.save();
							}
						}
            //回原点
						strcpy(tmpBuff, "G28");
						parser.parse(tmpBuff);
            gcode.process_next_command();
            //切换到绝对位置模式
            relative_mode = false;
            //清除标志
            CalibrateMethod = 0;
            //应答
            SettingReack(0x07, 0);
            //解除屏幕锁定
						HMICommandSave = 0;
					}
				break;

				//退出调平点
				case 8:
					if(CMD_BUFF_EMPTY() == true)
					{
						//Load
						settings.load();
						strcpy(tmpBuff, "G28");
						parser.parse(tmpBuff);
            gcode.process_next_command();
	
						HMICommandSave = 0;
						//切换到绝对位置模式
						relative_mode = false;
						//应答
						SettingReack(0x08, 0);
						//解除屏幕锁定
						HMICommandSave = 0;
					}
				break;

				//出厂设置
				case 9:
				break;

				//读取激光Z  轴高度
				case 10:
					//读取
					ExecuterHead.Laser.LoadFocusHeight();
					SendLaserFocus(tmpBuff[9]);
				break;

				//设置激光Z  轴高度
				case 11:
					//关闭激光
					ExecuterHead.Laser.SetLaserPower(0.5f);
					//保存
					ExecuterHead.Laser.SaveFocusHeight(0, current_position[Z_AXIS]);
					//读取
					ExecuterHead.Laser.LoadFocusHeight();
					SendLaserFocus(tmpBuff[9]);
				break;

				//激光Z  轴回原点调焦专用
				case 12:
					//全部回原点
					strcpy(tmpBuff, "G28");
					parser.parse(tmpBuff);
          gcode.process_next_command();

					//调平数据失效
					set_bed_leveling_enabled(false);

					//Z  轴移动到70  位置
					do_blocking_move_to_z(70, 40);

					//X  Y  移动到中心位置
					fCenterX = (X_MAX_POS + home_offset[X_AXIS]) / 2.0f;
					fCenterY = (Y_MAX_POS + home_offset[Y_AXIS]) / 2.0f;
					do_blocking_move_to_xy(fCenterX, fCenterY, 4.0f);
					SettingReack(0x0c, 0);
				break;

				//读取尺寸参数
				case 20:
					SendMachineSize();
				break;
			}
		}
		//Movement Request
		else if(eventId == 0x0b)
		{
			switch(tmpBuff[9])
			{
				//激光回原点应答
				case 0x01:
					strcpy(tmpBuff, "G28 Z");
					parser.parse(tmpBuff);
          gcode.process_next_command();
					//调平数据失效
					set_bed_leveling_enabled(false);
					MovementRequestReack(tmpBuff[9], 0);
				break;
			}
		}
        //升级
		else if(eventId == 0xA9)
        {
            switch(tmpBuff[9])
            {
                //启动升级
                case 0:
                    StartUpdate();
                break;

                //升级包数据
                case 1:
                    UpdatePackProcess((uint8_t*)&tmpBuff[10], cmdLen - 2);
                break;

                //升级结束
                case 2:
                    UpdateComplete();
                break;
            }
        }
		//ReadTail = ReadHead;
	}
}


/***********************************************
发送进度
参数    Percent:进度值，取值范围0-100
************************************************/
void HMI_SC20::SendProgressPercent(uint8_t Percent)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//行号
	tmpBuff[i++] = 0x09;

	//断点数据
	tmpBuff[i++] = 0;
	tmpBuff[i++] = 0;
	tmpBuff[i++] = 0;
	tmpBuff[i++] = Percent;

	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;

	HmiWriteData(tmpBuff, i);
}


/***********************************************
发送断电续打应答
参数    OpCode:操作码
      Result:结果，0表示成功，非0表示失败
************************************************/
void HMI_SC20::SendPowerPanicResume(uint8_t OpCode, uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//Operation ID
	tmpBuff[i++] = OpCode;
	//结果
	tmpBuff[i++] = Result;

	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
发送激光焦点
************************************************/
void HMI_SC20::SendLaserFocus(uint8_t OpCode)
{
	uint32_t u32Value;
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x0a;
	//获取尺寸
	tmpBuff[i++] = OpCode;
	//结果
	tmpBuff[i++] = 0;

	switch(OpCode)
	{
		//读取激光焦点高度应答
		case 10:
			u32Value = (uint32_t)((ExecuterHead.Laser.FocusHeight) * 1000.0f);
			tmpBuff[i++] = (uint8_t)(u32Value >> 24);
			tmpBuff[i++] = (uint8_t)(u32Value >> 16);
			tmpBuff[i++] = (uint8_t)(u32Value >> 8);
			tmpBuff[i++] = (uint8_t)u32Value;
		break;

		//设置激光焦点高度应答
		case 11:
			tmpBuff[i++] = 0;
		break;

		//无效指令
		default:
			//重填结果
			tmpBuff[10] = 1;
		break;
	}

	//重填长度
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/***********************************************
发送尺寸信息
************************************************/
void HMI_SC20::SendMachineSize()
{
	int32_t int32Value;
	uint32_t u32Value;
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x0a;
	//获取尺寸
	tmpBuff[i++] = 20;
	tmpBuff[i++] = 0;

	//Size
	u32Value = (uint32_t)(X_MAX_POS * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	u32Value = (uint32_t)(Y_MAX_POS * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	u32Value = (uint32_t)(Z_MAX_POS * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	//Offset
	int32Value = (int32_t)(home_offset[X_AXIS] * 1000.0f);
	tmpBuff[i++] = (uint8_t)(int32Value >> 24);
	tmpBuff[i++] = (uint8_t)(int32Value >> 16);
	tmpBuff[i++] = (uint8_t)(int32Value >> 8);
	tmpBuff[i++] = (uint8_t)(int32Value);

	int32Value = (int32_t)(home_offset[Y_AXIS] * 1000.0f);
	tmpBuff[i++] = (uint8_t)(int32Value >> 24);
	tmpBuff[i++] = (uint8_t)(int32Value >> 16);
	tmpBuff[i++] = (uint8_t)(int32Value >> 8);
	tmpBuff[i++] = (uint8_t)(int32Value);

	int32Value = (int32_t)(home_offset[Z_AXIS] * 1000.0f);
	tmpBuff[i++] = (uint8_t)(int32Value >> 24);
	tmpBuff[i++] = (uint8_t)(int32Value >> 16);
	tmpBuff[i++] = (uint8_t)(int32Value >> 8);
	tmpBuff[i++] = (uint8_t)(int32Value);

	//重填长度
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/***********************************************
启动升级应答
参数    Result:应答结果，0表示启动成功，非0表示失败
***********************************************/
void HMI_SC20::SendStartUpdateReack(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0xAA;
	//OP ID
	tmpBuff[i++] = 0;
	//结果
	tmpBuff[i++] = Result;
	
	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
升级包请求
参数    PackRequested:请求的包序号
***********************************************/
void HMI_SC20::SendUpdatePackRequest(uint16_t PackRequested)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0xAA;
	//OP ID
	tmpBuff[i++] = 1;
	//包序号
	tmpBuff[i++] = (PackRequested >> 8);
    tmpBuff[i++] = (PackRequested);
	
	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
升级结束应答
参数    Resultl:结果，0表示成功，非0表示失败
***********************************************/
void HMI_SC20::SendUpdateCompleteReack(uint16_t Resultl)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0xAA;
	//OP ID
	tmpBuff[i++] = 2;
	//包序号
    tmpBuff[i++] = (Resultl);
	
	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
回复移动
***********************************************/
void HMI_SC20::MovementRequestReack(uint8_t OP_ID, uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x0C;
	//OP ID
	tmpBuff[i++] = OP_ID;
	//结果
	tmpBuff[i++] = Result;

	
	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
回复设置
***********************************************/
void HMI_SC20::SettingReack(uint8_t OP_ID, uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x0A;
	//OP ID
	tmpBuff[i++] = OP_ID;
	//结果
	tmpBuff[i++] = Result;

	
	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/***********************************************
回复暂停打印行号
***********************************************/
void HMI_SC20::SendBreakPointLine()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//行号
	tmpBuff[i++] = 0x08;

	//断点数据
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition);

	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/***********************************************
清除错误应答
***********************************************/
void HMI_SC20::SendFaultClearReack()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//OpCode
	tmpBuff[i++] = 0x0A;

	//结果
	tmpBuff[i++] = 0;

	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
发送断点数据
************************************************/
void HMI_SC20::SendBreakPointData()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//Opcode
	tmpBuff[i++] = 0x08;

	//断点数据
	tmpBuff[i++] = PowerPanicData.Data.Valid;
	tmpBuff[i++] = PowerPanicData.Data.GCodeSource;
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition);

	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/***********************************************
发送报警
************************************************/
void HMI_SC20::SendMachineFaultFlag()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//异常上报
	tmpBuff[i++] = 0x02;
	//异常标志
	uint32_t SysFaultFlag;
  SysFaultFlag = SystemStatus.GetSystemFault();
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 24);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 16);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 8);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag);

	//打印文件源
	if(SystemStatus.GetCurrentPrinterStatus() == STAT_IDLE)
		tmpBuff[i++] = 3;
	else
		tmpBuff[i++] = PowerPanicData.Data.GCodeSource;

	//行号
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition);

	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
发送状态切变
************************************************/
void HMI_SC20::SendMachineStatusChange(uint8_t Status, uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//目前状态
	tmpBuff[i++] = Status;
	//处理结果
	tmpBuff[i++] = Result;

	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
发送状态信息
************************************************/
void HMI_SC20::SendMachineStatus()
{
	float fValue;
	uint32_t u32Value;
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//同步状态
	tmpBuff[i++] = 0x01;

	//坐标
	fValue = pCounter_X / planner.settings.axis_steps_per_mm[X_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_Y / planner.settings.axis_steps_per_mm[Y_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_Z / planner.settings.axis_steps_per_mm[Z_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_E / planner.settings.axis_steps_per_mm[E_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	//温度
	int16_t T0,TB,T0S,TBS;
	//if(current_temperature[0] >= 0)
		T0 = (int16_t)thermalManager.temp_hotend[0].current;

	T0S = (int16_t)thermalManager.temp_hotend[0].target;
	
	//if(current_temperature_bed >= 0)
		TB = (int16_t)thermalManager.temp_bed.current;

	TBS = (int16_t)thermalManager.temp_bed.target;
	
	tmpBuff[i++] = (uint8_t)((int)TB >> 8);
	tmpBuff[i++] = (uint8_t)((int)TB);
	tmpBuff[i++] = (uint8_t)((int)TBS >> 8);
	tmpBuff[i++] = (uint8_t)((int)TBS);
	tmpBuff[i++] = (uint8_t)((int)T0 >> 8);
	tmpBuff[i++] = (uint8_t)((int)T0);
	tmpBuff[i++] = (uint8_t)((int)T0S >> 8);
	tmpBuff[i++] = (uint8_t)((int)T0S);

	//FeedRate
	//tmpBuff[i++] = (uint8_t)(HmiFeedRate >> 8);
	//tmpBuff[i++] = (uint8_t)(HmiFeedRate);
	tmpBuff[i++] = 0;
  tmpBuff[i++] = 0;

	//LaserPower
	tmpBuff[i++] = 0;
	tmpBuff[i++] = 0;
	tmpBuff[i++] = 0;
	tmpBuff[i++] = (uint8_t)(ExecuterHead.Laser.LastPercent);

	//RPM
	tmpBuff[i++] = 0;
	tmpBuff[i++] = 0;
	tmpBuff[i++] = (uint8_t)(ExecuterHead.CNC.RPM >> 8);
	tmpBuff[i++] = (uint8_t)(ExecuterHead.CNC.RPM);

	//打印机状态
	j = SystemStatus.GetCurrentPrinterStatus();
	tmpBuff[i++] = (uint8_t)(j);

	//外设状态
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 24);
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 16);
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 8);
	tmpBuff[i++] = (uint8_t)(SystemStatus.GetPeriphDeviceStatus());

	//执行头类型
	tmpBuff[i++] = ExecuterHead.MachineType;

	//CNC  转速
	tmpBuff[i++] = (uint8_t)(ExecuterHead.CNC.RPM >> 8);
	tmpBuff[i++] = (uint8_t)(ExecuterHead.CNC.RPM);

	//重填长度
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

#if(0)
#if ENABLED(SDSUPPORT)
/***********************************************
重新挂载U  盘
参数    Result:结果，0表示成功，非0表示失败
************************************************/
void HMI_SC20::SendInitUdisk(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}


/***********************************************
进入目录应答
参数    Result:结果，0表示成功，非0表示失败
************************************************/
void HMI_SC20::SendChDirResult(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x02;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}


/***********************************************
获取UDisk  当前目录应答
参数    Result:结果，0表示成功，非0表示失败
************************************************/
void HMI_SC20::SendCurrentUDiskPath(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x01;
	if(Result == 0)
	{
		j = 0;
		while(card.workdirpath[j] != 0)
			FileListBuff[i++] = card.workdirpath[j++];
	}
	FileListBuff[i++] = 0;
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}


/***********************************************
获取目录内容
参数    Offset:起始索引，表示本次第1个项目在遍历中的序号
返回值:  本次发送的个数
************************************************/
uint8_t HMI_SC20::SendDirItems(uint16_t Offset)
{
	char Res;
	uint8_t Count;
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	Count = 0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x04;
	//结果
	FileListBuff[i++] = 0x00;
	//Offset
	FileListBuff[i++] = (uint8_t)(Offset >> 8);
	FileListBuff[i++] = (uint8_t)Offset;
	//个数
	FileListBuff[i++] = 0x00;
	//一次最多传20  个，且总数据长度在缓冲大小之内
	while((Count < 10) && ((i + 257) < sizeof(FileListBuff)))
	{
		//读取目录 
		Res = card.getWorkDirItems();
		//类型
		if(Res == 0)
			FileListBuff[i++] = 0;
		else if(Res == 1)
			FileListBuff[i++] = 1;
		else
			break;
		j = 0;
		if(card.longFilename[0] != 0)
		{
			while(card.longFilename[j] != 0)
				FileListBuff[i++] = card.longFilename[j++];
		}
		else
		{
			while(card.filename[j] != 0)
				FileListBuff[i++] = card.filename[j++];
		}
		FileListBuff[i++] = 0;
		Count++;
	}

	//填结果
	if((Res == (char)-2) || (Res == (char)-1))
		FileListBuff[10] = 1;
		
	//重填个数
	FileListBuff[13] = Count;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);

	return Count;
}


/***********************************************
获取文件的特殊内容
************************************************/
void HMI_SC20::SendSpecialData()
{
	float tmpTamp, tmpTampBed;
	int32_t ContentLen;
	
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	uint8_t dataValid;
	
	tmpTamp = current_temperature[0];
	tmpTampBed = current_temperature_bed;

	card.openFile(FileListBuff, true);
	dataValid = 1;
	//文件打开成功
	if(card.isFileOpen() == true)
	{
		FileListBuff[0] = FileListBuff[1] = FileListBuff[2] = FileListBuff[3] = 0;
		card.ReadBytes((uint8_t*)FileListBuff, 20);
		if((FileListBuff[0] != 0x20) || (FileListBuff[1] != 0x18) || (FileListBuff[2] != 0x10) || (FileListBuff[3] != 0x01))
		{
			dataValid = 0;
		}
		else
		{
			//计算内容长度
			ContentLen = (uint32_t)((FileListBuff[19] << 24) | (FileListBuff[18] << 16) | (FileListBuff[17] << 8) | FileListBuff[16]) - 20;
		}
	}
	else
	{
		ContentLen = 0;
	}

	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x05;
	FileListBuff[i++] = dataValid;
	do
	{
		i = 11;
		//正常的特殊文件
		if(dataValid == 1)
		{
			if(ContentLen > 1024)
			{
				j = 1024;
			}
			else
			{
				j = ContentLen;
				FileListBuff[10] = 2;
			}
			card.ReadBytes((uint8_t*)&FileListBuff[i], j);
			i = i + j;
			ContentLen -= j;
			
		}
		else
		{
			FileListBuff[9] = 2;
		}
		FileListBuff[2] = (uint8_t)((i - 8) >> 8);
		FileListBuff[3] = (uint8_t)(i - 8);
		FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
		//校验
		checksum = 0;
		for(j = 8;j<(i - 1);j = j + 2)
			checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

		if((i - 8) % 2)
			checksum += FileListBuff[i-1];
		while(checksum > 0xffff)
			checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
		checksum = ~checksum;

		FileListBuff[6] = checksum >> 8;
		FileListBuff[7] = checksum;
		
		HmiWriteData(FileListBuff, i);
	}while(ContentLen > 0);
	card.closefile();
	current_temperature[0] = tmpTamp;
	current_temperature_bed = tmpTampBed;
}
#endif
#endif

/***********************************************
打印文件应答
参数    Result:结果，0表示启动成功，非0表示失败
************************************************/
void HMI_SC20::SendStartPrintReack(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}


//发送Gcode
void HMI_SC20::SendGcode(char *GCode, uint8_t EventID)
{
	uint8_t i, j;
	uint32_t checksum;
	i = 0;

	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = EventID;
	while(*GCode != 0)
		FileListBuff[i++] = *GCode++;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}

//发送继续打印
void HMI_SC20::SendContinuePrint()
{
	uint8_t i, j;
	uint32_t checksum;
	i = 0;
	
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x52;
	FileListBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 24);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 16);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 8);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.Data.FilePosition >> 0);
	//包长
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	HmiWriteData(FileListBuff, i);
}


#endif //ENABLED(HMI_SC20)
