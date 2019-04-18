#include "../inc/MarlinConfig.h"

#if ENABLED(HMI_SC20)

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
#include "../module/StatusControl.h"
#include "../module/LaserExecuter.h"
#include "../module/CNCexecuter.h"
#include "../module/PeriphDevice.h"



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


//调平点索引
uint8_t PointIndex;
float MeshPointZ[3 * 3];
uint16_t ZHomeOffsetIndex;


/**
 *SC20屏幕获取指令
 */
short HMI_SC20::GetCommand(unsigned char *pBuff)
{
    uint8_t *pReadBuff;
	uint16_t tmplen;
	uint16_t tmphead;
	uint16_t tmptail;
	uint16_t i;
    uint16_t BuffSize;
	uint32_t checksum;

	tmphead = ScreenOps.ReadHead;
	tmptail = ScreenOps.ReadTail;
	
	//没数据
	if (tmphead == tmptail)
	{
		return (short)-1;
	}

    BuffSize = ScreenOps.ReadBuffSize;
    pReadBuff = ScreenOps.pReadBuff;

	tmplen = (uint16_t)((tmphead + BuffSize - tmptail) % BuffSize);
	
	//数据长度足够
	while(tmplen > 9)
	{
		if(pReadBuff[tmptail] != 0xAA)
		{
			tmptail = (tmptail + 1) % BuffSize;
			tmplen--;
			//更新读指针
			ScreenOps.ReadTail = tmptail;
			continue;
		}
		if(pReadBuff[(tmptail + 1) % BuffSize] != 0x55)
		{
			tmptail = (tmptail + 2) % BuffSize;
			tmplen = tmplen - 2;
			//更新读指针
			ScreenOps.ReadTail = tmptail;
			continue;
		}
		//读取包长
		uint8_t cmdLen0 = pReadBuff[(tmptail + 2) % BuffSize];
		uint8_t cmdLen1 = pReadBuff[(tmptail + 3) % BuffSize];
		uint16_t commandLen = (uint16_t)((cmdLen0 << 8) | cmdLen1);
		//包长效验错误
		if((((commandLen >> 8) & 0xff) ^ (commandLen & 0xff)) != pReadBuff[(tmptail + 5) % BuffSize])
		{
			tmptail = (tmptail + 2) % BuffSize;
			tmplen = tmplen - 2;
			//更新读指针
			ScreenOps.ReadTail = tmptail;
			continue;
		}

		//缓冲数据足够
		if(commandLen <= (tmplen - 8))
		{
			//复制数据
			for(i=0;i<(commandLen + 8);i++)
			{
				pBuff[i] = pReadBuff[tmptail];
				tmptail = (tmptail + 1) % BuffSize;
			}

			//更新读指针
			ScreenOps.ReadTail = tmptail;

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

uint32_t UpdateDataSize;
uint8_t UpdateInProgress;
uint16_t UpdatePackRequest;
/********************************************************
启动升级
*********************************************************/
void HMI_SC20::StartUpdate(void)
{
    uint32_t Address;

    Address = FLASH_UPDATE_FLAG;
	//擦除FLASH
	FLASH_Unlock();
    for(int i=0;i<128;i++)
    {
        FLASH_ErasePage(Address);
        Address += 2048;
    }
    FLASH_Lock();
    HMI_SendStartUpdateReack(0);
    UpdateDataSize = 0;
    UpdateInProgress = 1;
    UpdatePackRequest = 0;
    SendUpdatePackRequest(UpdatePackRequest);
}




/********************************************************
平自动调平处理
*********************************************************/
static uint8_t CalibrateXIndeX[]={0, 1, 2, 2, 2, 1, 0, 0, 1};
static uint8_t CalibrateXIndeY[]={0, 0, 0, 1, 2, 2, 2, 1, 1};
void HMI_SC20::HalfAutoCalibrate(void)
{
	int i, j;
	int indexx, indexy;
	if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
	{
	    //请求执行头的开关状态
	    CanRequestIOSwichStatus(0);
		//关闭热床和加热头
		disable_all_heaters();
    strcpy(tmpBuff, "G28");
    parser.parse(tmpBuff);
    gcode.process_next_command();
		while(planner.movesplanned());
		mbl.active = false;

		//设置Z  轴最大速度
		strcpy(tmpBuff, "M203 Z50");
    parser.parse(tmpBuff);
    gcode.process_next_command();

		//设置海平面点的坐标
		current_position[Z_AXIS] = MaxPos[Z_AXIS];
		sync_plan_position();

		for(j=0;j<(GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y);j++)
		{
			//绝对模式
			relative_mode = false;
			//Z  轴移动到13mm
			strcpy(tmpBuff, "G0 Z7 F2000");
			parser.parse(tmpBuff);
      gcode.process_next_command();
			while(planner.movesplanned());

			//获取调平点索引值
			indexx = CalibrateXIndeX[j];
			indexy = CalibrateXIndeY[j];
			//X  Y  移动到第i  个调平点
			sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", mbl.get_x(indexx) - 11, mbl.get_y(indexy) - 13);
			parser.parse(tmpBuff);
      gcode.process_next_command();
			while(planner.movesplanned());

			//相对坐标模式
			relative_mode = true;
			HalfAutoCalibrateState = 1;
			strcpy(tmpBuff, "G0 Z-11 F70");
			parser.parse(tmpBuff);
      gcode.process_next_command();
			while(planner.movesplanned());
			current_position[Z_AXIS] = pCounter_Z / axis_steps_per_unit[Z_AXIS];
			MeshPointZ[indexy * GRID_MAX_POINTS_X + indexx] = current_position[Z_AXIS];
			HalfAutoCalibrateState = 0;
			sync_plan_position();
			//发送进度
			SettingReack(0x03, indexy * GRID_MAX_POINTS_X + indexx + 1);
		}

		//Zoffset
		relative_mode = false;
		strcpy(tmpBuff, "G0 Z7 F2000");
		parser.parse(tmpBuff);
    gcode.process_next_command();
		while(planner.movesplanned());

		sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F3000", (mbl.get_x(0) + mbl.get_x(GRID_MAX_POINTS_X - 1)) / 2.0f, (mbl.get_y(0) + mbl.get_y(GRID_MAX_POINTS_Y - 1)) / 2.0f);
		parser.parse(tmpBuff);
    gcode.process_next_command();
		while(planner.movesplanned());
		
		relative_mode = true;

		//设置Z  轴最大速度
		strcpy(tmpBuff, "M203 Z20");
		parser.parse(tmpBuff);
    gcode.process_next_command();
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
	if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
	{
	    //请求执行头的开关状态
	    CanRequestIOSwichStatus(0);
		//关闭热床和加热头
		disable_all_heaters();
		strcpy(tmpBuff, "G28");
		parser.parse(tmpBuff);
    gcode.process_next_command();
		while(planner.movesplanned());

		mbl.active = false;

		//绝对坐标模式
		relative_mode = false;
		/*
		sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F3000",home_offset[X_AXIS], home_offset[Y_AXIS]);
		parser.parse(tmpBuff);
    gcode.process_next_command();
		st_synchronize();

		strcpy(tmpBuff, "G0 Z0");
		parser.parse(tmpBuff);
    gcode.process_next_command();
		st_synchronize();
		*/
		
		//设置海平面点的坐标
		//限位开关在最高处
		if(AxisHomeDir[Z_AXIS] > 0)
			current_position[Z_AXIS] = MaxPos[Z_AXIS];
		//限位开关在最低处
		else
			current_position[Z_AXIS] = 0;
		sync_plan_position();

		//Z  轴移到20  的位置
		strcpy(tmpBuff, "G0 Z15");
		parser.parse(tmpBuff);
    gcode.process_next_command();
		while(planner.movesplanned());
		
		//相对模式
		relative_mode = true;

		//初始化
		PointIndex = 99;
		
		//屏幕锁定
		//HMICommandSave = 1;

		for(i=0;i<GRID_MAX_POINTS_Y;i++)
		{
			for(j=0;j<GRID_MAX_POINTS_X;j++)
			{
				MeshPointZ[i * GRID_MAX_POINTS_X + j] = mbl.z_values[i][j] - home_offset[Z_AXIS];
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
	char strCmd[50];
	uint32_t u32Value;
	int32_t int32Value;
	uint16_t j;

	//长宽高
	j = 0;
	//X
	u32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	MaxPos[X_AXIS] = u32Value / 1000.0f;
	j = j + 4;
	//Y
	u32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	MaxPos[Y_AXIS] = u32Value / 1000.0f;
	j = j + 4;
	//Z
	u32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	MaxPos[Z_AXIS] = u32Value / 1000.0f;
	j = j + 4;

	//回原点方向
	//X
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		AxisHomeDir[X_AXIS] = -1;
	else
		AxisHomeDir[X_AXIS] = 1;
	j = j + 4;
	
	//Y
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		AxisHomeDir[Y_AXIS] = -1;
	else
		AxisHomeDir[Y_AXIS] = 1;
	j = j + 4;
	
	//Z
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		AxisHomeDir[Z_AXIS] = -1;
	else
		AxisHomeDir[Z_AXIS] = 1;
  	j = j + 4;

	//电机方向
	//X
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		AxisDir[X_AXIS] = false;
	else
		AxisDir[X_AXIS] = true;
	j = j + 4;
	
	//Y
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		AxisDir[Y_AXIS] = false;
	else
		AxisDir[Y_AXIS] = true;
	j = j + 4;
	
	//Z
	int32Value = (pBuff[j] << 24) | (pBuff[j + 1] << 16) | (pBuff[j + 2] << 8) | (pBuff[j + 3]);
	if(int32Value < 0)
		AxisDir[Z_AXIS] = false;
	else
		AxisDir[Z_AXIS] = true;
  	j = j + 4;

	//E
	AxisDir[E_AXIS] = INVERT_E0_DIR;

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

	soft_endstop[X_AXIS] = MaxPos[X_AXIS];
	soft_endstop[Y_AXIS] = MaxPos[Y_AXIS];
	soft_endstop[Z_AXIS] = MaxPos[Z_AXIS];

  /*
	mbl.MinX = 20;
	mbl.MaxX = Maxpos[X_AXIS] + home_offset[X_AXIS] - mbl.MinX;
	mbl.MinY = 20;
	mbl.MaxY = MaxPos[Y_AXIS] + home_offset[Y_AXIS] - mbl.MinY;
	*/

	//相对模式
	relative_mode = true;
	//微动轴，更新方向
	strcpy(strCmd, "G0 X0.05 Y0.05 Z0.05 F1000");
	parser.parse(tmpBuff);
  gcode.process_next_command();
	while(planner.movesplanned());
	strcpy(strCmd, "G0 X-0.05 Y-0.05 Z-0.05 F1000");
	parser.parse(tmpBuff);
  gcode.process_next_command();
	while(planner.movesplanned());

	//绝对模式
	relative_mode = false;
	//保存数据
	settings.save();
}

/****************************************************
激光焦点设置启动
***************************************************/
void HMI_SC20::EnterLaserFocusSetting(void)
{
	bool tmpRelativeMode;
	char strCmd[50];

	//保存之前的运动模式
	tmpRelativeMode = relative_mode;
	//回原点
	strcpy(strCmd, "G28 Z");
	parser.parse(tmpBuff);
  gcode.process_next_command();
	//绝模式
	relative_mode = false;
	//走到特定高度
	strcpy(strCmd, "G0 Z20");
	parser.parse(tmpBuff);
  gcode.process_next_command();
	while(planner.movesplanned());
	//微光
	Laser.SetLaserLowPower();
	//恢得模式
	relative_mode = tmpRelativeMode;
}

void HMI_SC20::Process(void)
{
	static uint16_t ListFileOffset = 0;
	PrinterStatus CurStatus;
	uint32_t ID;
	float fCenterX, fCenterY; 
	uint32_t u32Value;
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
			ShieldLedOn();
			//指令尾补0
			j = cmdLen + 8;
			tmpBuff[j] = 0;
			Screen_enqueue_and_echo_commands(&tmpBuff[13], 0xffffffff, 0x02);
			
		}
		//GCode  打印
		else if(eventId == 0x03)
		{
		  
			ShieldLedOn();
			//获取当前状态
			CurStatus = SystemStatus.GetCurrentPrinterStatus();
			//上位启动打印
			if((CurStatus== STAT_RUNNING_ONLINE) || (CurStatus== STAT_PAUSE_ONLINE))
			{
				//激光头
				if(isLaserMachine(MachineSelected) == 1)
				{
					//行号
					ID = (tmpBuff[9] << 24) | (tmpBuff[10] << 16) | (tmpBuff[11] << 8) | tmpBuff[12];
					//指令尾补0
					j = cmdLen + 8;
					tmpBuff[j] = 0;
					
					#ifdef DOOR_PORT
					if(DoorCheckEnable == true)
					{
						//门已关闭
						if(READ(DOOR_PORT, DOOR_PIN) == false)
						{
							Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
						}
						else
						{
							SystemStatus.PauseTriggle(ManualPause);
							SetCurrentPrinterStatus(STAT_PAUSE_ONLINE);
						}
					}
					#else
					Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
					#endif
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
				lastlineNum = ID;
			}
		}
		//文件操作
		else if(eventId == 0x05)
		{
			ShieldLedOn();
			//控制字
			switch(tmpBuff[9])
			{
				//重新挂载
				case 0:
					card.initsd();
					if(card.cardOK == true)
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
					/*
					//激光执行头
					if(isLaserMachine(MachineSelected) == 1)
					{
						//Z  未知坐标
						if(HomeAfterReset[Z_AXIS] == false)
						{
							//Z  轴回原点
							strcpy(tmpBuff, "G28 Z");
							parser.parse(tmpBuff);
              gcode.process_next_command();
						}
						//走到工件坐标
						strcpy(tmpBuff, "G0 X0 Y0");
						parser.parse(tmpBuff);
            gcode.process_next_command();
						while(planner.movesplanned());
					}
					*/
					PowerPanicData.FilePosition = 0;
					PowerPanicData.accumulator = 0;
					PowerPanicData.HeaterTamp[0] = 0;
					PowerPanicData.BedTamp = 0;
					PowerPanicData.PositionData[0] = 0;
					PowerPanicData.PositionData[1] = 0;
					PowerPanicData.PositionData[2] = 0;
					PowerPanicData.PositionData[3] = 0;
					PowerPanicData.GCodeSource = GCODE_SOURCE_SCREEN;
					PowerPanicData.MachineType = ExecuterHead.MachineType;
					lastlineNum = 0;
					//使能断电检测
					EnablePowerPanicCheck();
					//使能检测
					#if((HAVE_FILAMENT_SENSOR == 1) || (HAVE_FILAMENT_SWITCH == 1))
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
						Periph.StartFilamentCheck();
					#endif
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
						Laser.SetLaserPower(0);
					SystemStatus.PauseTriggle(ManualPause);
				}
				else if(CurStatus == STAT_RUNNING_ONLINE)
				{
					HmiRequestStatus = STAT_PAUSE_ONLINE;
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
						Laser.SetLaserPower(0);
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
						#ifdef DOOR_PORT
						//关闭检测
						//DisableDoorCheck();
						//门开关检测使能
						if(Periph.GetDoorCheckFlag() == true)
						{
							//门已关闭
							if(READ(DOOR_PIN) == false)
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
						#else
						SystemStatus.PauseResume();
						#endif
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
					PowerPanicData.Valid = 0;
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
					PowerPanicData.Valid = 0;
				}
			}
			//请求最近行号
			else if(StatuID == 0x08)
			{
				//SendBreakPointLine();
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
				if(PowerPanicData.Valid == 1)
				{
					//清除Flash  有效位
					MaskBreadkPointData();
				}
				//标志断电续打无效
				PowerPanicData.Valid = 0;
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
						if((READ(DOOR_PIN) == false) && (CurStatus == STAT_IDLE))
						{
							//启动打印
							PowerPanicResumeWork(NULL);
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
						PowerPanicResumeWork(NULL);
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
						PowerPanicResumeWork(NULL);
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
						if((READ(DOOR_PIN) == false) && (CurStatus == STAT_IDLE))
						{
							//启动打印
							PowerPanicResumeWork(NULL);
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
						PowerPanicResumeWork(NULL);
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
						PowerPanicResumeWork(NULL);
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
					if((MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType) && (HeadProbeEnable == true))
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
						//相对模式
						do_blocking_move_to_z(current_position[Z_AXIS] + 5, 30);
						
						//绝对模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", mbl.get_x(PointIndex % GRID_MAX_POINTS_X), mbl.get_y(PointIndex / GRID_MAX_POINTS_Y));
						parser.parse(tmpBuff);
            gcode.process_next_command();
						while(planner.movesplanned());
						
						#if(HAVE_PROBE == 1)
						//使能调平传感器
						if(HeadProbeEnable == true)
						{
							HalfAutoCalibrateState = 1;
							//走到-5  位置
							do_blocking_move_to_z(-5.0, 0.2);
							
							//关闭探头触发
							HalfAutoCalibrateState = 0;
							//更新Z  轴坐标
							current_position[Z_AXIS] = pCounter_Z / planner.settings.axis_steps_per_mm[Z_AXIS];
							sync_plan_position();
						}
						//禁能调平传感器
						else
						{
							do_blocking_move_to_z(current_position[Z_AXIS] - 5, 0.2);
						}
						//相对模式
						relative_mode = true;
						#else
						//相对模式
						relative_mode = true;
						//下降5  mm
						do_blocking_move_to_z(current_position[Z_AXIS] - 5.0, 0.2);
						#endif
						
						//应答
						SettingReack(0x05, 0);
					}
				break;

				//Z  轴移动
				case 6:
					//保存原来状态
					if(relative_mode == false)
						i = 0;
					else
						i = 1;
					//相对模式
					relative_mode = true;
					int32Value = (tmpBuff[10] << 24) | (tmpBuff[11] << 16) | (tmpBuff[12] << 8) | (tmpBuff[13]);
					sprintf(tmpBuff, "G0 Z%0.2f F1000", (float)int32Value / 1000.0f);
					parser.parse(tmpBuff);
          gcode.process_next_command();
					if(i == 0)
						relative_mode = false;
					else
						relative_mode = true;

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
									mbl.set_z(j, i, MeshPointZ[i * GRID_MAX_POINTS_X + j] - MeshPointZ[4]);
							}

							//回原点
							strcpy(tmpBuff, "G28");
							parser.parse(tmpBuff);
              gcode.process_next_command();

							//保存数据
							settings.save();
							HMICommandSave = 0;
							//切换到绝对位置模式
							relative_mode = false;
							//清除标志
							CalibrateMethod = 0;
							//应答
							SettingReack(0x07, 0);
							//解除屏幕锁定
							HMICommandSave = 0;
						}
						//手动调平
						else if(CalibrateMethod == 2)
						{
							if(PointIndex != 99)
							{
								MeshPointZ[PointIndex] = current_position[Z_AXIS];

								//设置ZOffset
    							// 4  点调平
    							if((GRID_MAX_POINTS_X == 2) && (GRID_MAX_POINTS_Y == 2))
    							{
    							    //暂时设置调平值，为了计算中心点的Z轴高度，
    								for(i=0;i<GRID_MAX_POINTS_Y;i++)
    								{
    									for(j=0;j<GRID_MAX_POINTS_X;j++)
    										mbl.set_z(j, i, MeshPointZ[i * GRID_MAX_POINTS_X + j]);
    								}
    								//中心点坐标
    								float tmpCenterX, tmpCenterY, tmpCenterZ;
    								//计算中心坐标X
    								tmpCenterX = (mbl.get_x(0) + mbl.get_x(1)) / 2.0f;
    								//计算中心坐标Y
    								tmpCenterY = (mbl.get_y(0) + mbl.get_y(1)) / 2.0f;
                                    //计算中心坐标Z
    								tmpCenterZ = mbl.get_z(tmpCenterX, tmpCenterY);
    								//
    								sprintf(tmpBuff, "M206 Z%0.2f", tmpCenterZ);
                                    //重新计算4个调平点的值
                                    for(i=0;i<GRID_MAX_POINTS_Y;i++)
    								{
    									for(j=0;j<GRID_MAX_POINTS_X;j++)
    										mbl.set_z(j, i, MeshPointZ[i * GRID_MAX_POINTS_X + j] - tmpCenterZ);
    								}
    							}
    							//9  点调平
    							else
    							{
    							    //设置调平值
    								for(i=0;i<GRID_MAX_POINTS_Y;i++)
    								{
    									for(j=0;j<GRID_MAX_POINTS_X;j++)
    										mbl.set_z(j, i, MeshPointZ[i * GRID_MAX_POINTS_X + j]);
    								}
    								sprintf(tmpBuff, "M206 Z%0.2f", -current_position[Z_AXIS]);
    							}
    							parser.parse(tmpBuff);
              gcode.process_next_command();
								
								//回原点
								strcpy(tmpBuff, "G28");
								parser.parse(tmpBuff);
                gcode.process_next_command();

								//保存数据
								settings.save();
								//切换到绝对位置模式
								relative_mode = false;
								//清除标志
								CalibrateMethod = 0;
								//应答
								SettingReack(0x07, 0);
								//解除屏幕锁定
								HMICommandSave = 0;
							}
							//未作任何设置
							else
							{
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
						}
					}
				break;

				//退出调平点
				case 8:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Load
						settings.load();
						relative_mode = false;
						strcpy(tmpBuff, "G28");
						parser.parse(tmpBuff);
            gcode.process_next_command();
						while(planner.movesplanned());
	
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
					LoadLaserZValue(&LaserZValue);
					//
					SendLaserFocus(tmpBuff[9]);
				break;

				//设置激光Z  轴高度
				case 11:
					//关闭激光
					SetLaserPower(0);
					//当前坐标标为焦点高度
					LaserZValue = current_position[Z_AXIS];
					//保存
					SaveLaserZValue(LaserZValue);
					//读取
					LoadLaserZValue(&LaserZValue);
					//
					SendLaserFocus(tmpBuff[9]);
				break;

				//激光Z  轴回原点调焦专用
				case 12:
					//全部回原点
					strcpy(tmpBuff, "G28");
					parser.parse(tmpBuff);
          gcode.process_next_command();

					//调平数据失效
					mbl.active = false;

					//Z  轴移动到70  位置
					do_blocking_move_to_z(70, 40);

					//X  Y  移动到中心位置
					fCenterX = (MaxPos[X_AXIS] + home_offset[X_AXIS]) / 2.0f;
					fCenterY = (MaxPos[Y_AXIS] + home_offset[Y_AXIS]) / 2.0f;
					sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F2400", fCenterX, fCenterY);
					parser.parse(tmpBuff);
          gcode.process_next_command();
					while(planner.movesplanned());
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
					mbl.active = false;
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
		//ScreenOps.ReadTail = ReadHead;
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
			u32Value = (uint32_t)((LaserZValue + LaserFocus) * 1000.0f);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	u32Value = (uint32_t)(MaxPos[X_AXIS] * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	u32Value = (uint32_t)(MaxPos[Y_AXIS] * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	u32Value = (uint32_t)(MaxPos[Z_AXIS] * 1000);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	tmpBuff[i++] = PowerPanicData.Valid;
	tmpBuff[i++] = PowerPanicData.GCodeSource;
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 24);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 16);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 8);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag);

	//打印文件源
	if(SystemStatus.GetCurrentPrinterStatus() == STAT_IDLE)
		tmpBuff[i++] = 3;
	else
		tmpBuff[i++] = PowerPanicData.GCodeSource;

	//行号
	//tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	//tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	//tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	//tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
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
	fValue = pCounter_X / axis_steps_per_unit[X_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_Y / axis_steps_per_unit[Y_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_Z / axis_steps_per_unit[Z_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_E / axis_steps_per_unit[E_AXIS];
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
	tmpBuff[i++] = (uint8_t)(HmiFeedRate >> 8);
	tmpBuff[i++] = (uint8_t)(HmiFeedRate);

	//LaserPower
	tmpBuff[i++] = 0;
	tmpBuff[i++] = 0;
	tmpBuff[i++] = 0;
	tmpBuff[i++] = (uint8_t)(laserPercent);

	//RPM
	tmpBuff[i++] = 0;
	tmpBuff[i++] = 0;
	tmpBuff[i++] = (uint8_t)(CurRPM >> 8);
	tmpBuff[i++] = (uint8_t)(CurRPM);

	//打印机状态
	j = SystemStatus.GetCurrentPrinterStatus();
	tmpBuff[i++] = (uint8_t)(j);

	//外设状态
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 24);
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 16);
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 8);
	tmpBuff[i++] = (uint8_t)(SysStatusFlag);

	//执行头类型
	tmpBuff[i++] = ExecuterHead.MachineType;

	//CNC  转速
	tmpBuff[i++] = (uint8_t)(CurRPM >> 8);
	tmpBuff[i++] = (uint8_t)(CurRPM);

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
	
	ScreenOps.lpScreenWriteData(tmpBuff, i);
}


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
	
	ScreenOps.lpScreenWriteData(FileListBuff, i);
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
	
	ScreenOps.lpScreenWriteData(FileListBuff, i);
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
	
	ScreenOps.lpScreenWriteData(FileListBuff, i);
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
	
	ScreenOps.lpScreenWriteData(FileListBuff, i);

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
		
		ScreenOps.lpScreenWriteData(FileListBuff, i);
	}while(ContentLen > 0);
	card.closefile();
	current_temperature[0] = tmpTamp;
	current_temperature_bed = tmpTampBed;
}


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
	
	ScreenOps.lpScreenWriteData(FileListBuff, i);
}


//发送Gcode
void HMI_SC20::SC20SendGcode(char *GCode, uint8_t EventID)
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
	
	ScreenOps.lpScreenWriteData(FileListBuff, i);
}

//发送继续打印
void HMI_SC20::SendContinuePrint(void)
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
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 0);
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
	ScreenOps.lpScreenWriteData(FileListBuff, i);
}


#endif //ENABLED(HMI_SC20)