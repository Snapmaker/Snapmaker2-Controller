#include "../inc/MarlinConfig.h"

#if ENABLED(HMI_LONG)


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


#include "HMILong.h"

#if ENABLED(SDSUPPORT)	
#define IS_UDISK_INSERTED IS_SD_INSERTED()
#else
#define IS_UDISK_INSERTED false
#endif

//#define WaitingStepper() {while(blocks_queued()){manage_heater();process_next_command();}}

#define IsOnlineBusy()   (SystemStatus.GetCurrentPrinterStatus() == STAT_RUNNING_ONLINE || SystemStatus.GetCurrentPrinterStatus() == STAT_PAUSE_ONLINE)
#define IsOffLineBusy()   (SystemStatus.GetCurrentPrinterStatus() == STAT_RUNNING || SystemStatus.GetCurrentPrinterStatus() == STAT_PAUSE)


short HMILong::HmiGetCommand(void)
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

  if(tmphead == tmptail)
    return (short)-1;

  tmplen = (tmphead + sizeof(ReadBuff) - tmptail) % sizeof(ReadBuff);

  //数据长度足够
	while(tmplen > 8)
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
		uint8_t commandLen = ReadBuff[(tmptail + 2) % sizeof(ReadBuff)];

		//缓冲数据足够
		if(commandLen <= (tmplen - 4))
		{
			//复制数据
			for(i=0;i<(commandLen + 4);i++)
			{
				tmpBuff[i] = ReadBuff[tmptail];
				tmptail = (tmptail + 1) % sizeof(ReadBuff);
			}

			//更新读指针
			ReadTail = tmptail;

			//校验
			checksum = 0;
			for(i=0;i<commandLen;i++)
				checksum = checksum + tmpBuff[i + 3];
			checksum = 0x100 - checksum;
      checksum = checksum & 0xff;

			if((uint8_t)checksum != (uint8_t)tmpBuff[commandLen + 3])
				return (short)-1;
      
			return (short)(commandLen + 4);
		}
		//数据长度不足
		else
		{
			return (short)-1;
		}
		
	}
	return (short)-1;
  
}

void HMILong::PollingCommand(void)
{
  static uint8_t ShowFileCount = 0;
	static uint8_t SeekFileCount = 0;
	static uint8_t SeekingFile = 0;
	static uint8_t ShowFileIndex = 0;
  float xGridSpacing, yGridSpacing;
	uint32_t ID;
	uint16_t SetTamp;
	uint8_t eventId;
  short i;
  
  i = HmiGetCommand();

  if(i > 0)
  {
		eventId = tmpBuff[7];
		//按钮
		if(eventId == 1)
		{
			ID = (uint16_t)(tmpBuff[3] << 8) | (uint8_t)tmpBuff[4];
			switch((uint16_t)ID)
			{
				case BUTTON_MOTOR_HOME:
					//电机不在同步中
					planner.synchronize();
					ChangePage(8);
					set_bed_leveling_enabled(true);
					gcode.home_all_axes();

					relative_mode = false;

          do_blocking_move_to_z(0, 1000);
					ChangePage(0);
				break;

				//换料
				case BUTTON_CHANGE_FILAMENT:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
						ChangePage(PAGE_CHANGEFILAMENT);
				break;

				//停止加热
				case BUTTON_STOP_HEAT:
				case BUTTON_END_HEAD:
				case BUTTON_STOP_HEAT_2:
          thermalManager.setTargetHotend(0, 0);
					ChangePage(PAGE_CONTROL);
				break;

				case BUTTON_CANCEL_CHANGE_FILAMENT:
          thermalManager.setTargetHotend(0, 0);
					ChangePage(PAGE_CONTROL);
				break;
				
				case BUTTON_CONFIRM_CHANGE_FILAMENT:
          thermalManager.setTargetHotend(200, 0);
				  ChangePage(PAGE_WAITHEAT);
				break;

				//温度加
				case BUTTON_TAMP_INC:
					SetTamp = (int)thermalManager.temp_hotend[0].target;
					if(SetTamp < 250)
					{
						SetTamp = SetTamp + 10;
						thermalManager.setTargetHotend((int16_t)SetTamp, 0);
						ShowChangeFilamentTamp();
					}
				break;

				//温度减
				case BUTTON_TAMP_DEC:
					SetTamp = (int)thermalManager.temp_hotend[0].target;
					if(SetTamp > 200)
					{
						SetTamp = SetTamp - 10;
						thermalManager.setTargetHotend((int16_t)SetTamp, 0);
						ShowChangeFilamentTamp();
					}
				break;

				case BUTTON_FILE1:
					if(CardFileName[0][0] != 0)
					{
						strcpy(tmpCardFileNameSelected, CardFileName[0]);
						strcpy(PrintFileNameSelected, PrintFileName[0]);
						//跳转到确认对话框
						ChangePage(PAGE_CONFIRMPRINT);
					}
				break;

				case BUTTON_FILE2:
					if(CardFileName[1][0] != 0)
					{
						strcpy(tmpCardFileNameSelected, CardFileName[1]);
						strcpy(PrintFileNameSelected, PrintFileName[1]);
						//跳转到确认对话框
						ChangePage(PAGE_CONFIRMPRINT);
					}
				break;

				case BUTTON_FILE3:
					if(CardFileName[2][0] != 0)
					{
						strcpy(tmpCardFileNameSelected, CardFileName[2]);
						strcpy(PrintFileNameSelected, PrintFileName[2]);
						//跳转到确认对话框
						ChangePage(PAGE_CONFIRMPRINT);
					}
				break;

				case BUTTON_FILE4:
					if(CardFileName[3][0] != 0)
					{
						strcpy(tmpCardFileNameSelected, CardFileName[3]);
						strcpy(PrintFileNameSelected, PrintFileName[3]);
						//跳转到确认对话框
						ChangePage(PAGE_CONFIRMPRINT);
					}
				break;

				case BUTTON_FILE5:
					if(CardFileName[4][0] != 0)
					{
						strcpy(tmpCardFileNameSelected, CardFileName[4]);
						strcpy(PrintFileNameSelected, PrintFileName[4]);
						//跳转到确认对话框
						ChangePage(PAGE_CONFIRMPRINT);
					}
				break;

				case BUTTON_FILE6:
					if(CardFileName[5][0] != 0)
					{
						strcpy(tmpCardFileNameSelected, CardFileName[5]);
						strcpy(PrintFileNameSelected, PrintFileName[5]);
						//跳转到确认对话框
						ChangePage(PAGE_CONFIRMPRINT);
					}
				break;

				case 0x0a2f:
				case BUTTON_FILE_CONFIRM_UP:
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
						ChangePage(PAGE_PRINT);
					else
						ChangePage(PAGE_CNC);
						
					//清除文件个数
					FileCounts = 0;
					//开始打印选择的文件
					strcpy(CardFileNameSelected, tmpCardFileNameSelected);
					card.openFile(PrintFileNameSelected, true);
					print_job_timer.start();
          //card.LastPercent = 0;
					ShowPrintTime();
					ShowProgressBar(0);
					
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
					{
						//门已关闭
						#ifdef DOOR_PORT
						if(Periph.GetDoorCheckFlag() == true)
						{
							if(READ(DOOR_PIN) == false)
							{
								card.startFileprint();
							}
							else
							{
								SystemStatus.PauseTriggle(ManualPause);
							}
						}
						else
						{
							card.startFileprint();
						}
						#else
						  card.startFileprint();
						#endif
					}
					else if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
					{
						card.startFileprint();
					}
					else
					{
						card.startFileprint();
					}
				break;

				case BUTTON_FILE_CANCEL_UP:
					PrintFileNameSelected[0] = 0;
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
						ChangePage(PAGE_PRINT);
					else
						ChangePage(PAGE_CNC);
					//清除文件个数
					FileCounts = 0;
				break;
				

				//文件显示上一页
				case BUTTON_NEXT_PAGE_UP:
					//没有文件或正在搜文件中
					if((FileCounts == 0) || (SeekingFile == 1) || (ShowFileCount > 0))
					{
						break;
					}
					if(CurFilePage < (FilePages - 1))
					{
						CurFilePage++;
						FileIndex = CurFilePage * FILE_PER_PAGE;
						//最后一页
						if(CurFilePage == (FilePages - 1))
						{
							//非整数
							if((FileCounts % FILE_PER_PAGE) != 0)
								SeekFileCount = (FileCounts % FILE_PER_PAGE);
							else
								SeekFileCount = FILE_PER_PAGE;
						}
						else
						{
							SeekFileCount = FILE_PER_PAGE;
						}
						//清空列表
						for(i=0;i<FILE_PER_PAGE;i++)
						{
							CardFileName[i][0] = 0;
							PrintFileName[i][0] = 0;
						}
						//标识需要查找文件
						SeekingFile = 1;
						//显示文件个数与查找个数相同
						ShowFileCount = FILE_PER_PAGE;
						//显示文件索引从0开始
						ShowFileIndex = 0;
					}
				break;


				//显示文件下一页
				case BUTTON_PREV_PAGE_UP:
					//没有文件或正在搜文件中
					if((FileCounts == 0) || (SeekingFile == 1) || (ShowFileCount > 0))
					{
						break;
					}
					if(CurFilePage > 0)
					{
						CurFilePage--;
						FileIndex = CurFilePage * FILE_PER_PAGE;
						if(FileCounts > FILE_PER_PAGE)
						{
							SeekFileCount = FILE_PER_PAGE;
						}
						else
						{
							SeekFileCount = FileCounts;
						}
						//清空列表
						for(i=0;i<FILE_PER_PAGE;i++)
						{
							CardFileName[i][0] = 0;
							PrintFileName[i][0] = 0;
						}
						//标识需要查找文件
						SeekingFile = 1;
						//显示文件个数与查找个数相同
						ShowFileCount = FILE_PER_PAGE;
						//显示文件索引从0开始
						ShowFileIndex = 0;
					}
				break;

				//退出文件选择
				case BUTTON_SELECT_FILE_BACK:
				case BUTTON_SELECT_FILE_HOME:
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
						ChangePage(PAGE_PRINT);
					else
						ChangePage(PAGE_CNC);
					//清除文件个数
					FileCounts = 0;
				break;

				//退出控制界面
				case BUTTON_CONTROL_BACK:
				case BUTTON_CNC_CONTROL_BACK:
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
					{
						ChangePage(PAGE_PRINT);
					}
					else if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
					{
						ExecuterHead.Laser.SetLaserPower((uint16_t)0);
						ChangePage(PAGE_LASER);
					}
					else if(MACHINE_TYPE_CNC == ExecuterHead.MachineType)
					{
						ExecuterHead.CNC.SetCNCPower(0);
						ChangePage(PAGE_CNC);
					}
				break;

				//OpenFile按键
				case BUTTON_PRINT_SHOW_FILE:
				case BUTTON_CNC_SHOW_FILE:
          #if ENABLED(SDSUPPORT)
						if(SystemStatus.GetCurrentPrinterStatus() == STAT_IDLE)
						{
							if(IS_UDISK_INSERTED)
							{
								card.initsd();
								SERIAL_ECHOPGM("Card Insert!\r\n");
								if(card.isDetected())
								{
									SERIAL_ECHOPGM("Card Inited!\r\n");
									ChangePage(PAGE_FILELIST);
								}
							}
						}
						else
						{
							SERIAL_ECHOPGM("Sd is printing\r\n");
						}	
				  #endif
				break;

				//移动轴
				case BUTTON_MOVE_AXIS:
					//电机不在同步中
					ChangePage(PAGE_MOVE100);
					relative_mode = true;
          //HMICommandSave = 1;
				break;

				//移动轴退出
				case BUTTON_MOVE_AXIS_BACK_001:
				case BUTTON_MOVE_AXIS_BACK_010:
				case BUTTON_MOVE_AXIS_BACK_100:
					//电机不在同步中
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
						{
							ChangePage(PAGE_CONTROL);
						}
						else
						{
							ChangePage(PAGE_CNCCONTROL);
							if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
							{
								//关闭激光，CNC
								ExecuterHead.Laser.SetLaserPower((uint16_t)0);
								//LastLaserPwm = 0;             
 							}
						}
						relative_mode = false;
						//HMICommandSave = 0;
					}
				break;
				
				case BUTTON_MOVE_AXIS_HOME_001:
				case BUTTON_MOVE_AXIS_HOME_010:
				case BUTTON_MOVE_AXIS_HOME_100:
					//电机不在同步中
						if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
						{
							ChangePage(PAGE_PRINT);
						}
						else
						{
							ChangePage(PAGE_CNC);
							if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
							{
								//关闭激光，CNC
								ExecuterHead.Laser.SetLaserPower((uint16_t)0);
								//LastLaserPwm = 0;
							}
							else if(MACHINE_TYPE_CNC != ExecuterHead.MachineType)
							{
								ExecuterHead.CNC.SetCNCPower(0);
							}
						}

						relative_mode = false;
						//HMICommandSave = 0;
					
				break;
				
				//负方向移动
				case BUTTON_XL_01:
          do_blocking_move_to(current_position[X_AXIS] - 0.1f, current_position[Y_AXIS], current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_YL_01:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS] - 0.1f, current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_ZL_01:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] - 0.1f, 50.0f);
				break;

				case BUTTON_XL_10:
					do_blocking_move_to(current_position[X_AXIS] - 1, current_position[Y_AXIS], current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_YL_10:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS] - 1, current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_ZL_10:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] - 1, 50.0f);
				break;

				case BUTTON_XL_100:
					do_blocking_move_to(current_position[X_AXIS] - 10, current_position[Y_AXIS], current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_YL_100:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS] - 10, current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_ZL_100:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] - 10, 50.0f);
				break;

				//正方向移动
				case BUTTON_XR_01:
					do_blocking_move_to(current_position[X_AXIS] + 0.1, current_position[Y_AXIS], current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_YR_01:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS] + 0.1, current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_ZR_01:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + 0.1, 50.0f);
				break;

				case BUTTON_XR_10:
					do_blocking_move_to(current_position[X_AXIS] + 1, current_position[Y_AXIS], current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_YR_10:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS] + 1, current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_ZR_10:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + 1, 50.0f);
				break;

				case BUTTON_XR_100:
					do_blocking_move_to(current_position[X_AXIS] + 10, current_position[Y_AXIS], current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_YR_100:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS] + 10, current_position[Z_AXIS], 50.0f);
				break;

				case BUTTON_ZR_100:
					do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + 10, 50.0f);
				break;

				//控制
				case BUTTON_CONTROLL:
					#if ENABLED(SDSUPPORT)	
						if(card.isFileOpen() == false)
						{
							ChangePage(PAGE_CONTROL);
						}
					#else
						ChangePage(PAGE_CONTROL);
					#endif
				break;

				//设置
				case BUTTON_SETTING:
					#if ENABLED(SDSUPPORT)	
						if(card.isFileOpen() == false)
						{
							ChangePage(PAGE_SETTING);
						}
					#else
						ChangePage(PAGE_SETTING);
					#endif
				break;

				case BUTTON_SETTING_BACK:
					ChangePage(PAGE_PRINT);
				break;

				//Z Offset
				case BUTTON_ZOFFSET_SETUP:
					planner.synchronize();
          MeshPointZ[0] = z_values[0][0];
					MeshPointZ[2] = z_values[1][0];
					MeshPointZ[1] = z_values[0][1];
					MeshPointZ[3] = z_values[1][1];
          
					ChangePage(PAGE_ZCENTER);
 					gcode.home_all_axes();
          set_bed_leveling_enabled(false);
          
          do_blocking_move_to_z(MANUAL_PROBE_START_Z);

					//初始化之后为基准点
					PointIndex = 2;
					
					//HMICommandSave = 1;
					ChangePage(PAGE_CALIBRATE1);
				break;

				case BUTTON_ZOFFSET_HOME:
				case BUTTON_CALIBRATE1_BACK:
				case BUTTON_CALIBRATE2_BACK:
				case BUTTON_CALIBRATE3_BACK:
				case BUTTON_ZOFFSET_BACK:
          settings.load();
				case BUTTON_CALIBRATE1_SAVE:
				case BUTTON_CALIBRATE2_SAVE:
				case BUTTON_CALIBRATE3_SAVE:
				case BUTTON_ZOFFSET_CONFIRM:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//切换等待界面
						ChangePage(PAGE_ZCENTER);
            
            sprintf(tmpBuff, "G29 W I0 J0 Z%0.3f", MeshPointZ[0]);
            process_cmd_imd(tmpBuff);

            sprintf(tmpBuff, "G29 W I1 J0 Z%0.3f", MeshPointZ[1]);
            process_cmd_imd(tmpBuff);

            sprintf(tmpBuff, "G29 W I0 J1 Z%0.3f", MeshPointZ[2]);
            process_cmd_imd(tmpBuff);

            sprintf(tmpBuff, "G29 W I1 J1 Z%0.3f", MeshPointZ[3]);
            process_cmd_imd(tmpBuff);

            do_blocking_move_to_z(current_position[Z_AXIS] + 5, 30);

            //保存数据
						settings.save();
            //回原点
						gcode.home_all_axes();
            
						set_bed_leveling_enabled(true);
						
						//HMICommandSave = 0;
						//切换到绝对位置模式
						relative_mode = false;

						do_blocking_move_to_z(0);
						//切换主界面
						ChangePage(PAGE_PRINT);
					}
				break;
				
				case BUTTON_ZOFFSET_UP:
          do_blocking_move_to_z(current_position[Z_AXIS] + 0.05f);
					MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				case BUTTON_ZOFFSET_DOWN:
					do_blocking_move_to_z(current_position[Z_AXIS] - 0.05f);					
					MeshPointZ[PointIndex] = current_position[Z_AXIS];		
				break;

				case BUTTON_CALIBRATE_UP_005:
					do_blocking_move_to_z(current_position[Z_AXIS] + 0.05f);
					MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				case BUTTON_CALIBRATE_UP_020:
					do_blocking_move_to_z(current_position[Z_AXIS] + 0.2f);
					MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				case BUTTON_CALIBRATE_UP_050:
					do_blocking_move_to_z(current_position[Z_AXIS] + 0.5f);
					MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				case BUTTON_CALIBRATE_DOWN_005:
					do_blocking_move_to_z(current_position[Z_AXIS] - 0.05f);
					MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				case BUTTON_CALIBRATE_DOWN_020:
					do_blocking_move_to_z(current_position[Z_AXIS] - 0.2f);
					MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				case BUTTON_CALIBRATE_DOWN_050:
          do_blocking_move_to_z(current_position[Z_AXIS] - 0.5f);
				  MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				//重置调平参数
				case BUTTON_CALIBRATE1_RESET:
				case BUTTON_CALIBRATE2_RESET:
				case BUTTON_CALIBRATE3_RESET:
					ChangePage(PAGE_RESETCALIBRATE);
				break;

				//确定重置调平参数
				case BUTTON_CALIBRATE_RESET_CONFIRM:
					ChangePage(PAGE_WAITHOME);
					z_values[0][0] = MANUAL_PROBE_START_Z;
					z_values[1][1] = MANUAL_PROBE_START_Z;
					z_values[1][0] = MANUAL_PROBE_START_Z;
					z_values[0][1] = MANUAL_PROBE_START_Z;
          settings.save();
    
          MeshPointZ[0] = MANUAL_PROBE_START_Z;
					MeshPointZ[1] = MANUAL_PROBE_START_Z;
					MeshPointZ[2] = MANUAL_PROBE_START_Z;
					MeshPointZ[3] = MANUAL_PROBE_START_Z;
          
					gcode.home_all_axes();

					//失效调平补尝
					set_bed_leveling_enabled(false);
					//绝对坐标模式
					relative_mode = false;

					//回到初始化位置
					do_blocking_move_to_z(MANUAL_PROBE_START_Z);
					
					ChangePage(PAGE_CALIBRATE1);

					//初始化之后为基准点
					PointIndex = 2;
					
					//HMICommandSave = 1;
					ChangePage(PAGE_CALIBRATE1);
				break;

				//取消重置调平参数
				case BUTTON_CALIBRATE_RESET_BACK:
					ChangePage(PAGE_CALIBRATE1);
				break;

				case BUTTON_CALIBRATE1_P1:
				case BUTTON_CALIBRATE2_P1:
				case BUTTON_CALIBRATE3_P1:
				case BUTTON_ZOFFSET_P1:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//向上走2  MM
						do_blocking_move_to_z(current_position[Z_AXIS] + 2, 50.0f);
						//向到调整点位置
						int X, Y;
						X = LEFT_PROBE_BED_POSITION;
						Y = BACK_PROBE_BED_POSITION;
            do_blocking_move_to(X, Y, current_position[Z_AXIS], 50.0f);
            
            do_blocking_move_to_z(MeshPointZ[2]);		
						
						//对应M421 S2
						PointIndex = 2;
					}
				break;

				case BUTTON_CALIBRATE1_P2:
				case BUTTON_CALIBRATE2_P2:
				case BUTTON_CALIBRATE3_P2:
				case BUTTON_ZOFFSET_P2:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//向上走2  MM
						do_blocking_move_to_z(current_position[Z_AXIS] + 2, 50.0f);

						//向到调整点位置
						int X, Y;
						X = RIGHT_PROBE_BED_POSITION;
						Y = BACK_PROBE_BED_POSITION;
						do_blocking_move_to(X, Y, current_position[Z_AXIS], 50.0f);

						do_blocking_move_to_z(MeshPointZ[3]);
						//对应M421 S3
						PointIndex = 3;
					}
				break;

				case BUTTON_CALIBRATE1_P3:
				case BUTTON_CALIBRATE2_P3:
				case BUTTON_CALIBRATE3_P3:
				case BUTTON_ZOFFSET_P3:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//向上走2  MM
						do_blocking_move_to_z(current_position[Z_AXIS] + 2, 50.0f);
            
						//向到调整点位置
						int X, Y;
						X = RIGHT_PROBE_BED_POSITION;
						Y = FRONT_PROBE_BED_POSITION;
            do_blocking_move_to(X, Y, current_position[Z_AXIS], 50.0f);

						do_blocking_move_to_z(MeshPointZ[1]);
						//对应M421 S1
						PointIndex = 1;
					}
				break;

				case BUTTON_CALIBRATE1_P4:
				case BUTTON_CALIBRATE2_P4:
				case BUTTON_CALIBRATE3_P4:
				case BUTTON_ZOFFSET_P4:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//向上走2  MM
						do_blocking_move_to_z(current_position[Z_AXIS] + 2, 50.0f);
						
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = LEFT_PROBE_BED_POSITION;
						Y = FRONT_PROBE_BED_POSITION;
						do_blocking_move_to(X, Y, current_position[Z_AXIS], 50.0f);

						do_blocking_move_to_z(MeshPointZ[0]);
						//对应M421 S0
						PointIndex = 0;
					}
				break;

				//停止打印
				case BUTTON_STOP_PRINT:
					#if ENABLED(SDSUPPORT)	
						//SD  卡打印中
						if(card.isFileOpen() == true)
						{
							//切换到停止界面
							ChangePage(PAGE_CONFIRMSTOP);
						}
					#endif
				break;
				
				//停止打印确认
				case BUTTON_STOP_PRINT_CONFIRM:
          //该按钮只在停止工作界面中有效
          //if(CurrentHMIPage == PAGE_CONFIRMSTOP)
          {
						//脱机打印
						if(SystemStatus.GetCurrentPrinterStatus() == STAT_RUNNING)
            {      
              if(card.isFileOpen() == true)
              {
  					    card.stopSDPrint();
  							SystemStatus.StopTriggle(ManualStop);
  							ChangePage(PAGE_PROCESSING);
  							//禁能断电检测
  							//DisablePowerPanicCheck();
  						}
            }
						//联机打印
						else if(SystemStatus.GetCurrentPrinterStatus() == STAT_RUNNING_ONLINE)
						{
							if(MACHINE_TYPE_CNC != ExecuterHead.MachineType)
								thermalManager.disable_all_heaters();
							wait_for_heatup = true;
							SystemStatus.StopTriggle(ManualStop);
							ChangePage(PAGE_PROCESSING);
							//禁能断电检测
							//DisablePowerPanicCheck();
						}
					}
				break;

				//停止打印取消
				case BUTTON_STOP_PRINT_CANCEL:
					//该按钮只在停止工作界面中有效
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
					{
						ChangePage(PAGE_PRINT);
					}
					else
					{
						ChangePage(PAGE_CNC);
					}
				break;

				//暂停打印
				case BUTTON_PAUSE_PRINT:
          if(SystemStatus.GetCurrentPrinterStatus() == STAT_RUNNING)
          {
  					//文件已打开
  					if(card.isFileOpen() == true)
  					{
  						SystemStatus.PauseTriggle(ManualPause);
  						ChangePage(PAGE_PROCESSING);
  					}
          }
          else if(SystemStatus.GetCurrentPrinterStatus() == STAT_RUNNING_ONLINE)
          {
            
          }
				break;

				//继续打印
				case BUTTON_CONTINUE_PRINT:
					//文件已打开
					if(SystemStatus.GetCurrentPrinterStatus() == STAT_RUNNING)
          {     
  					if(card.isFileOpen() == true)
  					{
  					  ChangePage(PAGE_PROCESSING);
  					}
          }
				break;

				//CNC 
				case BUTTON_CNC_PAUSE:
					//U  盘打印
					if(card.isFileOpen() == true)
					{
						//关闭检测
						Periph.SetDoorCheck(false);
						SystemStatus.PauseTriggle(ManualPause);
						//current_block = NULL;
						planner.clear_block_buffer();
						//激光关闭
						//SetLaserPower(0);
						//开启检测
						if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
							Periph.SetDoorCheck(false);
					}
				break;

				case BUTTON_CNC_RUN:
					//if(card.isFileOpen() == true)
					{
						//CNC  功能
						if(MACHINE_TYPE_CNC == ExecuterHead.MachineType)
						{
							if(SystemStatus.GetCurrentPrinterStatus() ==  STAT_PAUSE)
								SystemStatus.PauseResume();
						}
						//激光功能
						else
						{
							if(SystemStatus.GetCurrentPrinterStatus() ==  STAT_PAUSE)
							{
									//关闭检测
                  Periph.StopDoorCheck();
									if(Periph.GetDoorCheckFlag() == true)
									{
										SystemStatus.PauseResume();
									}
									else 
									{
										if(Periph.IsDoorOpened() == false)
										{
											SystemStatus.PauseResume();
										}
									}
									//开启检测
									Periph.StartDoorCheck();
							}
							else
								SystemStatus.PauseResume();
						}
					}
				break;

				case BUTTON_CNC_STOP:
					#if ENABLED(SDSUPPORT)	
						//打印中
						if(IsOffLineBusy() == true)
            {      
  						if(card.isFileOpen() == true)
  						{
  							ChangePage(PAGE_CONFIRMSTOP);
  						}
            }
					#endif
				break;

				case BUTTON_CNC_CONTROL:
					#if ENABLED(SDSUPPORT)	
						if(card.isFileOpen() == false)
							ChangePage(PAGE_CNCCONTROL);
					#else
						ChangePage(PAGE_CNCCONTROL);
					#endif
				break;
        

				case BUTTON_CNC_ON:
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
					{
					  if(Periph.GetDoorCheckFlag() == true)
            {
              if(Periph.IsDoorOpened() == true)
              {
                //ExecuterHead.Laser.SetLaserPower();
              }
            }
            else
            {
              //ExecuterHead.Laser.SetLaserPower();
            }
					}
					else if(MACHINE_TYPE_CNC == ExecuterHead.MachineType)
            ExecuterHead.CNC.SetCNCPower(100);
				break;

				case BUTTON_CNC_OFF:
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
						ExecuterHead.Laser.SetLaserPower((uint16_t)0);
					else if(MACHINE_TYPE_CNC == ExecuterHead.MachineType)
						ExecuterHead.CNC.SetCNCPower(0);
				break;

				case BUTTON_CNC_SET_ORIGIN:
					strcpy(tmpBuff, "G92 X0 Y0 Z0");
          process_cmd_imd(tmpBuff);
					ChangePage(PAGE_ORIGINSAVED);
					millis_t tick;
					tick = millis();
					while(millis() < (tick + 800));
					ChangePage(PAGE_CNCCONTROL);
					//设置原点之后，保持电流
					//StepperKeepCurrent = true;
					enable_X();
					enable_Y();
					enable_Z();
				break;

				case BUTTON_CNC_MOVE_AXIS:
					if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
					{
						ExecuterHead.Laser.SetLaserLowPower();
					}
					ChangePage(PAGE_MOVE100);
				break;

				//FAQ
				case BUTTON_FAQ:
					ChangePage(PAGE_FAQ);
				break;

				//FAQ  返回
				case BUTTON_FAQ_BACK:
					ChangePage(PAGE_SETTING);
				break;

				//FAQ  回主页
				case BUTTON_FAQ_HOME:
					ChangePage(PAGE_PRINT);
				break;

				//About
				case BUTTON_ABOUT:
					ChangePage(PAGE_ABOUT);
				break;

				//About  界面返回
				case BUTTON_ABOUT_BACK:
					ChangePage(PAGE_SETTING);
				break;

				//About  界面回主页
				case BUTTON_ABOUT_HOME:
					ChangePage(PAGE_PRINT);
				break;

				//About  界面升级屏幕
				case BUTTON_ABOUT_UPDATE:
					//SendScreenUpdate();
				break;
				
				default:
					
				break;
			}
		}
		//界面初始化
		else if(eventId == 2)
		{
			ID = (uint16_t)tmpBuff[3];

			uint8_t i;
			CurrentHMIPage = 0xff;
			//获取界面ID
			for(i=0;i<sizeof(PageID);i++)
			{
				if(ID == PageID[i])
				{
					CurrentHMIPage = i;
					break;
				}
			}

			//保存最后界面ID
			switch(CurrentHMIPage)
			{
				case PAGE_PRINT:
					#if ENABLED(SDSUPPORT)	
						//SD  打印
						if(card.isFileOpen()== true)
						{
							StopEnable(true);
							SetControlEnable(false);
							SetSettingEnable(false);
							//本次检测被拔出来
							if(!(card.isDetected()))
							{
								//设置图标
								SetTFIcon(false);
							}
							else
							{
								//设置图标
								SetTFIcon(true);
							}
							//正在打印
							if(card.isPrinting() == true)
							{
								ChangePrinterStatus(1);
							}
							//暂停打印
							else
							{
								ChangePrinterStatus(2);
							}
						}
						//没有SD  打印
						else
						{
							//本次检测被拔出来
							if(!(card.isDetected()))
							{
								//设置图标
								SetTFIcon(false);
							}
							else
							{
								//设置图标
								SetTFIcon(true);
							}
						}
						ShowPrintTime();
						ShowPrintFileName(CardFileNameSelected);
						//ShowProgressBar(card.LastPercent);
					#endif
				break;

				case PAGE_ABOUT:
					SetAboutLabels();
				break;

				case PAGE_CNC:
          #if ENABLED(SDSUPPORT)	
					//SD  打印
					if(card.isFileOpen()== true)
					{
						StopEnable(true);
						PauseEnable(true);
						SetControlEnable(false);
						ChangePrinterStatus(1);
					}
					else
					{
						//本次检测被拔出来
						if(!(card.isDetected()))
						{
							//设置图标
							SetTFIcon(false);
						}
						else
						{
							//设置图标
							SetTFIcon(true);
						}
					}
					//显示功率
					ShowPrintTime();
					//ShowLaserPower(laserPercent);
					ShowPrintFileName(CardFileNameSelected);
					//ShowProgressBar(card.LastPercent);
					#endif
				break;
				
				case PAGE_FILELIST:
				  #if ENABLED(SDSUPPORT)	
						//文件选择对话框初始化
						if(FileCounts == 0)
						{
							if(card.isDetected())
							{
								//FileCounts = card.getnrfilenames(FileFilter);
								FileCounts = card.getnrfilenames();
								FilePages = FileCounts / FILE_PER_PAGE;
								if((FileCounts % FILE_PER_PAGE) != 0)
									FilePages++;
								CurFilePage = 0;
							}
						}
						FileIndex = CurFilePage * FILE_PER_PAGE;
						if(FileCounts > FILE_PER_PAGE)
							SeekFileCount = FILE_PER_PAGE;
						else
							SeekFileCount = FileCounts;
						//清空列表
						for(i=0;i<FILE_PER_PAGE;i++)
						{
							CardFileName[i][0] = 0;
							PrintFileName[i][0] = 0;
						}
						//显示文件个数与查找个数相同
						ShowFileCount = FILE_PER_PAGE;
						//标识需要查找文件
						SeekingFile = 1;
						//显示文件索引从0开始
						ShowFileIndex = 0;		
					#endif
				break;

				case PAGE_LOGO:
					if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
						ChangePage(PAGE_PRINT);
					else
						ChangePage(PAGE_CNC);
				break;

				case PAGE_CONTROL:
				break;

				case PAGE_CNCCONTROL:
          /*
					if(MACHINE_TYPE_CNC == ExecuterHead.MachineType)
					{
						if(READ() == HIGH)
							ChangeLaserCNCStatus(2);
						else
							ChangeLaserCNCStatus(4);
					}
					else if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
					{
						if(GetLaserPower() != 0)
							if(LASER_PORT->ODR & SPINDLE_LASER_PWM_PIN)
								ChangeLaserCNCStatus(2);
						else
							ChangeLaserCNCStatus(4);
					}
					*/
				break;

				case PAGE_CONFIRMPRINT:
					ShowConfirmFileName(tmpCardFileNameSelected);
				break;

			}
		}
		//查询页面
		else if(eventId == 3)
		{
			ID = (uint16_t)tmpBuff[3];
			uint8_t i;
			uint8_t tmpPage;
			tmpPage = CurrentHMIPage;
			CurrentHMIPage = 0xff;
			//获取界面ID
			for(i=0;i<sizeof(PageID);i++)
			{
				if(ID == PageID[i])
				{
					CurrentHMIPage = i;
					break;
				}
			}

			//屏幕热插拔
			if(CurrentHMIPage == PAGE_LOGO)
			{
				switch(tmpPage)
				{
					case PAGE_ZOFFSET:
					case PAGE_MOVE001:
					case PAGE_MOVE010:
					case PAGE_MOVE100:
						ChangePage(tmpPage);
					break;

					default:
						if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
							ChangePage(PAGE_PRINT);
						else
							ChangePage(PAGE_CNC);
					break;
				}
			}
		}
	}
	
	#if ENABLED(SDSUPPORT)	
		//文件列举中
		if(SeekingFile == 1)
		{
			//需要列举的文件个数大于0，表示还要列举
			if(SeekFileCount > 0)
			{
				//card.getfilename(FileIndex++, NULL, FileFilter);
				card.getfilename(FileIndex++, NULL);
				//防止溢出
				card.longFilename[LONG_FILENAME_LENGTH - 1] = 0;
				if(card.longFilename[0] != 0)
				{
					for(uint8_t k=0;k<strlen(card.longFilename);k++)
					{
						CardFileName[ShowFileIndex][k] = card.longFilename[k];
						CardFileName[ShowFileIndex][k + 1] = 0;
					}
				}
				else
				{
					for(uint8_t k=0;k<strlen(card.filename);k++)
					{
						CardFileName[ShowFileIndex][k] = card.filename[k];
						CardFileName[ShowFileIndex][k + 1] = 0;
					}
				}

				//防止溢出
				card.filename[FILENAME_LENGTH - 1] = 0;
				for(uint8_t k=0;k<strlen(card.filename);k++)
				{
					PrintFileName[ShowFileIndex][k] = card.filename[k];
					PrintFileName[ShowFileIndex][k+1] = 0;
				}
				//列举个数减1
				SeekFileCount--;
				//索引加1
				ShowFileIndex++;
			}
			//不需要列举文件
			else
			{
				//清除文件列举中标置
				SeekingFile = 0;
				//显示文件索引从0开始
				ShowFileIndex = 0;
			}
		}
		//文件不在列举，显示文件个数大于0，表示有文件需要显示
		else if(ShowFileCount > 0)
		{
			ShowFileName(ShowFileIndex, CardFileName[ShowFileIndex]);
			//显示个数减1
			ShowFileCount--;
			//显示索引加1
			ShowFileIndex++;
		}
	#endif
}

/**
 *HmiWriteData:Write datas to the HMI serial port
 *para pData:the pointer to the datas
 *para len:number of the datas to be written
 */
void HMILong::HmiWriteData(char *pData, uint16_t len)
{
  uint8_t c;
	while(len--)
	{
	  c = *pData++;
		HMISERIAL.write(c);
	} 
}

//清除串口数据
void HMILong::BuffFlush(void)
{

}

void HMILong::Show(void)
{
	uint8_t percent;
	static millis_t nextShowMill;
	
	//主界面才发送
	if(CurrentHMIPage == PAGE_PRINT)
	{
		if (nextShowMill < millis())
		{
			//显示喷头温度
			ShowTemperature(0, (int)thermalManager.temp_hotend[0].current, (int)thermalManager.temp_hotend[0].target);
			//显示床温度
			ShowTemperature(1, (int)thermalManager.temp_bed.current, (int)thermalManager.temp_bed.target);
			
			#if ENABLED(SDSUPPORT)	
				//文件已经打开
				if (card.isFileOpen())
				{
					//获取文件进度
					percent = card.percentDone();
					//card.LastPercent = percent;

					//显示进度条
					ShowProgressBar(percent);

					//显示打印时间	
					ShowPrintTime();
				}
			#endif
			nextShowMill = millis() + 1000;
		}
	}
	else if(CurrentHMIPage == PAGE_CNC)
	{
		if (nextShowMill < millis())
		{
			#if ENABLED(SDSUPPORT)	
				//文件已经打开
				if (card.isFileOpen())
				{
					//获取文件进度
					percent = card.percentDone();
					//card.LastPercent = percent;

					//显示进度条
					ShowProgressBar(percent);
					
					//显示打印时间	
					ShowPrintTime();
				}
			#endif
			nextShowMill = millis() + 1000;
		}
	}
	else if(CurrentHMIPage == PAGE_WAITHEAT)
	{
		if (nextShowMill < millis())
		{
			ShowChangeFilamentTamp();
			nextShowMill = millis() + 1500;
		}
	}
	
	
	//存储器已经插入
	if(SdInsert == true)
	{
		//本次检测被拔出来
		if(!(IS_UDISK_INSERTED))
		{
			//设置图标
			SetTFIcon(false);
			//标识卡被拔出
			SdInsert = false;
			//退出主界面
			if(CurrentHMIPage == PAGE_FILELIST)
			{
				nextShowMill = millis() + 50;
				FileCounts = 0;
				while (nextShowMill > millis());
				if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
					ChangePage(PAGE_PRINT);
				else
					ChangePage(PAGE_CNC);
			}
		}
	}
	//SD  卡未插入
	else
	{
		//本次检测已插入
		if(IS_UDISK_INSERTED)
		{
			//设置图标
			SetTFIcon(true);
			//标识卡已插入
			SdInsert = true;
		}
	}
}


//Add By Paladin
void HMILong::ChangePage(uint8_t Page)
{
	uint8_t i;
	uint8_t pagenum;

	pagenum = PageID[Page];
	
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x05;
	tmpBuff[i++] = 0x03;
	tmpBuff[i++] = pagenum;
	tmpBuff[i++] = 0xFF;
	tmpBuff[i++] = 0xFF;
	tmpBuff[i++] = 0xFF;
	tmpBuff[i++] = 0x100 - pagenum;
	HmiWriteData(tmpBuff, i);
}


void HMILong::ShowFileName(uint8_t Index, char *FileName)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	id[0] = 0x01;
	id[1] = (uint8_t)(FileLable[Index] >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(FileLable[Index]);
	i = 0;

	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	while(*FileName)
	{
		tmpBuff[i++] = *FileName++;
		if(i > 80)
			break;
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
}


void HMILong::ShowTemperature(uint8_t Index, int CurrentTemp, int TargetTemp)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	char tmpStr[15];
	id[0] = 0x00;
	id[1] = (uint8_t)(TemperatureLable[Index] >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(TemperatureLable[Index]);

	if(CurrentTemp <= 5)
		sprintf(tmpStr, PSTR("NA"));
	else
		sprintf(tmpStr, PSTR("%d/%d"), CurrentTemp, TargetTemp);

	i = 0;
    
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

void HMILong::ShowProgressBar(uint8_t Value)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
	{
		id[0] = 0;
		id[1] = (uint8_t)(PROGRESSBAR_PRINT >> 8);
		id[2] = 0x0C;
		id[3] = (uint8_t)(PROGRESSBAR_PRINT);
	}
	else
	{
		id[0] = 15;
		id[1] = (uint8_t)(PROGRESSBAR_CNC >> 8);
		id[2] = 0x0C;
		id[3] = (uint8_t)(PROGRESSBAR_CNC);
	}
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	tmpBuff[i++] = Value;

	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=3;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
}


void HMILong::SetControlEnable(bool Enable)
{
	uint8_t i=0;
	uint8_t j;
	uint8_t checksum;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;

	if(MACHINE_TYPE_3DPRINT != ExecuterHead.MachineType)
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x4D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x04;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x04;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x04;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x04;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
}

void HMILong::SetSettingEnable(bool Enable)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;

	i = 0;
	
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;
	if(MACHINE_TYPE_3DPRINT != ExecuterHead.MachineType)
	{
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4E;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4E;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
}

void HMILong::PauseEnable(bool Enable)
{
	uint8_t i=0;
	uint8_t j;
	uint8_t checksum;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;
	if(MACHINE_TYPE_3DPRINT != ExecuterHead.MachineType)
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x4F;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4F;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4F;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4F;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
}


void HMILong::StopEnable(bool Enable)
{
	uint8_t i=0;
	uint8_t j;
	uint8_t checksum;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;
	if(MACHINE_TYPE_3DPRINT != ExecuterHead.MachineType)
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x4B;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x02;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x02;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4B;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4B;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x02;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x02;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4B;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
}

void HMILong::SetTFIcon(bool Enable)
{
	uint8_t i = 0;
	uint8_t j;
	uint8_t checksum;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;
	if(MACHINE_TYPE_3DPRINT != ExecuterHead.MachineType)
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x3D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x3C;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x3C;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x3D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x43;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x03;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x03;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x43;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
}

void HMILong::ShowConfirmFileName(char *FileName)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	id[0] = 0x0A;
	id[1] = (uint8_t)(LABEL_CONFIRM_PRINT_FILENAME >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_CONFIRM_PRINT_FILENAME);
	i = 0;

	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	while(*FileName)
	{
		tmpBuff[i++] = *FileName++;
		if(i > 50)
			break;
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

void HMILong::ShowPrintFileName(char *FileName)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	if(MACHINE_TYPE_3DPRINT != ExecuterHead.MachineType)
	{
		id[0] = 15;
		id[1] = (uint8_t)(LABEL_CNC_FILENAME >> 8);
		id[2] = 0x05;
		id[3] = (uint8_t)(LABEL_CNC_FILENAME);
	}
	else
	{
		id[0] = 0x00;
		id[1] = (uint8_t)(LABEL_PRINT_FILENAME >> 8);
		id[2] = 0x05;
		id[3] = (uint8_t)(LABEL_PRINT_FILENAME);
	}
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];

	//获取文件名长度
	uint8_t Len;
	Len = strlen(FileName);
	//长度超过显示范围
	if(Len > 32)
	{
		//规划显示
		uint8_t extlen;
		uint8_t divlen;
		uint8_t dotcount;
		uint8_t sflen;
		uint8_t fl;
		char *p;
		p = &FileName[Len - 1];
		extlen = 0;
		while(*p != '.')
		{
			p--;
			extlen++;
		}
		//扩展名在10  个字节以内表示正常
		if((extlen < 10) && (extlen > 0))
		{
			extlen++;
			//计算除去后缀，可显示长度
			sflen = 31 - extlen;
			//计算文件名
			fl = Len - extlen;
			//计算省略号
			dotcount = fl - sflen;
			//省略号过多
			if(dotcount > 3)
			{
				if((sflen % 2) != 0)
					dotcount = 3;
				else
					dotcount = 4;
			}

			//计算前后各占多少字节 
			divlen = (sflen - dotcount) / 2;

			//前半部份文件名
			for(j=0;j<divlen;j++)
				tmpBuff[i++] = FileName[j];

			//  省略号
			for(j=0;j<dotcount;j++)
				tmpBuff[i++] = '.';
			
			//后半部份文件名
			p = &FileName[Len - extlen - divlen];
			for(j=0;j<divlen + extlen;j++)
			{
				tmpBuff[i++] = *p++;
			}
		}
		else
		{
			return;
		}
	}
	else
	{
		while(*FileName)
		{
			tmpBuff[i++] = *FileName++;
			if(i > 38)
				break;
		}
	}
	tmpBuff[i++] = 0;
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
    
	HmiWriteData(tmpBuff, i);
}



void HMILong::ZOffsetLable(float Value)
{
	int i, j;
	uint8_t checksum;
	uint8_t id[4];
	char tmpStr[10];

	i = (uint8_t)((Value) * 10.0f);
	j = (uint8_t)(Value);
	sprintf(tmpStr, PSTR("%d.%dmm"), j, i%10);
	id[0] = 0x07;
	id[1] = (uint8_t)(LABEL_ZOFFSET >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_ZOFFSET);
	i = 0;

	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	
	HmiWriteData(tmpBuff, i);
			
}


void HMILong::ShowChangeFilamentTamp()
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	char tmpStr[20];
	id[0] = 23;
	id[1] = (uint8_t)(LABEL_HEATER_TARGET_TEMP >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_HEATER_TARGET_TEMP);
	
	sprintf(tmpStr, "%d&",(int)thermalManager.temp_hotend[0].target);
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);

	id[0] = 23;
	id[1] = (uint8_t)(LABEL_HEATER_CUR_TEMP >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_HEATER_CUR_TEMP);
	
	sprintf(tmpStr, "%d&",(int)thermalManager.temp_hotend[0].current);
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

void HMILong::ShowLaserPower(uint8_t Power)
{
	char tmpStr[15];
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];

	id[0] = 15;
	id[1] = (uint8_t)(LABEL_CNC_POWER >> 8);
	id[2] = 4;
	id[3] = (uint8_t)(LABEL_CNC_POWER);
	
	if(MACHINE_TYPE_LASER == ExecuterHead.MachineType)
		sprintf(tmpStr, "%d", Power);
	else
		strcpy(tmpStr, "100");
	strcat(tmpStr, "%");

	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

void HMILong::ShowPrintTime(void)
{
	char tmpStr[15];
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
	{
		id[0] = 0;
		id[1] = (uint8_t)(LABEL_PRINT_TIME >> 8);
		id[2] = 4;
		id[3] = (uint8_t)(LABEL_PRINT_TIME);
	}
	else
	{
		id[0] = 15;
		id[1] = (uint8_t)(LABEL_CNC_TIME >> 8);
		id[2] = 4;
		id[3] = (uint8_t)(LABEL_CNC_TIME);
	}

	millis_t t = print_job_timer.duration();
    int hours = t / 60 / 60, minutes = (t / 60) % 60;

	sprintf(tmpStr, PSTR("%dh%dm"), hours, minutes);
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

void HMILong::ChangePrinterStatus(uint8_t Status)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4] = {0xff, 0xff, 0xff, 0xff};

	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x0B;
	tmpBuff[i++] = Status;
	tmpBuff[i++] = ~Status;
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
}


void HMILong::ChangeLaserCNCStatus(uint8_t       Status)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4] = {0xff, 0xff, 0xff, 0xff};

	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x0C;
	tmpBuff[i++] = Status;
	tmpBuff[i++] = ~Status;
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
}


void HMILong::SetAboutLabels(void)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	
	char Ver[30];

	strcpy(Ver, SHORT_BUILD_VERSION);

	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = 0x14;
	tmpBuff[i++] = (char)(LABEL_ABOUT_1 >> 8);
	tmpBuff[i++] = 0x04;
	tmpBuff[i++] = (char)LABEL_ABOUT_1;

	for(j=0;j<30;j++)
	{
		if(Ver[j] == 0)
			break;
		tmpBuff[i++] = Ver[j];
	}
	tmpBuff[i++] = 0;
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);

	strcpy(Ver, "Heated Bed: Up to 80&");

	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = 0x14;
	tmpBuff[i++] = (char)(LABEL_ABOUT_7 >> 8);
	tmpBuff[i++] = 0x04;
	tmpBuff[i++] = (char)LABEL_ABOUT_7;

	for(j=0;j<30;j++)
	{
		if(Ver[j] == 0)
			break;
		tmpBuff[i++] = Ver[j];
	}
	tmpBuff[i++] = 0;
	
	tmpBuff[2] = i - 3;
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);

}


#endif
