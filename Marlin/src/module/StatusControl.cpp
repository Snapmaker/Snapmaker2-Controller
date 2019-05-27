#include "../inc/MarlinConfig.h"


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
#include "../sd/cardreader.h"
#include "StatusControl.h"
#include "PowerPanic.h"
#include "printcounter.h"

StatusControl SystemStatus;


/**
 * Init
 */
void StatusControl::Init()
{
 
}

/**
 * InterruptAllCommand:Clean all motion and actions
 */
void StatusControl::InterruptAllCommand()
{
  clear_command_queue();
  quickstop_stepper();
  print_job_timer.stop();
  if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
  {
    thermalManager.disable_all_heaters();
    thermalManager.zero_fan_speeds();
  }
  wait_for_heatup = false;
}

/**
 * PauseTriggle:Triggle the pause
 * return: true if pause triggle success, or false
 */
bool StatusControl::PauseTriggle(PausePrintType type)
{
  block_t *pBlock;
  if((CurrentStatus != STAT_IDLE) && (TriggleStat == TRIGGLE_STAT_IDLE)) {
    switch(type) {
      case FilamentFaultPause:
        CRITICAL_SECTION_START;
  			//U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
        SetSystemFaultBit(FAULT_FLAG_FILAMENT);
        InterruptAllCommand();
        if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
          Periph.SetFilamentCheck(false);
        //保存文件位置
        //PowerPanicData.Data.FilePosition = pBlock->FilePosition;
        //保存空跑速度
        //PowerPanicData.Data.TravelFeedRate = pBlock->TravelFeedRate;
        //保存打印速度
        //PowerPanicData.Data.PrintFeedRate = pBlock->PrintFeedRate;
        CRITICAL_SECTION_END;
      break;
      
      case DoorOpenPause:
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
        InterruptAllCommand();
      break;

      case ManualPause:
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
        InterruptAllCommand();
      break;
      
      default:
      break;
    }
    TriggleStat = TRIGGLE_STAT_PAUSE;
    PauseType = type;
    return true;
  }
  return false;
}

/**
 * Triggle the stop
 * return: true if stop triggle success, or false
 */
bool StatusControl::StopTriggle(StopPrintType type)
{
  if((CurrentStatus != STAT_IDLE) && (TriggleStat == TRIGGLE_STAT_IDLE)) {
    switch(type) {
      case EndPrint:
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
        InterruptAllCommand();
      break;

      case ManualStop:
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
        InterruptAllCommand();
      break;

      case PowerPanicStop:
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
        InterruptAllCommand();
      break;

      default:
        return false;
      break;
    }
    TriggleStat = TRIGGLE_STAT_STOP;
    StopType = type;
    return true;
  }
  return false;
}

/**
 * Pause processing
 */
void StatusControl::PauseProcess()
{
  if(TriggleStat == TRIGGLE_STAT_PAUSE) {
    TriggleStat = TRIGGLE_STAT_IDLE;
    if((PauseType == ManualPause) || (PauseType == FilamentFaultPause) || (PauseType == DoorOpenPause) || (PauseType == ExecuterLostPause))  {
			//标置暂停状态
			if(CurrentStatus == STAT_RUNNING_ONLINE)
				CurrentStatus = STAT_PAUSE_ONLINE;
			else
				CurrentStatus = STAT_PAUSE;

			//保存温度
			for(int i=0;i<HOTENDS;i++)		
				PowerPanicData.Data.HeaterTamp[i] = thermalManager.temp_hotend[i].current;

			//热床温度
			PowerPanicData.Data.BedTamp = thermalManager.temp_bed.current;
			//风扇速度 
			
			for(int i=0;i<FAN_COUNT;i++)
			  PowerPanicData.Data.FanSpeed[i] = Periph.GetFanSpeed(i);
			//机型保存
			PowerPanicData.Data.MachineType = ExecuterHead.MachineType;

			//标置有效
			PowerPanicData.Data.Valid = 1;

			//文件名
			if(PowerPanicData.Data.GCodeSource == GCODE_SOURCE_UDISK)
			{
				strcpy(PowerPanicData.Data.FileName, card.filename);
			}
			else
			{
				PowerPanicData.Data.FileName[0] = 0;
			}
			
			sync_plan_position();

			//坐标
			for(int i=0;i<NUM_AXIS;i++)
				PowerPanicData.Data.PositionData[i] = current_position[i];

			//上报状态
			if(PauseType == FilamentFaultPause)
				HMI.SendMachineFaultFlag();
      
			if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
			{
				//回抽
				if(thermalManager.temp_hotend[0].current > 180)
				{
					//切换到相对坐标模式
					relative_mode = true;
					current_position[E_AXIS] -= 4;
					line_to_current_position(40);
					planner.synchronize();
					while(planner.movesplanned()) thermalManager.manage_heater();
				}
				
				//切换到绝对坐标模式
				relative_mode = false;
        
				if(all_axes_known != false)
				{
					//Z轴抬升30  
					do_blocking_move_to_z(current_position[Z_AXIS] + 30, 10);
					//X  轴走到限位开关位置
          do_blocking_move_to_x(0, 35);
					//Y  轴走到最大位置
					do_blocking_move_to_xy(current_position[X_AXIS], home_offset[Y_AXIS] + Y_MAX_POS, 30);
				}
				//切换到主界面
				HMI.ChangePage(PAGE_PRINT);
				//关闭断料检测
				Periph.StopFilamentCheck();
			}
			else if(MACHINE_TYPE_CNC == ExecuterHead.MachineType)
			{
        do_blocking_move_to_z(current_position[Z_AXIS] + 30, 10);
				while(planner.movesplanned())thermalManager.manage_heater();

				//走到工件原点
				do_blocking_move_to_xy(0, 0, 50);

				//关闭电机
				ExecuterHead.CNC.SetCNCPower(0);
				//切换到主界面
				HMI.ChangePage(PAGE_CNC);
			}
			//激光
			else
			{
				//切换到主界面
				HMI.ChangePage(PAGE_LASER);
				//启动检测
				Periph.StartDoorCheck();
			}

			//回应上位机
			if((HMI.RequestStatus == STAT_PAUSE) || (HMI.RequestStatus == STAT_PAUSE_ONLINE))
			{
				HMI.SendMachineStatusChange(0x04, 0);
			}
			//清除标志
			HMI.RequestStatus = STAT_IDLE;
			
			//清除标置
			PauseType = NonePause;
		}
  }
}

/**
 * Stop processing
 */
void StatusControl::StopProcess()
{
  if(TriggleStat == TRIGGLE_STAT_STOP) {
    TriggleStat = TRIGGLE_STAT_IDLE;
    if(StopType == ManualStop) {
      if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType) {			        
				if(all_axes_known() != false) {
					//Z轴抬升30  
					do_blocking_move_to_z(current_position[Z_AXIS] + 30, 10);
					//X  轴走到限位开关位置
          do_blocking_move_to_x(0, 35);
					//Y  轴走到最大位置
					do_blocking_move_to_xy(current_position[X_AXIS], home_offset[Y_AXIS] + Y_MAX_POS, 30);
				}
				//切换到主界面
				HMI.ChangePage(PAGE_PRINT);
				//关闭断料检测
				Periph.StopFilamentCheck();
			}
			else if(MACHINE_TYPE_CNC == ExecuterHead.MachineType) {
        do_blocking_move_to_z(current_position[Z_AXIS] + 30, 10);
				while(planner.movesplanned())thermalManager.manage_heater();

				//走到工件原点
				do_blocking_move_to_xy(0, 0, 50);

				//关闭电机
				ExecuterHead.CNC.SetCNCPower(0);
				//切换到主界面
				HMI.ChangePage(PAGE_CNC);
			}
			//激光
			else {
				//切换到主界面
				HMI.ChangePage(PAGE_LASER);
				//启动检测
				Periph.StartDoorCheck();
			}

			//回应上位机
			if((HMI.RequestStatus == STAT_PAUSE) || (HMI.RequestStatus == STAT_PAUSE_ONLINE)) {
				HMI.SendMachineStatusChange(0x04, 0);
			}
			//清除标志
			HMI.RequestStatus = STAT_IDLE;
			
			//清除标置
			PauseType = NonePause;
      //状态切换
      CurrentStatus = STAT_IDLE;
    }
  }
}

/**
 * Resume Pause
 */
void StatusControl::PauseResume()
{
}

/**
 * Get current machine working status
 * return:Current Status,(STAT_IDLE,STAT_PAUSE,STAT_WORKING)
 */
uint8_t StatusControl::GetCurrentPrinterStatus()
{
  return CurrentStatus;
}

/**
 * Set current machine working status
 * para newstatus:STAT_IDLE,STAT_PAUSE,STAT_WORKING
 */
void StatusControl::SetCurrentPrinterStatus(uint8_t newstatus)
{
  CurrentStatus = newstatus;
}

/**
 * Get periph device status
 * return:periph device status
 */
uint8_t StatusControl::StatusControl::GetPeriphDeviceStatus()
{
  return PeriphDeviceStatus;
}


/**
 * StatusControl:Get Faults
 */
uint32_t StatusControl::GetSystemFault()
{
  return FaultFlag;
}

/**
 * StatusControl:Get Faults
 */
void StatusControl::ClearSystemFaultBit(uint32_t BitsToClear)
{
  FaultFlag &= ~BitsToClear;
}

/**
 * StatusControl:Get Faults
 */
void StatusControl::SetSystemFaultBit(uint32_t BitsToClear)
{
  FaultFlag |= BitsToClear;
}

