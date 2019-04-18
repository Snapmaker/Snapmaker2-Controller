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

#include "StatusControl.h"

StatusControl SystemStatus;


/**
 * Init
 */
void StatusControl::Init()
{
 
}

/**
 * Triggle the pause
 * return: true if pause triggle success, or false
 */
bool StatusControl::PauseTriggle(PausePrintType type)
{
  if((CurrentStatus == STAT_IDLE) && (TriggleStat == TRIGGLE_STAT_IDLE)) {
    switch(type) {
      case FilamentFaultPause:
      break;

      case ManualPause:
      break;

      case DoorOpenPause:
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
  if((CurrentStatus == STAT_IDLE) && (TriggleStat == TRIGGLE_STAT_IDLE)) {
    switch(type) {
      case EndPrint:
      break;

      case ManualStop:
      break;

      case PowerPanicStop:
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
  char tmpBuff[64];
  if(TriggleStat == TRIGGLE_STAT_IDLE) {
    if((PauseType == ManualPause) || (PauseType == FilamentFaultPause) || (PauseType == DoorOpenPause) || (PauseType == ExecuterLostPause))  {
			//标置暂停状态
			if(CurrentStatus == STAT_RUNNING_ONLINE)
				CurrentStatus = STAT_PAUSE_ONLINE;
			else
				CurrentStatus = STAT_PAUSE;

			planner.clear_block_buffer();

			//清除指令
			//FlushCommand();

			//保存温度
			//for(int i=0;i<4;i++)		
			//	PowerPanicData.HeaterTamp[i] = target_temperature[i];

			//热床温度
			//PowerPanicData.BedTamp = target_temperature_bed;
			//风扇速度 
			//PowerPanicData.FanSpeed = TIM3->CCR1;
			//机型保存
			//PowerPanicData.MachineType = MachineSelected;

			//标置有效
			//PowerPanicData.Valid = 1;

			//文件名
			/*
			if(PowerPanicData.GCodeSource == GCODE_SOURCE_UDISK)
			{
				strcpy(PowerPanicData.FileName, card.currentselectfilename);
			}
			else
			{
				PowerPanicData.FileName[0] = 0;
			}
			
			
			current_position[X_AXIS] = StableCounter[X_AXIS] / axis_steps_per_unit[X_AXIS];
			current_position[Y_AXIS] = StableCounter[Y_AXIS] / axis_steps_per_unit[Y_AXIS];
			current_position[Z_AXIS] = StableCounter[Z_AXIS] / axis_steps_per_unit[Z_AXIS];
			current_position[E_AXIS] = StableCounter[E_AXIS] / axis_steps_per_unit[E_AXIS];
			*/

			//Z  轴补偿
			if(MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
			{
				//float delZ = mbl.get_z(current_position[X_AXIS], current_position[Y_AXIS]);
				//current_position[Z_AXIS] -= delZ;
			}
			
			sync_plan_position();

			//坐标
			/*
			for(int i=0;i<NUM_AXIS;i++)
				PowerPanicData.PositionData[i] = current_position[i];
		  */

			//清除请求停止标置，清除之后，电机才可重新运动
			//SystemRequestStop = false;

			//电流保持
			//StepperKeepCurrent = true;

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
					//Z  轴抬升30  
					if((current_position[Z_AXIS] + 30) < (home_offset[Z_AXIS] + Z_MAX_POS))
						sprintf(tmpBuff, "G0 Z%0.3f F2000", (current_position[Z_AXIS] + 30));
					else
						sprintf(tmpBuff, "G0 Z%0.3f F2000", (home_offset[Z_AXIS] + Z_MAX_POS));
					parser.parse(tmpBuff);
					gcode.process_parsed_command();
					while(planner.movesplanned())thermalManager.manage_heater();
					//X  轴走到限位开关位置
					sprintf(tmpBuff, "G0 X0 F2000");
					parser.parse(tmpBuff);
					gcode.process_parsed_command();
					while(planner.movesplanned())thermalManager.manage_heater();
					//Y  轴走到最大位置
					sprintf(tmpBuff, "G0 Y%0.3f F2000", (home_offset[Y_AXIS] + Y_MAX_POS));
					parser.parse(tmpBuff);
					gcode.process_parsed_command();
					while(planner.movesplanned())thermalManager.manage_heater();
				}
				//切换到主界面
				HMI.ChangePage(PAGE_PRINT);
				//关闭断料检测
				Periph.StopFilamentCheck();
			}
			else if(MACHINE_TYPE_CNC == ExecuterHead.MachineType)
			{			
				//Z  轴抬升30  
				if((current_position[Z_AXIS] + 30) < (home_offset[Z_AXIS] + Z_MAX_POS))
					sprintf(tmpBuff, "G0 Z%0.3f", (current_position[Z_AXIS] + 30));
				else
					sprintf(tmpBuff, "G0 Z%0.3f", (home_offset[Z_AXIS] + Z_MAX_POS));
				parser.parse(tmpBuff);
				gcode.process_parsed_command();
				while(planner.movesplanned())thermalManager.manage_heater();

				//走到工件原点
				strcpy(tmpBuff, "G0 X0 Y0 F3000");
				parser.parse(tmpBuff);
				gcode.process_parsed_command();
				while(planner.movesplanned())thermalManager.manage_heater();

				//关闭电机
				WRITE(LASER_PIN, false);
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

