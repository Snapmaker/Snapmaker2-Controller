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
#include "stepper.h"

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
      #if (BOARD_VER == BOARD_SNAPMAKER2_v1)
  			//U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
      #endif
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
      #if (BOARD_VER == BOARD_SNAPMAKER2_v1)
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
      #endif
        InterruptAllCommand();
      break;

      case ManualPause:
      #if (BOARD_VER == BOARD_SNAPMAKER2_v1)
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
      #endif
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
      #if (BOARD_VER == BOARD_SNAPMAKER2_v1)
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
      #endif
        InterruptAllCommand();
      break;

      case ManualStop:
        #if (BOARD_VER == BOARD_SNAPMAKER2_v1)
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
        #endif
        InterruptAllCommand();
      break;

      case PowerPanicStop:
      #if (BOARD_VER == BOARD_SNAPMAKER2_v1)
        //U盘打印
        if(GetCurrentPrinterStatus() == STAT_RUNNING)
          card.pauseSDPrint();
      #endif
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
  block_t *current_blk;

  if(TriggleStat == TRIGGLE_STAT_PAUSE) {
    TriggleStat = TRIGGLE_STAT_IDLE;
    if((PauseType == ManualPause) || (PauseType == FilamentFaultPause) || (PauseType == DoorOpenPause) || (PauseType == ExecuterLostPause))  {
			//标置暂停状态
			if(CurrentStatus == STAT_RUNNING_ONLINE)
				CurrentStatus = STAT_PAUSE_ONLINE;
			else
				CurrentStatus = STAT_PAUSE;

      // disable all interrupt, then stepper will be stopped
      DISABLE_ISRS();

      stepper.quick_stop();
      set_current_from_steppers_for_axis(ALL_AXES);

      current_blk = stepper.get_current_block();
      if (current_blk) {
        PowerPanicData.Data.FilePosition = current_blk->filePos;
      }

      planner.clear_block_buffer();
      clear_command_queue();

      print_job_timer.pause();

      // save panic data to buffer
			PowerPanicData.saveWork();

      ENABLE_ISRS();

      //上报状态
  		if(PauseType == FilamentFaultPause)
				HMI.SendMachineFaultFlag();

      // move to stop point
      PowerPanicData.towardStopPoint();

      // switch to rellated pages in HMI
      switch(ExecuterHead.MachineType) {
        case MACHINE_TYPE_3DPRINT:
          //切换到主界面
          HMI.ChangePage(PAGE_PRINT);
          //关闭断料检测
          Periph.StopFilamentCheck();
          break;

        case MACHINE_TYPE_CNC:
        	HMI.ChangePage(PAGE_CNC);
          break;

        case MACHINE_TYPE_LASER:
          //切换到主界面
          HMI.ChangePage(PAGE_LASER);
          //启动检测
          Periph.StartDoorCheck();
          break;

        default:
          break;
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

      quickstop_stepper();

      // move to stop point
      PowerPanicData.towardStopPoint();

      // switch to rellated pages in HMI
      switch(ExecuterHead.MachineType) {
        case MACHINE_TYPE_3DPRINT:
          //切换到主界面
          HMI.ChangePage(PAGE_PRINT);
          //关闭断料检测
          Periph.StopFilamentCheck();
          break;

        case MACHINE_TYPE_CNC:
        	HMI.ChangePage(PAGE_CNC);
          break;

        case MACHINE_TYPE_LASER:
          //切换到主界面
          HMI.ChangePage(PAGE_LASER);
          //启动检测
          Periph.StartDoorCheck();
          break;

        default:
          break;
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


/*
 * follow functions are used to resume work
 */
#if ENABLED(VARIABLE_G0_FEEDRATE)
  extern float saved_g0_feedrate_mm_s;
  extern float saved_g1_feedrate_mm_s;
#endif
#define RESUME_PROCESS_CMD_SIZE 40

static void restore_xyz(void) {

  char cmd[RESUME_PROCESS_CMD_SIZE] = {0};

  // restore X, Y
  snprintf(cmd, RESUME_PROCESS_CMD_SIZE, "G0 X%.2f Y%.2f F4000",
      PowerPanicData.Data.PositionData[X_AXIS], PowerPanicData.Data.PositionData[Y_AXIS]);
  parser.parse(cmd);
  gcode.process_parsed_command();
  planner.synchronize();

  // restore Z
  snprintf(cmd, RESUME_PROCESS_CMD_SIZE, "G0 Z%.2f F4000", PowerPanicData.Data.PositionData[Z_AXIS]);
  parser.parse(cmd);
  gcode.process_parsed_command();
  planner.synchronize();
}

void inline StatusControl::resume_3dp(void) {
  enable_all_steppers();

  // pre-extrude
  relative_mode = true;

  parser.parse("G0 E15 F800");
  gcode.process_parsed_command();

  parser.parse("G0 E-4 F2400");
  gcode.process_parsed_command();

  planner.synchronize();

  relative_mode = false;

  // sync E position
  current_position[E_AXIS] = PowerPanicData.Data.PositionData[E_AXIS];
  sync_plan_position_e();

  restore_xyz();

  // switch printing page
  HMI.ChangePage(PAGE_PRINT);

  // enable filament runout
  Periph.StartFilamentCheck();
}


void inline StatusControl::resume_cnc(void) {
  // enable CNC motor

  restore_xyz();

  HMI.ChangePage(PAGE_CNC);
}

void inline StatusControl::resume_laser(void) {
  HMI.ChangePage(PAGE_LASER);

  // enable door check
  Periph.StartDoorCheck();
}

/**
 * Resume Pause
 */
void StatusControl::PauseResume()
{
  if (TriggleStat != TRIGGLE_STAT_RESUME)
    return;

  if (CurrentStatus != STAT_PAUSE_ONLINE || CurrentStatus != STAT_PAUSE)
    return;

  TriggleStat = TRIGGLE_STAT_IDLE;

  switch(ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    resume_3dp();
    break;

  case MACHINE_TYPE_CNC:
    resume_cnc();
    break;

  case MACHINE_TYPE_LASER:
    resume_laser();
    break;

  default:
    break;
  }

  // restore speed
  saved_g0_feedrate_mm_s = PowerPanicData.Data.TravelFeedRate;
  saved_g1_feedrate_mm_s = PowerPanicData.Data.PrintFeedRate;

  // clear command queue
  clear_command_queue();

  // resume stopwatch
  if (print_job_timer.isPaused()) print_job_timer.start();

  // status change
  if (CurrentStatus == STAT_PAUSE_ONLINE)
    CurrentStatus = STAT_RUNNING_ONLINE;
  else
    CurrentStatus = STAT_RUNNING;
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

