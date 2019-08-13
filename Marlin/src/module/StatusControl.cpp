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
#include "../feature/runout.h"
#include "../snap_module/lightbar.h"
#include "../snap_module/quickstop.h"
#include "../snap_module/snap_dbg.h"

StatusControl SystemStatus;


/**
 * Init
 */
void StatusControl::Init() {
  cur_status_ = SYSTAT_INIT;
  work_port_ = WORKING_PORT_NONE;
  fault_flag_ = 0;
}

/**
 *  CheckFatalError:Check if the machine have fatal error
 */
void StatusControl::CheckFatalError() {
  uint32_t Flag = FAULT_FLAG_BED | FAULT_FLAG_HEATER0 | FAULT_FLAG_LOAD;
  if((Flag & fault_flag_) != 0) {
    #if PIN_EXISTS(POWER2_SUPPLY)
      OUT_WRITE(POWER2_SUPPLY_PIN, HIGH);
    #endif
    #if PIN_EXISTS(POWER1_SUPPLY)
      OUT_WRITE(POWER1_SUPPLY_PIN, HIGH);
    #endif
    while(1) {
      HMI.CommandProcess();
    }
  }
}

/**
 * PauseTriggle:Triggle the pause
 * return: true if pause triggle success, or false
 */
ErrCode StatusControl::PauseTrigger(TriggerSource type)
{
  ErrCode ret = E_SUCCESS;
  SysStage stage = GetCurrentStage();

  if (stage != SYSTAGE_WORK && cur_status_ != SYSTAT_RESUME_WAITING) {
    LOG_W("cannot pause in current status: %d\n", cur_status_);
    return E_INVALID_STATE;
  }

  // if pause is triggered at first time
  // we need to do some operation which only can be performed at most once
  SetCurrentStatus(SYSTAT_PAUSE_TRIG);
  print_job_timer.pause();

  switch (type) {
  case TRIGGER_SOURCE_RUNOUT:
    quickstop.Trigger(QS_EVENT_RUNOUT);
    break;

  case TRIGGER_SOURCE_DOOR_OPEN:
    quickstop.Trigger(QS_EVENT_DOOR_OPEN);
    break;

  case TRIGGER_SOURCE_SC:
    if (work_port_ != WORKING_PORT_SC) {
      LOG_W("current working port is not SC!");
      return E_INVALID_STATE;
    }
    quickstop.Trigger(QS_EVENT_PAUSE);
    break;

  case TRIGGER_SOURCE_PC:
    if (work_port_ != WORKING_PORT_PC) {
      LOG_W("current working port is not PC!");
      return E_INVALID_STATE;
    }
    quickstop.Trigger(QS_EVENT_PAUSE);
    break;

  default:
    LOG_W("invlaid trigger source： %d\n", type);
    return E_PARAM;
    break;
  }

  // here the operations can be performed many times
  pause_source_ = type;

  if (MACHINE_TYPE_LASER == ExecuterHead.MachineType)
    ExecuterHead.Laser.SetLaserPower(0.0f);

  switch(type) {
    case TRIGGER_SOURCE_RUNOUT:
      SetSystemFaultBit(FAULT_FLAG_FILAMENT);
      parser.parse("M412 S0");
      gcode.process_parsed_command();
      HMI.SendMachineFaultFlag();
    break;

    case TRIGGER_SOURCE_DOOR_OPEN:
    break;

    case TRIGGER_SOURCE_SC:
    case TRIGGER_SOURCE_PC:
    break;

    default:
    return E_INVALID_STATE;
    break;
  }

  return ret;
}

/**
 * Triggle the stop
 * return: true if stop triggle success, or false
 */
ErrCode StatusControl::StopTrigger(TriggerSource type)
{
  SysStage stage = GetCurrentStage();

  if (stage != SYSTAGE_WORK && cur_status_ != SYSTAT_RESUME_WAITING) {
    LOG_W("cannot stop in current status[%d]\n", cur_status_);
    return E_INVALID_STATE;
  }

  if (stage == SYSTAGE_END) {
    LOG_I("we have stopped\n");
    return E_SUCCESS;
  }

  stop_type_ = type;

  SetCurrentStatus(SYSTAT_END_TRIG);

  print_job_timer.stop();

  switch(type) {
  case TRIGGER_SOURCE_SC:
    if (work_port_ != WORKING_PORT_SC) {
      LOG_W("current working port is not SC!");
      return E_INVALID_STATE;
    }
    quickstop.Trigger(QS_EVENT_PAUSE);
    break;

  case TRIGGER_SOURCE_PC:
    if (work_port_ != WORKING_PORT_PC) {
      LOG_W("current working port is not PC!");
      return E_INVALID_STATE;
    }
    quickstop.Trigger(QS_EVENT_PAUSE);
    break;

  case TRIGGER_SOURCE_FINISH:
    quickstop.Trigger(QS_EVENT_STOP);
    break;

  case TRIGGER_SOURCE_STOP_BUTTON:
    quickstop.Trigger(QS_EVENT_BUTTON);
    break;

  default:
    LOG_W("invalid trigger source: %d\n", type);
    return E_PARAM;
    break;
  }

  if (ExecuterHead.MachineType == MACHINE_TYPE_LASER)
    Periph.StopDoorCheck();

  // diable power panic data
  PowerPanicData.Data.Valid = 0;

  // disable filament checking
  if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT)
    process_cmd_imd("M412 S0");

  return E_SUCCESS;
}

/**
 * Pause processing
 */
void StatusControl::PauseProcess()
{
  if (GetCurrentStatus() != SYSTAT_PAUSE_STOPPED)
    return;

   // switch to rellated pages in HMI
  switch(ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    // disable filament runout
    parser.parse("M412 S0");
    gcode.process_parsed_command();
    break;

  case MACHINE_TYPE_CNC:
    break;

  case MACHINE_TYPE_LASER:
    // make sure door checking is enabled
    Periph.StartDoorCheck();
    break;

  default:
    break;
  }

  if (HMI.GetRequestStatus() == HMI_REQ_PAUSE) {
    HMI.SendMachineStatusChange((uint8_t)HMI.GetRequestStatus(), 0);
    // clear request flag of HMI
    HMI.ClearRequestStatus();
  }

  cur_status_ = SYSTAT_PAUSE_FINISH;
}

/**
 * Stop processing
 */
void StatusControl::StopProcess()
{
  if (cur_status_ != SYSTAT_END_FINISH)
    return;

  // tell Screen we finish stop
  if (HMI.GetRequestStatus() == HMI_REQ_STOP || HMI.GetRequestStatus() == HMI_REQ_FINISH) {
    HMI.SendMachineStatusChange((uint8_t)HMI.GetRequestStatus(), 0);
    work_port_ = WORKING_PORT_NONE;
    // clear flag
    HMI.ClearRequestStatus();
  }

  // clear stop type because stage will be changed
  stop_type_ = TRIGGER_SOURCE_NONE;

  cur_status_ = SYSTAT_IDLE;
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
  LOG_I("restore XYZ to (%f, %f, %f)\n", PowerPanicData.Data.PositionData[X_AXIS],
        PowerPanicData.Data.PositionData[Y_AXIS], PowerPanicData.Data.PositionData[Z_AXIS]);
  char cmd[RESUME_PROCESS_CMD_SIZE] = {0};

  // restore X, Y
  snprintf(cmd, RESUME_PROCESS_CMD_SIZE, "G0 X%.2f Y%.2f F4000",
      PowerPanicData.Data.PositionData[X_AXIS], PowerPanicData.Data.PositionData[Y_AXIS]);
  process_cmd_imd(cmd);
  planner.synchronize();

  // restore Z
  snprintf(cmd, RESUME_PROCESS_CMD_SIZE, "G0 Z%.2f F4000", PowerPanicData.Data.PositionData[Z_AXIS]);
  process_cmd_imd(cmd);
  planner.synchronize();
}

void inline StatusControl::resume_3dp(void) {
  enable_all_steppers();

  // pre-extrude
  relative_mode = true;

  process_cmd_imd("G0 E15 F800");

  process_cmd_imd("G0 E-4 F2400");

  planner.synchronize();

  relative_mode = false;

  // sync E position
  current_position[E_AXIS] = PowerPanicData.Data.PositionData[E_AXIS];
  sync_plan_position_e();

  restore_xyz();

  // switch printing page
  HMI.ChangePage(PAGE_PRINT);
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
 * Resume Process
 */
void StatusControl::ResumeProcess() {
  if (cur_status_ != SYSTAT_RESUME_TRIG)
    return;

  cur_status_ = SYSTAT_RESUME_MOVING;

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

  if (HMI.GetRequestStatus() == HMI_REQ_RESUME) {
    HMI.SendMachineStatusChange((uint8_t)HMI.GetRequestStatus(), 0);
    HMI.ClearRequestStatus();
  }

  pause_source_ = TRIGGER_SOURCE_NONE;
  cur_status_ = SYSTAT_RESUME_WAITING;

  // reset the state of quick stop handler
  quickstop.Reset();
}

/**
 * trigger resuming from other event
 */
ErrCode StatusControl::ResumeTrigger(TriggerSource s) {
  if (cur_status_ != SYSTAT_PAUSE_FINISH) {
    LOG_W("cannot trigger in current status: %d\n", cur_status_);
    return E_INVALID_STATE;
  }

  switch (s) {
  case TRIGGER_SOURCE_SC:
    if (work_port_ != WORKING_PORT_SC) {
      LOG_W("current working port is not SC!");
      return E_INVALID_STATE;
    }
    break;

  case TRIGGER_SOURCE_PC:
    if (work_port_ != WORKING_PORT_PC) {
      LOG_W("current working port is not PC!");
      return E_INVALID_STATE;
    }
    break;

  case TRIGGER_SOURCE_DOOR_CLOSE:
    break;

  default:
    LOG_W("invalid trigger source: %d\n", s);
    return E_PARAM;
    break;
  }

  if (Periph.IsDoorOpened() && MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
    return E_HARDWARE;

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  // need to check if we have filament ready
  if ((MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType) && runout.sensor_state()) {
    SetSystemFaultBit(FAULT_FLAG_FILAMENT);
    LOG_W("filament is runout, cannot resuem 3D print\n");
    return E_HARDWARE;
  }
#endif

  cur_status_ = SYSTAT_RESUME_TRIG;

  return E_SUCCESS;
}

/**
 * Get current working stage of system
 * return:Current Status,(STAT_IDLE,STAT_PAUSE,STAT_WORKING)
 */
SysStage StatusControl::GetCurrentStage() {
    switch (cur_status_) {
    case SYSTAT_INIT:
      return SYSTAGE_INIT;
      break;

    case SYSTAT_IDLE:
      return SYSTAGE_IDLE;
      break;

    case SYSTAT_WORK:
      return SYSTAGE_WORK;
      break;

    case SYSTAT_PAUSE_TRIG:
    case SYSTAT_PAUSE_STOPPED:
    case SYSTAT_PAUSE_FINISH:
      return SYSTAGE_PAUSE;
      break;

    case SYSTAT_RESUME_TRIG:
    case SYSTAT_RESUME_MOVING:
    case SYSTAT_RESUME_WAITING:
      return SYSTAGE_RESUMING;
      break;

    case SYSTAT_END_TRIG:
    case SYSTAT_END_FINISH:
      return SYSTAGE_END;
      break;

    default:
      return SYSTAGE_INVALID;
      break;
    }
}

/**
 * Map system status for screen
 * for screen:
 * 0 - idle
 * 1 - working with screen ports
 * 2 - pause with screen port
 * 3 - working with PC port
 * 4 - pause with port
 */
uint8_t StatusControl::MapCurrentStatusForSC() {
  SysStage stage = GetCurrentStage();
  uint8_t status;

  // idle for neither working and pause to screen
  if (stage != SYSTAGE_PAUSE && stage != SYSTAGE_WORK)
    return 0;

  if (stage == SYSTAGE_WORK)
    status = 1;
  else
    status = 2;

  if(work_port_ == WORKING_PORT_PC)
    status += 2;

  return status;
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
  return fault_flag_;
}

/**
 * StatusControl:Get Faults
 */
void StatusControl::ClearSystemFaultBit(uint32_t BitsToClear)
{
  fault_flag_ &= ~BitsToClear;
}

/**
 * StatusControl:Set Faults
 */
void StatusControl::SetSystemFaultBit(uint32_t BitsToSet)
{
  fault_flag_ |= BitsToSet;
}

/**
 * Resume work for all trigger
 */
void StatusControl::Process() {
  PauseProcess();
  StopProcess();
  ResumeProcess();
}
