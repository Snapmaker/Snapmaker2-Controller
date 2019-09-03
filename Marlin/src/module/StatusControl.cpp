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
  SysStage stage = GetCurrentStage();

  if (stage != SYSTAGE_WORK) {
    LOG_W("cannot pause in current status: %d\n", cur_status_);
    return E_INVALID_STATE;
  }

  // here the operations can be performed many times
  switch (type) {
  case TRIGGER_SOURCE_RUNOUT:
    SetSystemFaultBit(FAULT_FLAG_FILAMENT);
    HMI.SendMachineFaultFlag();
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

  cur_status_ = SYSTAT_PAUSE_TRIG;
  print_job_timer.pause();

  pause_source_ = type;

  if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
    powerpanic.Data.laser_pwm = ExecuterHead.Laser.GetTimPwm();
    powerpanic.Data.laser_percent = ExecuterHead.Laser.GetPowerPercent();
    ExecuterHead.Laser.RestorePower((float)0, 0);
  }

  lightbar.set_state(LB_STATE_STANDBY);

  return E_SUCCESS;
}

/**
 * Triggle the stop
 * return: true if stop triggle success, or false
 */
ErrCode StatusControl::StopTrigger(TriggerSource type)
{
  SysStage stage = GetCurrentStage();

  if (stage != SYSTAGE_WORK && cur_status_ != SYSTAT_RESUME_WAITING &&
      cur_status_ != SYSTAT_PAUSE_FINISH) {
    LOG_W("cannot stop in current status[%d]\n", cur_status_);
    return E_INVALID_STATE;
  }

  if ((type == TRIGGER_SOURCE_SC) && (work_port_ != WORKING_PORT_SC)) {
      LOG_W("current working port is not SC!");
      return E_INVALID_STATE;
  }

  if ((type == TRIGGER_SOURCE_PC) && (work_port_ != WORKING_PORT_PC)) {
    LOG_W("current working port is not PC!");
    return E_INVALID_STATE;
  }

  // if we already finish quick stop, just change system status
  // disable power-loss data, and exit with success
  if (cur_status_ == SYSTAT_PAUSE_FINISH) {
    // to make StopProcess work, cur_status_ need to be SYSTAT_END_FINISH
    cur_status_ = SYSTAT_END_FINISH;
    // diable bed and heater
    process_cmd_imd("M104 S0");
    process_cmd_imd("M140 S0");
    LOG_I("Stop in pauseing, trigger source: %d\n", type);
    return E_SUCCESS;
  }

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

  cur_status_ = SYSTAT_END_TRIG;

  stop_source_ = type;

  print_job_timer.stop();

  Periph.StopDoorCheck();

  if (ExecuterHead.MachineType == MACHINE_TYPE_LASER) {
    ExecuterHead.Laser.SetLaserPower((uint16_t)0);
  }

  // diable power panic data
  powerpanic.Data.Valid = 0;

  // disable filament checking
  if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
    process_cmd_imd("M412 S0");
  }

  lightbar.set_state(LB_STATE_STANDBY);


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
    process_cmd_imd("M412 S0");
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

  LOG_I("\nstop point:\n");
  LOG_I("X: %.2f, Y:%.2f, Z:%.2f, E: %.2f\n", powerpanic.Data.PositionData[0],
        powerpanic.Data.PositionData[1], powerpanic.Data.PositionData[2], powerpanic.Data.PositionData[3]);
  LOG_I("position shift:\n");
  LOG_I("X: %.2f, Y:%.2f, Z:%.2f\n", powerpanic.Data.position_shift[0],
        powerpanic.Data.position_shift[1], powerpanic.Data.position_shift[2]);

  LOG_I("Finish pause\n");
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
  stop_source_ = TRIGGER_SOURCE_NONE;
  pause_source_ = TRIGGER_SOURCE_NONE;
  cur_status_ = SYSTAT_IDLE;
  quickstop.Reset();

  process_cmd_imd("M140 S0");
  process_cmd_imd("M104 S0");

  LOG_I("Finish stop\n");
}


/*
 * follow functions are used to resume work
 */
#if ENABLED(VARIABLE_G0_FEEDRATE)
  extern float saved_g0_feedrate_mm_s;
  extern float saved_g1_feedrate_mm_s;
#endif
#define RESUME_PROCESS_CMD_SIZE 40

void inline StatusControl::RestoreXYZ(void) {
  LOG_I("\nrestore ponit:\n X:%.2f, Y:%.2f, Z:%.2f, E:%.2f)\n", powerpanic.Data.PositionData[X_AXIS],
        powerpanic.Data.PositionData[Y_AXIS], powerpanic.Data.PositionData[Z_AXIS],
        powerpanic.Data.PositionData[E_AXIS]);
  char cmd[RESUME_PROCESS_CMD_SIZE] = {0};

  // restore X, Y
  snprintf(cmd, RESUME_PROCESS_CMD_SIZE, "G0 X%.2f Y%.2f F4000",
      powerpanic.Data.PositionData[X_AXIS], powerpanic.Data.PositionData[Y_AXIS]);
  process_cmd_imd(cmd);
  planner.synchronize();

  // restore Z
  snprintf(cmd, RESUME_PROCESS_CMD_SIZE, "G0 Z%.2f F2400", powerpanic.Data.PositionData[Z_AXIS]);
  process_cmd_imd(cmd);
  planner.synchronize();
}

void inline StatusControl::resume_3dp(void) {
  enable_all_steppers();

  process_cmd_imd("M412 S1");

  process_cmd_imd("G92 E0");

  process_cmd_imd("G0 E15 F400");

  planner.synchronize();

  // restore E position
  current_position[E_AXIS] = powerpanic.Data.PositionData[E_AXIS];
  sync_plan_position_e();

  // retract filament to cut it out
  relative_mode = true;
  process_cmd_imd("G0 E-6.5 F3600");
  relative_mode = false;

  planner.synchronize();
}


void inline StatusControl::resume_cnc(void) {
  // enable CNC motor
  LOG_I("restore CNC power: %f\n", powerpanic.Data.cnc_power);
  ExecuterHead.CNC.SetPower(powerpanic.Data.cnc_power);
}

void inline StatusControl::resume_laser(void) {

}

/**
 * Resume Process
 */
void StatusControl::ResumeProcess() {
  if (cur_status_ != SYSTAT_RESUME_TRIG)
    return;

  cur_status_ = SYSTAT_RESUME_MOVING;

  set_bed_leveling_enabled(true);

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

  RestoreXYZ();

  // restore speed
  saved_g0_feedrate_mm_s = powerpanic.Data.TravelFeedRate;
  saved_g1_feedrate_mm_s = powerpanic.Data.PrintFeedRate;

  // clear command queue
  clear_command_queue();

  // resume stopwatch
  if (print_job_timer.isPaused()) print_job_timer.start();

  pause_source_ = TRIGGER_SOURCE_NONE;
  cur_status_ = SYSTAT_RESUME_WAITING;

  Periph.StartDoorCheck();

  // reset the state of quick stop handler
  quickstop.Reset();

  // tell screen we are ready  to work
  if (HMI.GetRequestStatus() == HMI_REQ_RESUME) {
    HMI.SendMachineStatusChange((uint8_t)HMI.GetRequestStatus(), 0);
    HMI.ClearRequestStatus();
  }
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

  if (Periph.IsDoorOpened())
    return E_HARDWARE;

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  // need to check if we have filament ready
  if ((MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType) &&
        runout.sensor_state()) {
    SetSystemFaultBit(FAULT_FLAG_FILAMENT);
    HMI.SendMachineFaultFlag();
    LOG_W("filament is runout, cannot resuem 3D print\n");
    return E_NO_RESRC;
  }
#endif

  cur_status_ = SYSTAT_RESUME_TRIG;

  return E_SUCCESS;
}

/**
 * when receive gcode in resume_waiting, we need to change state to work
 * and make some env ready
 */
void StatusControl::ResumeOver() {
  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    break;

  case MACHINE_TYPE_LASER:
    ExecuterHead.Laser.RestorePower(powerpanic.Data.laser_percent, powerpanic.Data.laser_pwm);
    break;

  case MACHINE_TYPE_CNC:
    break;

  default:
    LOG_E("invalid machine type: %d\n", ExecuterHead.MachineType);
    break;
  }

  LOG_I("Receive first cmd after resume\n");
  cur_status_ = SYSTAT_WORK;
  lightbar.set_state(LB_STATE_WORKING);
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
    status = 3;
  else
    status = 4;

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
 * screen start a work
 */
ErrCode StatusControl::StartWork(TriggerSource s) {

  if (cur_status_ != SYSTAT_IDLE) {
    LOG_W("cannot start work in current status: %d\n", cur_status_);
    return E_INVALID_STATE;
  }

  if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
    // z is un-homed
    if (axes_homed(Z_AXIS) == false) {
      // home
      process_cmd_imd("G28");
    }

    // move to original point
    do_blocking_move_to_logical_xy(0, 0);
  }

  powerpanic.Reset();

  if (s == TRIGGER_SOURCE_SC) {
    powerpanic.Data.GCodeSource = GCODE_SOURCE_SCREEN;
    work_port_ = WORKING_PORT_SC;
  }
  else if (s == TRIGGER_SOURCE_PC) {
    powerpanic.Data.GCodeSource = GCODE_SOURCE_PC;
    work_port_ = WORKING_PORT_PC;
  }

  if (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType &&
        runout.sensor_state()) {
    LOG_E("No filemant! Please insert filemant!\n");
    SetSystemFaultBit(FAULT_FLAG_FILAMENT);
    HMI.SendMachineFaultFlag();
  }

  // enable runout or not
  if (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
    process_cmd_imd("M412 S1");
  else
    process_cmd_imd("M412 S0");

  Periph.StartDoorCheck();

  print_job_timer.start();

  // set state
  cur_status_ = SYSTAT_WORK;

  lightbar.set_state(LB_STATE_WORKING);

  return E_SUCCESS;
}

/**
 * process Pause / Resume / Stop event
 */
void StatusControl::Process() {
  PauseProcess();
  StopProcess();
  ResumeProcess();
}
