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

  isr_e_rindex_ = 0;
  isr_e_windex_ = 0;
  isr_e_len_ = 0;
}

/**
 * PauseTriggle:Triggle the pause
 * return: true if pause triggle success, or false
 */
ErrCode StatusControl::PauseTrigger(TriggerSource type)
{
  if (cur_status_ != SYSTAT_WORK && cur_status_!= SYSTAT_RESUME_WAITING) {
    LOG_W("cannot pause in current status: %d\n", cur_status_);
    return E_NO_SWITCHING_STA;
  }

  // here the operations can be performed many times
  switch (type) {
  case TRIGGER_SOURCE_RUNOUT:
    fault_flag_ |= FAULT_FLAG_FILAMENT;
    HMI.SendMachineFaultFlag(FAULT_FLAG_FILAMENT);
    break;

  case TRIGGER_SOURCE_DOOR_OPEN:
    break;

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

  default:
    LOG_W("invlaid trigger source： %d\n", type);
    return E_PARAM;
    break;
  }

  cur_status_ = SYSTAT_PAUSE_TRIG;

  quickstop.Trigger(QS_EVENT_PAUSE);

  print_job_timer.pause();

  pause_source_ = type;

  lightbar.set_state(LB_STATE_STANDBY);

  // reset the status of filament monitor
  runout.reset();

  return E_SUCCESS;
}

/**
 * Triggle the stop
 * return: true if stop triggle success, or false
 */
ErrCode StatusControl::StopTrigger(TriggerSource type) {

  if (cur_status_ != SYSTAT_WORK && cur_status_ != SYSTAT_RESUME_WAITING &&
      cur_status_ != SYSTAT_PAUSE_FINISH) {
    LOG_E("cannot stop in current status[%d]\n", cur_status_);
    return E_NO_SWITCHING_STA;
  }

  switch(type) {
  case TRIGGER_SOURCE_SC:
    if (work_port_ != WORKING_PORT_SC) {
      LOG_E("current working port is not SC!");
      return E_FAILURE;
    }
    break;

  case TRIGGER_SOURCE_PC:
    if (work_port_ != WORKING_PORT_PC) {
      LOG_E("current working port is not PC!");
      return E_FAILURE;
    }
    break;

  case TRIGGER_SOURCE_FINISH:
    // if screen tell us Gcode is ended, wait for all movement output
    // because planner.synchronize() will call HMI process nestedly,
    // and maybe some function will check the status, so we change the status
    // firstly
    cur_status_ = SYSTAT_END_TRIG;
    planner.synchronize();
    break;

  default:
    break;
  }

  // recover scaling
  feedrate_scaling = 100;

  // set temp to 0
  if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
    thermalManager.setTargetBed(0);
    HOTEND_LOOP() { thermalManager.setTargetHotend(0, e); }

    // if user abort the work, FAN0 will not be closed
    // set target temp to 0 only make FAN1 be closed
    ExecuterHead.SetFan(0, 0);
  }

  print_job_timer.stop();

  // diable power panic data
  powerpanic.Data.Valid = 0;

  // if we already finish quick stop, just change system status
  // disable power-loss data, and exit with success
  if (cur_status_ == SYSTAT_PAUSE_FINISH) {
    // to make StopProcess work, cur_status_ need to be SYSTAT_END_FINISH
    cur_status_ = SYSTAT_END_FINISH;
    pause_source_ = TRIGGER_SOURCE_NONE;
    LOG_I("Stop in pauseing, trigger source: %d\n", type);
    return E_SUCCESS;
  }

  cur_status_ = SYSTAT_END_TRIG;

  // to workaround issue QS bug maybe make bed heating always
  // enable power after finish QS
  disable_power_domain(POWER_DOMAIN_BED);

  stop_source_ = type;

  if (ExecuterHead.MachineType == MACHINE_TYPE_LASER) {
    ExecuterHead.Laser.Off();
  }

  lightbar.set_state(LB_STATE_STANDBY);

  quickstop.Trigger(QS_EVENT_STOP);

  return E_SUCCESS;
}

/**
 * Pause processing
 */
void StatusControl::PauseProcess()
{
  if (GetCurrentStatus() != SYSTAT_PAUSE_STOPPED)
    return;

  if (HMI.GetRequestStatus() == HMI_REQ_PAUSE) {
    HMI.SendMachineStatusChange((uint8_t)HMI.GetRequestStatus(), 0);
    // clear request flag of HMI
    HMI.ClearRequestStatus();
  }

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

  enable_power_domain(POWER_DOMAIN_BED);

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

  // the positions we recorded are logical positions, so cannot use native movement API
  // restore X, Y
  do_blocking_move_to_logical_xy(powerpanic.Data.PositionData[X_AXIS], powerpanic.Data.PositionData[Y_AXIS], 60);
  planner.synchronize();

  // restore Z
  if (MACHINE_TYPE_CNC == ExecuterHead.MachineType) {
    do_blocking_move_to_logical_z(powerpanic.Data.PositionData[Z_AXIS] + 15, 30);
    do_blocking_move_to_logical_z(powerpanic.Data.PositionData[Z_AXIS], 10);
  }
  else {
    do_blocking_move_to_logical_z(powerpanic.Data.PositionData[Z_AXIS], 30);
  }
  planner.synchronize();
}

void inline StatusControl::resume_3dp(void) {
  enable_all_steppers();

  process_cmd_imd("G92 E0");

  relative_mode = true;
  process_cmd_imd("G0 E30 F400");
  // retract filament to try to cut it out
  process_cmd_imd("G0 E-6 F3600");
  planner.synchronize();

  process_cmd_imd("G0 E6 F400");
  planner.synchronize();
  relative_mode = false;

  // restore E position
  current_position[E_AXIS] = powerpanic.Data.PositionData[E_AXIS];
  sync_plan_position_e();
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


  switch(ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    set_bed_leveling_enabled(true);
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
  if (action_ban & ACTION_BAN_NO_WORKING) {
    LOG_E("System Fault! Now cannot start working!\n");
    return E_NO_WORKING;
  }

  if (cur_status_ != SYSTAT_PAUSE_FINISH) {
    LOG_W("cannot trigger in current status: %d\n", cur_status_);
    return E_NO_SWITCHING_STA;
  }

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    if (runout.is_filament_runout()) {
      LOG_E("No filemant!\n");
      fault_flag_ |= FAULT_FLAG_FILAMENT;
      return E_NO_FILAMENT;
    }
    break;

  case MACHINE_TYPE_CNC:
  case MACHINE_TYPE_LASER:
    if (Periph.IsDoorOpened()) {
      LOG_E("Door is opened!\n");
      fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
      return E_DOOR_OPENED;
    }
    break;

  default:
    break;
  }

  switch (s) {
  case TRIGGER_SOURCE_SC:
    if (work_port_ != WORKING_PORT_SC) {
      LOG_W("current working port is not SC!");
      return E_FAILURE;
    }
    break;

  case TRIGGER_SOURCE_PC:
    if (work_port_ != WORKING_PORT_PC) {
      LOG_W("current working port is not PC!");
      return E_FAILURE;
    }
    break;

  case TRIGGER_SOURCE_DOOR_CLOSE:
    break;

  default:
    LOG_W("invalid trigger source: %d\n", s);
    return E_FAILURE;
    break;
  }

  cur_status_ = SYSTAT_RESUME_TRIG;

  return E_SUCCESS;
}

/**
 * when receive gcode in resume_waiting, we need to change state to work
 * and make some env ready
 */
ErrCode StatusControl::ResumeOver() {
  // if exception happened duration resuming work
  // give a opportunity to stop working
  if (action_ban & ACTION_BAN_NO_WORKING) {
    LOG_E("System Fault! Now cannot start working!\n");
    StopTrigger(TRIGGER_SOURCE_EXCEPTION);
    return E_NO_WORKING;
  }

  if (cur_status_ != SYSTAT_RESUME_WAITING)
    return E_NO_SWITCHING_STA;

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    if (runout.is_filament_runout()) {
      LOG_E("No filemant! Please insert filemant!\n");
      fault_flag_ |= FAULT_FLAG_FILAMENT;
      PauseTrigger(TRIGGER_SOURCE_RUNOUT);
      return E_NO_FILAMENT;
    }
    break;

  case MACHINE_TYPE_CNC:
  case MACHINE_TYPE_LASER:
    if (Periph.IsDoorOpened()) {
      LOG_E("Door is opened, please close the door!\n");
      fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
      PauseTrigger(TRIGGER_SOURCE_DOOR_OPEN);
      return E_DOOR_OPENED;
    }

    if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
      if (powerpanic.Data.laser_pwm > 0)
        ExecuterHead.Laser.On();
    }
    break;

  default:
    LOG_E("invalid machine type: %d\n", ExecuterHead.MachineType);
    break;
  }

  LOG_I("Receive first cmd after resume\n");
  cur_status_ = SYSTAT_WORK;
  lightbar.set_state(LB_STATE_WORKING);

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

  if (action_ban & ACTION_BAN_NO_WORKING) {
    LOG_E("System Fault! Now cannot start working!\n");
    return E_NO_WORKING;
  }

  if (cur_status_ != SYSTAT_IDLE) {
    LOG_W("cannot start work in current status: %d\n", cur_status_);
    return E_NO_SWITCHING_STA;
  }

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    if (runout.is_filament_runout()) {
      fault_flag_ |= FAULT_FLAG_FILAMENT;
      LOG_E("No filemant!\n");
      return E_NO_FILAMENT;
    }
    break;

  case MACHINE_TYPE_LASER:
  case MACHINE_TYPE_CNC:
    if (Periph.IsDoorOpened()) {
      fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
      LOG_E("Door is opened!\n");
      return E_DOOR_OPENED;
    }
    if (axes_homed(Z_AXIS) == false)
      process_cmd_imd("G28 Z");

    // set to defualt power, but not turn on Motor
    if (MACHINE_TYPE_CNC == ExecuterHead.MachineType) {
      ExecuterHead.CNC.ChangePower(100);
    }
    break;

  default:
    break;
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

/**
 * Check if exceptions happened, this should be called in idle()[Marlin.cpp]
 * check follow exceptions:
 *    1. if sensor of bed / hotend is damagd
 */
void StatusControl::CheckException() {
  ExceptionHost h;
  ExceptionType t;
  uint8_t got_exception = 0;

  if (isr_e_len_ > 0) {
    DISABLE_TEMPERATURE_INTERRUPT();
    h = (ExceptionHost)isr_exception[isr_e_rindex_][0];
    t = (ExceptionType)isr_exception[isr_e_rindex_][1];
    if (++isr_e_rindex_ >= EXCEPTION_ISR_BUFFSER_SIZE)
      isr_e_rindex_ = 0;
    isr_e_len_--;
    ENABLE_TEMPERATURE_INTERRUPT();

    got_exception = 1;
  }

  if (got_exception)
    ThrowException(h, t);
}

/**
 * Use to throw a excetion, system will handle it, and return to Screen
 */
ErrCode StatusControl::ThrowException(ExceptionHost h, ExceptionType t) {
  uint8_t action = EACTION_NONE;
  uint8_t action_ban = ACTION_BAN_NONE;
  uint8_t power_ban = POWER_DOMAIN_NONE;
  uint8_t power_disable = POWER_DOMAIN_NONE;
  uint32_t new_fault_flag = 0;

  switch (t) {
  case ETYPE_NO_HOST:
    switch (h) {
    case EHOST_EXECUTOR:
      if (fault_flag_ & FAULT_FLAG_NO_EXECUTOR)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_NO_EXECUTOR;
      action_ban |= (ACTION_BAN_NO_WORKING | ACTION_BAN_NO_HEATING_HOTEND);
      LOG_E("Cannot detect Executor!\n");
      break;

    case EHOST_LINEAR:
      if (fault_flag_ & FAULT_FLAG_NO_LINEAR)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_NO_LINEAR;
      action_ban |= (ACTION_BAN_NO_WORKING | ACTION_BAN_NO_MOVING);
      Running = false;
      LOG_E("Cannot detect any Linear Module!\n");
      break;

    case EHOST_MC:
      if (fault_flag_ & FAULT_FLAG_UNKNOW_MODEL)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_UNKNOW_MODEL;
      action_ban |= ACTION_BAN_NO_WORKING;
      LOG_E("Cannot detect Machine model!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_LOST_HOST:
    switch (h) {
    case EHOST_EXECUTOR:
      if (fault_flag_ & FAULT_FLAG_LOST_EXECUTOR)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_LOST_EXECUTOR;
      action_ban |= (ACTION_BAN_NO_WORKING | ACTION_BAN_NO_HEATING_HOTEND);
      action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_HOTEND;
      LOG_E("Executor Lost!\n");
      break;

    case EHOST_LINEAR:
      if (fault_flag_ & FAULT_FLAG_LOST_LINEAR)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_LOST_LINEAR;
      action_ban |= ACTION_BAN_NO_WORKING;
      action = EACTION_STOP_WORKING;
      LOG_E("Linear Module Lost!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_RUNOUT:
    if (fault_flag_ & FAULT_FLAG_FILAMENT)
      return E_INVALID_STATE;
    new_fault_flag = FAULT_FLAG_FILAMENT;
    LOG_I("runout fault is cleared!\n");
    break;

  case ETYPE_LOST_CFG:
    if (fault_flag_ & FAULT_FLAG_LOST_SETTING)
      return E_SAME_STATE;
    new_fault_flag = FAULT_FLAG_LOST_SETTING;
    LOG_E("Configuration Lost!\n");
    break;

  case ETYPE_PORT_BAD:
    if (fault_flag_ & FAULT_FLAG_BED_PORT)
      return E_SAME_STATE;
    new_fault_flag = FAULT_FLAG_BED_PORT;
    if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT)
      action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_BED;
    action_ban |= ACTION_BAN_NO_HEATING_BED;
    power_ban = POWER_DOMAIN_BED;
    power_disable = POWER_DOMAIN_BED;
    LOG_E("Port of heating bed is damaged!\n");
    break;

  case ETYPE_POWER_LOSS:
    if (fault_flag_ & FAULT_FLAG_POWER_LOSS)
      return E_SAME_STATE;
    new_fault_flag = FAULT_FLAG_POWER_LOSS;
    LOG_E("power-loss apeared at last poweroff!\n");
    break;

  case ETYPE_HEAT_FAIL:
    if (ExecuterHead.MachineType != MACHINE_TYPE_3DPRINT)
      return E_SUCCESS;
    switch (h) {
    case EHOST_HOTEND0:
      if (fault_flag_ & FAULT_FLAG_HOTEND_HEATFAIL)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_HOTEND_HEATFAIL;
      action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_HOTEND;
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_HEATFAIL)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_HEATFAIL;
      action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_BED;
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_TEMP_RUNAWAY:
    if (ExecuterHead.MachineType != MACHINE_TYPE_3DPRINT)
      return E_SUCCESS;
    switch (h) {
    case EHOST_HOTEND0:
      if (fault_flag_ & FAULT_FLAG_HOTEND_RUNWAWY)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_HOTEND_RUNWAWY;
      action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_BED | EACTION_STOP_HEATING_HOTEND;
      break;

    case EHOST_BED:
      LOG_W("Not handle exception: BED TEMP RUNAWAY\n");
      if (fault_flag_ & FAULT_FLAG_BED_RUNAWAY)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_RUNAWAY;
      // action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_BED;
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_TEMP_REDUNDANCY:
    if (ExecuterHead.MachineType != MACHINE_TYPE_3DPRINT)
      return E_SUCCESS;
    LOG_E("Not handle exception: TEMP_REDUNDANCY\n");
    return E_FAILURE;
    break;

  case ETYPE_SENSOR_BAD:
    switch (h) {
    case EHOST_HOTEND0:
      if (ExecuterHead.MachineType != MACHINE_TYPE_3DPRINT)
        return E_SUCCESS;
      if (fault_flag_ & FAULT_FLAG_HOTEND_SENSOR_BAD)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_HOTEND_SENSOR_BAD;
      action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_BED | EACTION_STOP_HEATING_HOTEND;
      action_ban |= (ACTION_BAN_NO_WORKING | ACTION_BAN_NO_HEATING_HOTEND);
      LOG_E("Error happened in Thermistor of Hotend!\n");
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_SENSOR_BAD)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_SENSOR_BAD;
      action = EACTION_STOP_HEATING_BED;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
        LOG_E("Error happened in Thermistor of Heated Bed!\n");
      }
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_BELOW_MINTEMP:
    if (ExecuterHead.MachineType != MACHINE_TYPE_3DPRINT)
      return E_SUCCESS;
    LOG_E("Not handle exception: BELOW_MINTEMP\n");
    return E_FAILURE;
    break;

  case ETYPE_OVERRUN_MAXTEMP:
    switch (h) {
    case EHOST_HOTEND0:
      if (fault_flag_ & FAULT_FLAG_HOTEND_MAXTEMP)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_HOTEND_MAXTEMP;
      action = EACTION_STOP_HEATING_HOTEND;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
      }
      LOG_E("current temp of hotend is higher than MAXTEMP: %d!\n", HEATER_0_MAXTEMP);
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_MAXTEMP)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_MAXTEMP;
      action = EACTION_STOP_HEATING_BED;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
      }
      LOG_E("current temp of Bed is higher than MAXTEMP: %d!\n", BED_MAXTEMP);
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_OVERRUN_MAXTEMP_AGAIN:
    switch (h) {
    case EHOST_HOTEND0:
      if (fault_flag_ & FAULT_FLAG_HOTEND_SHORTCIRCUIT)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_HOTEND_SHORTCIRCUIT;
      action = EACTION_STOP_HEATING_HOTEND;
      action_ban = ACTION_BAN_NO_HEATING_HOTEND;
      power_disable = POWER_DOMAIN_HOTEND;
      power_ban = POWER_DOMAIN_HOTEND;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
        action_ban |= ACTION_BAN_NO_WORKING | ACTION_BAN_NO_MOVING;
      }
      LOG_E("current temp of hotend is more higher than MAXTEMP: %d!\n", HEATER_0_MAXTEMP + 10);
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_SHORTCIRCUIT)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_SHORTCIRCUIT;
      action = EACTION_STOP_HEATING_BED;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      power_disable = POWER_DOMAIN_BED;
      power_ban = POWER_DOMAIN_BED;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
      }
      LOG_E("current temp of Bed is more higher than MAXTEMP: %d!\n", BED_MAXTEMP + 5);
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_ABRUPT_TEMP_DROP:
    switch (h) {
    case EHOST_HOTEND0:
      if (fault_flag_ & FAULT_FLAG_HOTEND_SENSOR_COMEOFF)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_HOTEND_SENSOR_COMEOFF;
      action = EACTION_STOP_HEATING_HOTEND;
      action_ban = ACTION_BAN_NO_HEATING_HOTEND;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
        action_ban |= ACTION_BAN_NO_WORKING;
      }
      LOG_E("current temperature of hotend dropped abruptly!\n");
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_SENSOR_COMEOFF)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_SENSOR_COMEOFF;
      action = EACTION_STOP_HEATING_BED;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
      }
      LOG_E("current temperature of bed dropped abruptly!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_SENSOR_COME_OFF:
    switch (h) {
    case EHOST_HOTEND0:
      if (fault_flag_ & FAULT_FLAG_HOTEND_SENSOR_COMEOFF)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_HOTEND_SENSOR_COMEOFF;
      action = EACTION_STOP_HEATING_HOTEND;
      action_ban = ACTION_BAN_NO_HEATING_HOTEND;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
        action_ban |= ACTION_BAN_NO_WORKING;
      }
      LOG_E("Thermistor of hotend maybe is comed off!\n");
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_SENSOR_COMEOFF)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_SENSOR_COMEOFF;
      action = EACTION_STOP_HEATING_BED;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
      }
      LOG_E("Thermistor of bed maybe is comed off!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  default:
    LOG_E("unknown Exception [%d] happened with Host [%d]\n", t, h);
    return E_FAILURE;
    break;
  }

  if (power_disable)
    disable_power_domain(power_disable);

  if (power_ban)
    enable_power_ban(power_ban);

  if (action_ban)
    enable_action_ban(action_ban);

  if (action & EACTION_STOP_HEATING_HOTEND) {
    HOTEND_LOOP() thermalManager.setTargetHotend(0, e);
  }

  if (action & EACTION_STOP_HEATING_BED)
    thermalManager.setTargetBed(0);

  if (action & EACTION_STOP_WORKING) {
    StopTrigger(TRIGGER_SOURCE_EXCEPTION);
  }
  else if (action & EACTION_PAUSE_WORKING) {
    PauseTrigger(TRIGGER_SOURCE_EXCEPTION);
  }

  fault_flag_ |= new_fault_flag;
  HMI.SendMachineFaultFlag(new_fault_flag);

  return E_SUCCESS;
}


ErrCode StatusControl::ThrowExceptionISR(ExceptionHost h, ExceptionType t) {
  if (isr_e_len_ >= EXCEPTION_ISR_BUFFSER_SIZE)
    return E_NO_RESRC;

  isr_exception[isr_e_windex_][0] = h;
  isr_exception[isr_e_windex_][1] = t;

  if (++isr_e_windex_ >= EXCEPTION_ISR_BUFFSER_SIZE)
    isr_e_windex_ = 0;

  isr_e_len_++;

  return E_SUCCESS;
}


ErrCode StatusControl::ClearException(ExceptionHost h, ExceptionType t) {
  uint8_t action_ban = ACTION_BAN_NONE;
  uint8_t power_ban = POWER_DOMAIN_NONE;

  switch (t) {
  case ETYPE_NO_HOST:
    switch (h) {
    case EHOST_EXECUTOR:
      if (!(fault_flag_ & FAULT_FLAG_NO_EXECUTOR))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_NO_EXECUTOR;
      action_ban = (ACTION_BAN_NO_WORKING | ACTION_BAN_NO_HEATING_HOTEND);
      LOG_I("detect Executor!\n");
      break;

    case EHOST_LINEAR:
      if (!(fault_flag_ & FAULT_FLAG_NO_LINEAR))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_NO_LINEAR;
      action_ban = (ACTION_BAN_NO_WORKING | ACTION_BAN_NO_MOVING);
      Running = true;
      LOG_I("detect Linear Module!\n");
      break;

    case EHOST_MC:
      if (!(fault_flag_ & FAULT_FLAG_UNKNOW_MODEL))
        return E_SAME_STATE;
      fault_flag_ &= FAULT_FLAG_UNKNOW_MODEL;
      action_ban = ACTION_BAN_NO_WORKING;
      LOG_I("Detected Machine model!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_LOST_HOST:
    switch (h) {
    case EHOST_EXECUTOR:
      if (!(fault_flag_ & FAULT_FLAG_LOST_EXECUTOR))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_LOST_EXECUTOR;
      action_ban = (ACTION_BAN_NO_WORKING | ACTION_BAN_NO_HEATING_HOTEND);
      LOG_I("Executor Lost!\n");
      break;

    case EHOST_LINEAR:
      if (!(fault_flag_ & FAULT_FLAG_LOST_LINEAR))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_LOST_LINEAR;
      action_ban = ACTION_BAN_NO_WORKING;
      LOG_I("Linear Module Lost!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_RUNOUT:
    if (!(fault_flag_ & FAULT_FLAG_FILAMENT))
      return E_INVALID_STATE;
    fault_flag_ &= ~FAULT_FLAG_FILAMENT;
    LOG_I("runout fault is cleared!\n");
    break;

  case ETYPE_LOST_CFG:
    LOG_I("clear error of configiration lost!\n");
    if (!(fault_flag_ & FAULT_FLAG_LOST_SETTING))
      return E_INVALID_STATE;
    fault_flag_ &= ~FAULT_FLAG_LOST_SETTING;
    break;

  case ETYPE_PORT_BAD:
    if (!(fault_flag_ & FAULT_FLAG_BED_PORT))
      return E_INVALID_STATE;
    fault_flag_ &= ~FAULT_FLAG_BED_PORT;
    action_ban = ACTION_BAN_NO_HEATING_BED;
    power_ban = POWER_DOMAIN_BED;
    LOG_I("Port of heating bed is recovered!\n");
    break;

  case ETYPE_POWER_LOSS:
    if (!(fault_flag_ & FAULT_FLAG_POWER_LOSS))
      return E_INVALID_STATE;
    fault_flag_ &= ~FAULT_FLAG_POWER_LOSS;
    LOG_I("power-loss fault is cleared!\n");
    break;

  case ETYPE_HEAT_FAIL:
    switch (h) {
    case EHOST_HOTEND0:
      if (!(fault_flag_ & FAULT_FLAG_HOTEND_HEATFAIL))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_HOTEND_HEATFAIL;
      break;

    case EHOST_BED:
      if (!(fault_flag_ & FAULT_FLAG_BED_HEATFAIL))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_BED_HEATFAIL;
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_TEMP_RUNAWAY:
    switch (h) {
    case EHOST_HOTEND0:
      if (!(fault_flag_ & FAULT_FLAG_HOTEND_RUNWAWY))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_HOTEND_RUNWAWY;
      break;

    case EHOST_BED:
      if (!(fault_flag_ & FAULT_FLAG_BED_RUNAWAY))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_BED_RUNAWAY;
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_TEMP_REDUNDANCY:
    LOG_I("TEMP_REDUNDANCY fault is cleared\n");
    break;

  case ETYPE_SENSOR_BAD:
    switch (h) {
    case EHOST_HOTEND0:
      if (!(fault_flag_ & FAULT_FLAG_HOTEND_SENSOR_BAD))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_HOTEND_SENSOR_BAD;
      action_ban = (ACTION_BAN_NO_WORKING | ACTION_BAN_NO_HEATING_HOTEND);
      LOG_I("Thermistor of Hotend has recovered!\n");
      break;

    case EHOST_BED:
      if (!(fault_flag_ & FAULT_FLAG_BED_SENSOR_BAD))
        return E_INVALID_STATE;
      fault_flag_ &= ~FAULT_FLAG_BED_SENSOR_BAD;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      LOG_I("Thermistor of Bed has recovered!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_BELOW_MINTEMP:
    LOG_E("BELOW_MINTEMP fault is cleared\n");
    break;

  case ETYPE_OVERRUN_MAXTEMP:
    switch (h) {
    case EHOST_HOTEND0:
      if (!(fault_flag_ & FAULT_FLAG_HOTEND_MAXTEMP))
        return E_SAME_STATE;
      fault_flag_ &= ~FAULT_FLAG_HOTEND_MAXTEMP;
      LOG_I("current temp of hotend is lower than MAXTEMP!\n");
      break;

    case EHOST_BED:
      if (!(fault_flag_ & FAULT_FLAG_BED_MAXTEMP))
        return E_SAME_STATE;
      fault_flag_ &= ~FAULT_FLAG_BED_MAXTEMP;
      LOG_I("current temp of Bed is lower than MAXTEMP!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_OVERRUN_MAXTEMP_AGAIN:
    switch (h) {
    case EHOST_HOTEND0:
      if (!(fault_flag_ & FAULT_FLAG_HOTEND_SHORTCIRCUIT))
        return E_SAME_STATE;
      fault_flag_ &= ~FAULT_FLAG_HOTEND_SHORTCIRCUIT;
      action_ban = ACTION_BAN_NO_HEATING_HOTEND;
      power_ban = POWER_DOMAIN_HOTEND;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action_ban |= ACTION_BAN_NO_WORKING | ACTION_BAN_NO_MOVING;
      }
      LOG_I("current temp of hotend is now lower than MAX TEMP: %d!\n", HEATER_0_MAXTEMP);
      break;

    case EHOST_BED:
      if (!(fault_flag_ & FAULT_FLAG_BED_SHORTCIRCUIT))
        return E_SAME_STATE;
      fault_flag_ &= ~FAULT_FLAG_BED_SHORTCIRCUIT;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      power_ban = POWER_DOMAIN_BED;
      LOG_I("current temp of Bed is now lower than MAX TEMP: %d!\n", BED_MAXTEMP);
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_ABRUPT_TEMP_DROP:
    switch (h) {
    case EHOST_HOTEND0:
      if (!(fault_flag_ & FAULT_FLAG_HOTEND_SENSOR_COMEOFF))
        return E_SAME_STATE;
      fault_flag_ &= ~FAULT_FLAG_HOTEND_SENSOR_COMEOFF;
      action_ban = ACTION_BAN_NO_HEATING_HOTEND;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action_ban |= ACTION_BAN_NO_WORKING;
      }
      LOG_I("abrupt temperature drop in hotend recover.\n");
      break;

    case EHOST_BED:
      if (!(fault_flag_ & FAULT_FLAG_BED_SENSOR_COMEOFF))
        return E_SAME_STATE;
      fault_flag_ &= ~FAULT_FLAG_BED_SENSOR_COMEOFF;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      LOG_I("abrupt temperature drop in bed recover.\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  case ETYPE_SENSOR_COME_OFF:
    switch (h) {
    case EHOST_HOTEND0:
      if (!(fault_flag_ & FAULT_FLAG_HOTEND_SENSOR_COMEOFF))
        return E_SAME_STATE;
      fault_flag_ &= ~FAULT_FLAG_HOTEND_SENSOR_COMEOFF;
      action_ban = ACTION_BAN_NO_HEATING_HOTEND;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action_ban |= ACTION_BAN_NO_WORKING;
      }
      LOG_I("fault of HOTEND SENSOR COME OFF is cleared!\n");
      break;

    case EHOST_BED:
      if (!(fault_flag_ & FAULT_FLAG_BED_SENSOR_COMEOFF))
        return E_SAME_STATE;
      fault_flag_ &= ~FAULT_FLAG_BED_SENSOR_COMEOFF;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      LOG_I("fault of BED SENSOR COME OFF is cleared!\n");
      break;

    default:
      LOG_E("Exception [%d] happened with unknown Host [%d]\n", t, h);
      return E_FAILURE;
      break;
    }
    break;

  default:
    LOG_E("unknown Exception [%d] happened with Host [%d]\n", t, h);
    return E_FAILURE;
    break;
  }

  if (action_ban)
    disable_action_ban(action_ban);

  if (power_ban)
    disable_power_domain(power_ban);

  return E_SUCCESS;
}


void StatusControl::MapFaultFlagToException(uint32_t flag, ExceptionHost &host, ExceptionType &type) {
  switch (flag) {
  case FAULT_FLAG_NO_EXECUTOR:
    host = EHOST_EXECUTOR;
    type = ETYPE_NO_HOST;
    break;

  case FAULT_FLAG_NO_LINEAR:
    host = EHOST_LINEAR;
    type = ETYPE_NO_HOST;
    break;

  case FAULT_FLAG_BED_PORT:
    host = EHOST_BED;
    type = ETYPE_PORT_BAD;
    break;

  case FAULT_FLAG_FILAMENT:
    host = EHOST_EXECUTOR;
    type = ETYPE_RUNOUT;
    break;

  case FAULT_FLAG_LOST_SETTING:
    host = EHOST_MC;
    type = ETYPE_LOST_CFG;
    break;

  case FAULT_FLAG_LOST_EXECUTOR:
    host = EHOST_EXECUTOR;
    type = ETYPE_LOST_HOST;
    break;

  case FAULT_FLAG_POWER_LOSS:
    host = EHOST_MC;
    type = ETYPE_POWER_LOSS;
    break;

  case FAULT_FLAG_HOTEND_HEATFAIL:
    host = EHOST_HOTEND0;
    type = ETYPE_HEAT_FAIL;
    break;

  case FAULT_FLAG_BED_HEATFAIL:
    host = EHOST_BED;
    type = ETYPE_HEAT_FAIL;
    break;

  case FAULT_FLAG_HOTEND_RUNWAWY:
    host = EHOST_HOTEND0;
    type = ETYPE_TEMP_RUNAWAY;
    break;

  case FAULT_FLAG_BED_RUNAWAY:
    host = EHOST_BED;
    type = ETYPE_TEMP_RUNAWAY;
    break;

  case FAULT_FLAG_HOTEND_SENSOR_BAD:
    host = EHOST_HOTEND0;
    type = ETYPE_SENSOR_BAD;
    break;

  case FAULT_FLAG_BED_SENSOR_BAD:
    host = EHOST_BED;
    type = ETYPE_SENSOR_BAD;
    break;

  case FAULT_FLAG_LOST_LINEAR:
    host = EHOST_LINEAR;
    type = ETYPE_LOST_HOST;
    break;

  case FAULT_FLAG_HOTEND_MAXTEMP:
    host = EHOST_HOTEND0;
    type = ETYPE_OVERRUN_MAXTEMP;
    break;

  case FAULT_FLAG_BED_MAXTEMP:
    host = EHOST_BED;
    type = ETYPE_OVERRUN_MAXTEMP;
    break;

  case FAULT_FLAG_HOTEND_SHORTCIRCUIT:
    host = EHOST_HOTEND0;
    type = ETYPE_OVERRUN_MAXTEMP_AGAIN;
    break;

  case FAULT_FLAG_BED_SHORTCIRCUIT:
    host = EHOST_BED;
    type = ETYPE_OVERRUN_MAXTEMP_AGAIN;
    break;

  case FAULT_FLAG_HOTEND_SENSOR_COMEOFF:
    host = EHOST_HOTEND0;
    type = ETYPE_SENSOR_COME_OFF;
    break;

  case FAULT_FLAG_BED_SENSOR_COMEOFF:
    host = EHOST_BED;
    type = ETYPE_SENSOR_COME_OFF;
    break;

  case FAULT_FLAG_UNKNOW_MODEL:
    host = EHOST_MC;
    type = ETYPE_NO_HOST;
    break;

  default:
    LOG_W("cannot map fault flag: 0x%08X\n", flag);
    break;
  }
}

ErrCode StatusControl::ClearExceptionByFaultFlag(uint32_t flag) {
  int i;
  uint32_t index;

  ExceptionHost host;
  ExceptionType type;

  for (i=0; i<32; i++) {
    index = 1<<i;
    if (flag & index) {
      MapFaultFlagToException(flag, host, type);
      LOG_I("will clear except: host: %d, type: %u\n", host, type);
      ClearException(host, type);
    }
  }
}

void StatusControl::CallbackOpenDoor() {
  if (cur_status_ == SYSTAT_WORK) {
    PauseTrigger(TRIGGER_SOURCE_DOOR_OPEN);
    HMI.SendMachineFaultFlag(FAULT_FLAG_DOOR_OPENED);
  }

  fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
}

void StatusControl::CallbackCloseDoor() {
  fault_flag_ &= FAULT_FLAG_DOOR_OPENED;
}

ErrCode StatusControl::ChangeRuntimeEnv(uint8_t param_type, float param) {
  ErrCode ret = E_SUCCESS;

  switch (param_type) {
  case RENV_TYPE_FEEDRATE:
    LOG_I("feedrate scaling: %.2f\n", param);
    if (param > 500 || param < 0) {
      ret = E_PARAM;
      break;
    }
    feedrate_scaling = param;
    break;

  case RENV_TYPE_HOTEND_TEMP:
    LOG_I("new hotend temp: %.2f\n", param);
    if (MACHINE_TYPE_3DPRINT != ExecuterHead.MachineType) {
      ret = E_INVALID_STATE;
      break;
    }

    if (param < HEATER_0_MAXTEMP)
      thermalManager.setTargetHotend((int16_t)param, 0);
    else
      ret = E_PARAM;
    break;

  case RENV_TYPE_BED_TEMP:
    LOG_I("new bed temp: %.2f\n", param);
    if (MACHINE_TYPE_3DPRINT != ExecuterHead.MachineType) {
      ret = E_INVALID_STATE;
      break;
    }

    if (param < BED_MAXTEMP)
      thermalManager.setTargetBed(param);
    else
      ret = E_PARAM;
    break;

  case RENV_TYPE_LASER_POWER:
    LOG_I("new laser power: %.2f\n", param);
    if (MACHINE_TYPE_LASER != ExecuterHead.MachineType) {
      ret = E_INVALID_STATE;
      break;
    }

    if (param > 100 || param < 0)
      ret = E_PARAM;
    else {
      ExecuterHead.Laser.ChangePower(param);
      // change current output when it was turned on
      if (ExecuterHead.Laser.GetTimPwm() > 0)
        ExecuterHead.Laser.On();
    }
    break;

  case RENV_TYPE_ZOFFSET:
    LOG_I("adjust Z offset: %.2f\n", param);
    if (param < -0.5) {
      ret = E_PARAM;
      break;
    }
    // waiting all block buffer are outputed by stepper
    planner.synchronize();

    // for safety, we don't disable leveling here

    // Subtract the mean from all values
    for (uint8_t x = GRID_MAX_POINTS_X; x--;)
      for (uint8_t y = GRID_MAX_POINTS_Y; y--;)
        Z_VALUES(x, y) += param;
    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      bed_level_virt_interpolate();
    #endif

    move_to_limited_z(current_position[Z_AXIS] + param, 5);

    break;

  default:
    LOG_E("invalid parameter type\n", param_type);
    ret = E_PARAM;
    break;
  }

  return ret;
}