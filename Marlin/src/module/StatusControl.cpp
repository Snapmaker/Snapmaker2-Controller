#include "../inc/MarlinConfig.h"


#include "../Marlin.h"
#include "temperature.h"
#include "planner.h"
#include "ExecuterManager.h"
#include "motion.h"
#include "../gcode/gcode.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../module/configuration_store.h"
#include "../gcode/parser.h"
#include "PeriphDevice.h"
#include "StatusControl.h"
#include "PowerPanic.h"
#include "printcounter.h"
#include "stepper.h"
#include "../feature/runout.h"
#include "../snap_module/quickstop_service.h"
#include "../snap_module/snap_dbg.h"
#include "../snap_module/level_service.h"


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
  uint32_t notification = 0;

  if (cur_status_ != SYSTAT_WORK && cur_status_!= SYSTAT_RESUME_WAITING) {
    LOG_W("cannot pause in current status: %d\n", cur_status_);
    return E_NO_SWITCHING_STA;
  }

  // here the operations can be performed many times
  switch (type) {
  case TRIGGER_SOURCE_RUNOUT:
    fault_flag_ |= FAULT_FLAG_FILAMENT;
    SendException(FAULT_FLAG_FILAMENT);
    break;

  case TRIGGER_SOURCE_DOOR_OPEN:
    fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
    SendException(FAULT_FLAG_DOOR_OPENED);
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

  pause_source_ = type;

  // get notification of current task
  xTaskNotifyWait(0, 0, &notification, 0);
  if (notification & HMI_NOTIFY_WAITFOR_HEATING) {
    taskENTER_CRITICAL();

    // if we are waiting for heatup, abort the waiting
    wait_for_heatup = false;

    powerpanic.SaveCmdLine(CommandLine[cmd_queue_index_r]);

    taskEXIT_CRITICAL();
  }

  // clear event in queue to marlin
  xMessageBufferReset(snap_tasks->event_queue);

  quickstop.Trigger(QS_SOURCE_PAUSE);

  return E_SUCCESS;
}


ErrCode StatusControl::PreProcessStop() {
  // recover scaling
  feedrate_percentage = 100;

  // set temp to 0
  if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
    thermalManager.setTargetBed(0);
    HOTEND_LOOP() { thermalManager.setTargetHotend(0, e); }

    // if user abort the work, FAN0 will not be closed
    // set target temp to 0 only make FAN1 be closed
    ExecuterHead.SetFan(0, 0);
  }

  // diable power panic data
  powerpanic.Data.Valid = 0;

  print_job_timer.stop();

  if (ExecuterHead.MachineType == MACHINE_TYPE_LASER) {
    ExecuterHead.Laser.Off();
  }
}

/**
 * Triggle the stop
 * return: true if stop triggle success, or false
 */
ErrCode StatusControl::StopTrigger(TriggerSource source, uint16_t event_opc) {
  uint32_t notification = 0;

  if (cur_status_ != SYSTAT_WORK && cur_status_ != SYSTAT_RESUME_WAITING &&
      cur_status_ != SYSTAT_PAUSE_FINISH) {
    LOG_E("cannot stop in current status[%d]\n", cur_status_);
    return E_NO_SWITCHING_STA;
  }

  switch(source) {
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

  default:
    break;
  }

  // save live z offset
  levelservice.SaveLiveZOffset();

  // if we already finish quick stop, just change system status
  // disable power-loss data, and exit with success
  if (cur_status_ == SYSTAT_PAUSE_FINISH) {
    // to make StopProcess work, cur_status_ need to be SYSTAT_END_FINISH
    cur_status_ = SYSTAT_IDLE;
    pause_source_ = TRIGGER_SOURCE_NONE;

    PreProcessStop();

    if (source == TRIGGER_SOURCE_SC)
      FinishSystemStatusChange(event_opc, 0);

    LOG_I("Stop in PAUSE, trigger source: %d\n", source);
    return E_SUCCESS;
  }

  cur_status_ = SYSTAT_END_TRIG;

  if (event_opc == SYSCTL_OPC_FINISH) {
    // if screen tell us Gcode is ended, wait for all movement output
    // because planner.synchronize() will call HMI process nestedly,
    // and maybe some function will check the status, so we change the status
    // firstly
    stop_type_ = STOP_TYPE_FINISH;
    planner.synchronize();
  }
  else
    stop_type_ = STOP_TYPE_ABORTED;

  // get notification of current task
  xTaskNotifyWait(0, 0, &notification, 0);
  if (notification & HMI_NOTIFY_WAITFOR_HEATING) {
    taskENTER_CRITICAL();
    // if we are waiting for heatup, abort the waiting
    wait_for_heatup = false;

    taskEXIT_CRITICAL();
  }

  stop_source_ = source;

  quickstop.Trigger(QS_SOURCE_STOP);

  return E_SUCCESS;
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
  LOG_I("restore ponit: X :%.3f, Y:%.3f, Z:%.3f, E:%.3f\n", powerpanic.Data.PositionData[X_AXIS],
    powerpanic.Data.PositionData[Y_AXIS], powerpanic.Data.PositionData[Z_AXIS],
    powerpanic.Data.PositionData[E_AXIS]);

  // the positions we recorded are logical positions, so cannot use native movement API
  // restore X, Y
  move_to_limited_xy(powerpanic.Data.PositionData[X_AXIS], powerpanic.Data.PositionData[Y_AXIS], 60);
  planner.synchronize();

  // restore Z
  if (MACHINE_TYPE_CNC == ExecuterHead.MachineType) {
    move_to_limited_z(powerpanic.Data.PositionData[Z_AXIS] + 15, 30);
    move_to_limited_z(powerpanic.Data.PositionData[Z_AXIS], 10);
  }
  else {
    move_to_limited_z(powerpanic.Data.PositionData[Z_AXIS], 30);
  }
  planner.synchronize();
}

void inline StatusControl::resume_3dp(void) {
	current_position[E_AXIS] += 20;
	line_to_current_position(5);
	planner.synchronize();

	// try to cut out filament
	current_position[E_AXIS] -= 6;
	line_to_current_position(50);
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
ErrCode StatusControl::ResumeTrigger(TriggerSource source) {
  uint8_t event[2] = {EID_SYS_CTRL_REQ, SYSCTL_OPC_RESUME};

  if (action_ban & ACTION_BAN_NO_WORKING) {
    LOG_E("System Fault! Now cannot start working!\n");
    return E_NO_WORKING;
  }

  if (cur_status_ != SYSTAT_PAUSE_FINISH) {
    LOG_W("cannot trigger in current status: %d\n", cur_status_);
    return E_NO_SWITCHING_STA;
  }

  switch (source) {
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
    LOG_W("invalid trigger source: %d\n", source);
    return E_FAILURE;
    break;
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

  // send event to marlin task to handle
  if (xMessageBufferSend(snap_tasks->event_queue, event, 2, portTICK_PERIOD_MS * 100)) {
    cur_status_ = SYSTAT_RESUME_TRIG;
    return E_SUCCESS;
  }
  else
    return E_FAILURE;
}


ErrCode StatusControl::ResumeProcess() {
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

  return FinishSystemStatusChange(SYSCTL_OPC_RESUME, 0);
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
    StopTrigger(TRIGGER_SOURCE_EXCEPTION, INVALID_OP_CODE);
    return E_NO_WORKING;
  }

  if (cur_status_ != SYSTAT_RESUME_WAITING)
    return E_NO_SWITCHING_STA;

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    if (runout.is_filament_runout()) {
      LOG_E("No filemant! Please insert filemant!\n");
      PauseTrigger(TRIGGER_SOURCE_RUNOUT);
      return E_NO_FILAMENT;
    }
    // filament has been retracted for 6mm in resume process
    // we pre-extruder 6.5 to get better print quality
    current_position[E_AXIS] += 6.5;
    line_to_current_position(5);
    planner.synchronize();
    current_position[E_AXIS] = powerpanic.Data.PositionData[E_AXIS];
    sync_plan_position_e();
    break;

  case MACHINE_TYPE_CNC:
  case MACHINE_TYPE_LASER:
    if (Periph.IsDoorOpened()) {
      LOG_E("Door is opened, please close the door!\n");
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

  LOG_I("got 1rst cmd after resume\n");
  cur_status_ = SYSTAT_WORK;
  //lightbar.set_state(LB_STATE_WORKING);

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

  return E_SUCCESS;
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
    LOG_I("filament has run out!\n");
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
      LOG_E("heating failed for hotend, please check heating module & sensor! temp: %.2f / %d\n",
        thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_HEATFAIL)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_HEATFAIL;
      action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_BED;
      LOG_E("heating failed for bed, please check heating module & sensor! temp: %.2f / %d\n",
        thermalManager.degBed(), thermalManager.degTargetBed());
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
      LOG_E("thermal run away of hotend! temp: %.2f / %d\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
      break;

    case EHOST_BED:
      LOG_W("Not handle exception: BED TEMP RUNAWAY\n");
      if (fault_flag_ & FAULT_FLAG_BED_RUNAWAY)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_RUNAWAY;
      // action = EACTION_STOP_WORKING | EACTION_STOP_HEATING_BED;
      LOG_E("thermal run away of Bed! temp: %.2f / %d\n", thermalManager.degBed(), thermalManager.degTargetBed());
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
      LOG_E("Detected error in sensor of Hotend! temp: %.2f / %d\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_SENSOR_BAD)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_SENSOR_BAD;
      action = EACTION_STOP_HEATING_BED;
      action_ban = ACTION_BAN_NO_HEATING_BED;
      if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
        action |= EACTION_STOP_WORKING;
        LOG_E("Detected error in sensor of Bed! temp: %.2f / %d\n", thermalManager.degBed(), thermalManager.degTargetBed());
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
      LOG_E("current temp [%.2f] of hotend is more higher than MAXTEMP: %d!\n", thermalManager.degHotend(0), HEATER_0_MAXTEMP + 10);
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
      LOG_E("current temp [%.2f] of Bed is more higher than MAXTEMP: %d!\n", thermalManager.degBed(), BED_MAXTEMP + 5);
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
      LOG_E("temperature of hotend dropped abruptly! temp: %.2f / %d\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
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
      LOG_E("current temperature of bed dropped abruptly! temp: %.2f / %d\n", thermalManager.degBed(), thermalManager.degTargetBed());
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
      LOG_E("Thermistor of hotend maybe come off! temp: %.2f / %d\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
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
      LOG_E("Thermistor of bed maybe come off! temp: %.2f / %d\n", thermalManager.degBed(), thermalManager.degTargetBed());
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
    StopTrigger(TRIGGER_SOURCE_EXCEPTION, INVALID_OP_CODE);
  }
  else if (action & EACTION_PAUSE_WORKING) {
    PauseTrigger(TRIGGER_SOURCE_EXCEPTION);
  }

  fault_flag_ |= new_fault_flag;
  SendException(fault_flag_);

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

  return E_SUCCESS;
}

void StatusControl::CallbackOpenDoor() {
  if ((SYSTAT_WORK == cur_status_ ) ||(SYSTAT_RESUME_WAITING == cur_status_)) {
    PauseTrigger(TRIGGER_SOURCE_DOOR_OPEN);
  }

  fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
}

void StatusControl::CallbackCloseDoor() {
  fault_flag_ &= FAULT_FLAG_DOOR_OPENED;
}

ErrCode StatusControl::CheckIfSendWaitEvent() {
  Event_t event;
  ErrCode err;

  // make sure we are working
  if (SystemStatus.GetCurrentStatus() == SYSTAT_WORK) {
    // and no movement planned
    if(!planner.movesplanned()) {
      // and we have replied screen
      if (current_line_ && current_line_ == debug.GetSCGcodeLine()) {
        // then we known maybe screen lost out last reply
        LOG_I("waiting HMI command, current line: %u\n", current_line_);
        event.id = EID_SYS_CTRL_ACK;
        event.op_code = SYSCTL_OPC_WAIT_EVENT;
        event.data = &err;
        event.length = 1;
        err = E_FAILURE;

        return hmi.Send(event);
      }
    }
  }

  return E_SUCCESS;
}


#if 0
ErrCode StatusControl::SendStatus(Event_t &event) {
  SystemStatus_t sta;

  int32_t tmp_i32;
  int16_t tmp_i16;

  uint32_t tmp_u32;

  CheckIfSendWaitEvent();

  // save to use original event to construct new event
  event.data = (uint8_t *)&sta;
  event.length = sizeof(SystemStatus_t);

  // current logical position
  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[X_AXIS], X_AXIS) * 1000);
  hmi.ToPDUBytes((uint8_t *)&sta.x, (uint8_t *)&tmp_i32, 4);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[Y_AXIS], Y_AXIS) * 1000);
  hmi.ToPDUBytes((uint8_t *)&sta.y, (uint8_t *)&tmp_i32, 4);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[Z_AXIS], Z_AXIS) * 1000);
  hmi.ToPDUBytes((uint8_t *)&sta.z, (uint8_t *)&tmp_i32, 4);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[E_AXIS], E_AXIS) * 1000);
  hmi.ToPDUBytes((uint8_t *)&sta.e, (uint8_t *)&tmp_i32, 4);

  // temperatures of Bed
  tmp_i16 = (int16_t)thermalManager.degBed();
  hmi.ToPDUBytes((uint8_t *)&sta.bed_current_temp, (uint8_t *)&tmp_i16, 2);

  tmp_i16 = (int16_t)thermalManager.degTargetBed();
  hmi.ToPDUBytes((uint8_t *)&sta.bed_target_temp, (uint8_t *)&tmp_i16, 2);

  // temperatures of hotend
  tmp_i16 = (int16_t)thermalManager.degHotend(0);
  hmi.ToPDUBytes((uint8_t *)&sta.hotend_current_temp, (uint8_t *)&tmp_i16, 2);
  tmp_i16 = (int16_t)thermalManager.degTargetHotend(0);
  hmi.ToPDUBytes((uint8_t *)&sta.hotend_target_temp, (uint8_t *)&tmp_i16, 2);

  // save last feedrate
  tmp_i16 = (int16_t)last_feedrate;
  hmi.ToPDUBytes((uint8_t *)&sta.feedrate, (uint8_t *)&tmp_i16, 2);

  // laser power
  tmp_u32 = ExecuterHead.Laser.GetPower();
  hmi.ToPDUBytes((uint8_t *)&sta.laser_power, (uint8_t *)&tmp_u32, 2);

  // RPM of CNC
  tmp_u32 = ExecuterHead.CNC.GetRPM();
  hmi.ToPDUBytes((uint8_t *)&sta.cnc_rpm, (uint8_t *)&tmp_u32, 2);

  // system status
  sta.system_state = (uint8_t)SystemStatus.MapCurrentStatusForSC();

  // Add-On status
  sta.addon_state = (uint8_t)SystemStatus.GetPeriphDeviceStatus();

  // executor type
  sta.executor_type = ExecuterHead.MachineType;

  return hmi.Send(event);
}
#else
ErrCode StatusControl::SendStatus(Event_t &event) {
  SystemStatus_t sta;

  int i = 0;
  uint8_t *buff = (uint8_t *)&sta;

  int32_t   tmp_i32;
  int16_t   tmp_i16;
  uint32_t  tmp_u32;
  float     tmp_f32;

  CheckIfSendWaitEvent();

  // save to use original event to construct new event
  event.data = buff;
  event.length = sizeof(SystemStatus_t);

  // current logical position
  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[X_AXIS], X_AXIS) * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[Y_AXIS], Y_AXIS) * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[Z_AXIS], Z_AXIS) * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[E_AXIS], E_AXIS) * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);

  // temperatures of Bed
  tmp_i16 = (int16_t)thermalManager.degBed();
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);

  tmp_i16 = (int16_t)thermalManager.degTargetBed();
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);

  // temperatures of hotend
  tmp_i16 = (int16_t)thermalManager.degHotend(0);
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);
  tmp_i16 = (int16_t)thermalManager.degTargetHotend(0);
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);

  // save last feedrate
  tmp_f32 = MMS_SCALED(feedrate_mm_s) * 60;
  tmp_i16 = (int16_t)tmp_f32;
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);

  // laser power
  tmp_u32 = ExecuterHead.Laser.GetPower();
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_u32, i);

  // RPM of CNC
  tmp_u32 = ExecuterHead.CNC.GetRPM();
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_u32, i);

  // system status
  sta.system_state = (uint8_t)SystemStatus.MapCurrentStatusForSC();

  // Add-On status
  sta.addon_state = (uint8_t)SystemStatus.GetPeriphDeviceStatus();

  // executor type
  sta.executor_type = ExecuterHead.MachineType;

  return hmi.Send(event);
}
#endif


ErrCode StatusControl::SendException(Event_t &event) {
  LOG_I("SC req Exception\n");

  return SendException(fault_flag_);
}


ErrCode StatusControl::SendException(uint32_t fault) {
  Event_t event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_GET_EXCEPTION};
  uint8_t buff[4];

  event.length = 4;
  event.data = buff;

  WORD_TO_PDU_BYTES(buff, fault);

  return hmi.Send(event);
}


ErrCode StatusControl::ChangeSystemStatus(Event_t &event) {
  ErrCode err = E_SUCCESS;
  bool need_ack = true;

  switch (event.op_code)
  {
  case SYSCTL_OPC_START_WORK:
    LOG_I("SC req START work\n");
    err = StartWork(TRIGGER_SOURCE_SC);
    if (err == E_SUCCESS)
      current_line_ = 0;
    break;

  case SYSCTL_OPC_PAUSE:
    LOG_I("SC req PAUSE\n");
    err = PauseTrigger(TRIGGER_SOURCE_SC);
    if (err == E_SUCCESS)
      need_ack = false;
    break;

  case SYSCTL_OPC_RESUME:
    if (xTaskGetCurrentTaskHandle() == snap_tasks->hmi) {
      LOG_I("SC req RESUME\n");
      err = ResumeTrigger(TRIGGER_SOURCE_SC);
      if (err == E_SUCCESS)
        need_ack = false;
    }
    else {
      err = ResumeProcess();
      if (err == E_SUCCESS) {
        powerpanic.SaveCmdLine(powerpanic.Data.FilePosition);
        if (powerpanic.Data.FilePosition > 0) {
          current_line_ =  powerpanic.Data.FilePosition - 1;
        }
        else {
          current_line_ = 0;
        }

        SNAP_DEBUG_SET_GCODE_LINE(current_line_);
        LOG_I("RESUME over\n");
        return E_SUCCESS;
      }
      else {
        LOG_I("RESUME failed: %u\n", err);
        return E_FAILURE;
      }
    }
    break;

  case SYSCTL_OPC_STOP:
  case SYSCTL_OPC_FINISH:
    LOG_I("SC req %s\n", (event.op_code == SYSCTL_OPC_STOP)? "STOP" : "FINISH");
    err = StopTrigger(TRIGGER_SOURCE_SC, event.op_code);
    if (err == E_SUCCESS)
      need_ack = false;
    break;

  default:
    break;
  }

  event.length = 1;
  event.data = &err;

  if (err == E_SUCCESS) {
    LOG_I("SC req -> Sucess\n");
  }
  else {
    LOG_I("SC req -> failed\n");
  }

  if (need_ack)
    return hmi.Send(event);
  else
    return err;
}


ErrCode StatusControl::FinishSystemStatusChange(uint8_t op_code, uint8_t result) {
  Event_t event = {EID_SYS_CTRL_ACK, op_code};

  event.data = &result;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode StatusControl::SendLastLine(Event_t &event) {
  uint8_t buff[6];

  if (GetCurrentStage() != SYSTAGE_PAUSE) {
    LOG_I("SC req last line: %u\n", powerpanic.pre_data_.FilePosition);

    buff[0] = powerpanic.pre_data_.Valid;
    buff[1] = powerpanic.pre_data_.GCodeSource;
    WORD_TO_PDU_BYTES(buff + 2, powerpanic.pre_data_.FilePosition);
  }
  else {
    LOG_I("SC req last line: %u\n", powerpanic.Data.FilePosition);

    buff[0] = powerpanic.Data.Valid;
    buff[1] = powerpanic.Data.GCodeSource;
    WORD_TO_PDU_BYTES(buff + 2, powerpanic.Data.FilePosition);
  }

  event.data = buff;
  event.length = 6;

  return hmi.Send(event);
}


ErrCode StatusControl::ClearException(Event_t &event) {
  ErrCode err = E_SUCCESS;

  if (event.length == 0) {
    LOG_I("SC req clear power loss bits\n");
    SystemStatus.ClearExceptionByFaultFlag(FAULT_FLAG_POWER_LOSS);
    if (powerpanic.pre_data_.Valid == 1) {
      // clear flash data
      LOG_I("mask flash data ...");
      powerpanic.MaskPowerPanicData();
      powerpanic.pre_data_.Valid = 0;
      LOG_I("Done!\n");
    }
  }
  else if (event.length == 4) {
    uint32_t bit_to_clear = 0;

    PDU_TO_LOCAL_WORD(bit_to_clear, event.data);
    LOG_I("SC req clear exception, fault bits: 0x%08X\n", bit_to_clear);

    bit_to_clear &= FAULT_FLAG_SC_CLEAR_MASK;
    SystemStatus.ClearExceptionByFaultFlag(bit_to_clear);
  }
  else {
    LOG_E("too many data: %d\n", event.length);
    err = E_FAILURE;
  }

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode StatusControl::RecoverFromPowerLoss(Event_t &event) {
  ErrCode err = E_SUCCESS;

  LOG_I("SC trigger restore from power-loss\n");

  event.data = &err;
  event.length = 1;

  SysStatus cur_status = SystemStatus.GetCurrentStatus();

  if (cur_status != SYSTAT_IDLE) {
    LOG_E("cannot trigger recovery at current status: %d\n", cur_status);
    err = E_NO_SWITCHING_STA;
  }
  else {
    // screen bug: why will we receive two consecutive recovery command @TODO
    SystemStatus.SetCurrentStatus(SYSTAT_RESUME_TRIG);
    err = powerpanic.ResumeWork();
    if (err == E_SUCCESS) {
      SystemStatus.SetCurrentStatus(SYSTAT_RESUME_WAITING);
      SystemStatus.SetWorkingPort(WORKING_PORT_SC);
      powerpanic.Data.FilePosition = powerpanic.pre_data_.FilePosition;
      if (powerpanic.Data.FilePosition > 0)
        current_line_ =  powerpanic.Data.FilePosition - 1;
      else
        current_line_ = 0;
      SNAP_DEBUG_SET_GCODE_LINE(current_line_);
      powerpanic.SaveCmdLine(powerpanic.Data.FilePosition);
      LOG_I("trigger RESTORE: ok\n");
    }
    else {
      LOG_I("trigger RESTORE: failed, err = %d\n", err);
      SystemStatus.SetCurrentStatus(cur_status);
    }
  }

  return hmi.Send(event);
}


ErrCode StatusControl::SendHomeAndCoordinateStatus(Event_t &event) {
  uint8_t buff[16] = {0};
  int32_t pos_shift[XYZ];

  int i = 0;

  event.data = buff;

  if (all_axes_homed()) {
    buff[i++] = 0;
  }
  else {
    buff[i++] = 1;
  }

  if (gcode.active_coordinate_system < 0) {
    // number
    buff[i++] = 0;
    // state
    buff[i++] = 0;
    pos_shift[X_AXIS] = (int32_t)(position_shift[X_AXIS] * 1000);
    pos_shift[Y_AXIS] = (int32_t)(position_shift[Y_AXIS] * 1000);
    pos_shift[Z_AXIS] = (int32_t)(position_shift[Z_AXIS] * 1000);
  }
  else {
    buff[i++] = gcode.active_coordinate_system + 1;
    // check state
    if ((position_shift[X_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS]) &&
        (position_shift[Y_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS]) &&
        (position_shift[Z_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS])) {
      buff[i++] = 0;
    }
    else {
      buff[i++] = 1;
    }
    pos_shift[X_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS] * 1000);
    pos_shift[Y_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS] * 1000);
    pos_shift[Z_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS] * 1000);
  }

  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, pos_shift[X_AXIS], i);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, pos_shift[Y_AXIS], i);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, pos_shift[Z_AXIS], i);

  event.length = i;

  return hmi.Send(event);
}


ErrCode StatusControl::ChangeRuntimeEnv(Event_t &event) {
  ErrCode ret = E_SUCCESS;

  float param;
  float tmp_f32;

  PDU_TO_LOCAL_WORD(param, event.data + 1);

  param /= 1000;

  switch (event.data[0]) {
  case RENV_TYPE_FEEDRATE:
    if (param > 500 || param < 0) {
      LOG_E("invalid feedrate scaling: %.2f\n", param);
      ret = E_PARAM;
      break;
    }
    feedrate_percentage = (int16_t)param;
    LOG_I("feedrate scaling: %d\n", feedrate_percentage);
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
    ret = levelservice.UpdateLiveZOffset(param);
    break;

  case RENV_TYPE_CNC_POWER:
    if (MACHINE_TYPE_CNC != ExecuterHead.MachineType) {
      ret = E_INVALID_STATE;
      LOG_E("Not CNC toolhead!\n");
      break;
    }

    if (param > 100 || param < 0) {
      ret = E_PARAM;
      LOG_E("out of range: %.2f\n", param);
    }
    else {
      ExecuterHead.CNC.ChangePower(param);
      // change current output when it was turned on
      if (ExecuterHead.CNC.GetRPM() > 0)
        ExecuterHead.CNC.On();
      LOG_I("new CNC power: %.2f\n", param);
    }
    break;

  default:
    LOG_E("invalid parameter type\n", event.data[0]);
    ret = E_PARAM;
    break;
  }

  event.length = 1;
  event.data = &ret;
  return hmi.Send(event);
}


ErrCode StatusControl::GetRuntimeEnv(Event_t &event) {
  int   tmp_i32;
  uint8_t buff[5];

  LOG_I("SC get env: %u\n", event.data[0]);

  buff[0] = E_SUCCESS;

  switch (event.data[0]) {
  case RENV_TYPE_FEEDRATE:
    tmp_i32 = (int)(powerpanic.pre_data_.feedrate_percentage * 1000.0f);
    WORD_TO_PDU_BYTES(buff+1, (int)tmp_i32);
    LOG_I("feedrate_percentage: %d\n", powerpanic.pre_data_.feedrate_percentage);
    break;

  case RENV_TYPE_LASER_POWER:
    tmp_i32 = (int)(powerpanic.pre_data_.laser_percent * 1000.0f);
    WORD_TO_PDU_BYTES(buff+1, (int)tmp_i32);
    LOG_I("laser power: %.2f\n", powerpanic.pre_data_.laser_percent);
    break;

  case RENV_TYPE_ZOFFSET:
    tmp_i32 = (int)(levelservice.live_z_offset() * 1000);
    WORD_TO_PDU_BYTES(buff+1, tmp_i32);
    LOG_I("live z offset: %.3f\n", levelservice.live_z_offset());
    break;

  case RENV_TYPE_CNC_POWER:
    tmp_i32 = (int)(powerpanic.pre_data_.cnc_power * 1000);
    WORD_TO_PDU_BYTES(buff+1, tmp_i32);
    LOG_I("laser power: %.2f\n", powerpanic.pre_data_.cnc_power);
    break;

  default:
    buff[0] = E_FAILURE;
    LOG_I("cannot get this env: %u\n", event.data[0]);
    break;
  }

  event.data = buff;
  event.length = 5;

  hmi.Send(event);
}


ErrCode StatusControl::GetMachineSize(Event_t &event) {
  uint8_t buffer[64];
  uint16_t i = 0;

  int32_t  tmp_i32;
  uint32_t tmp_u32;


  buffer[i++] = 0;

  //Machine size type
  buffer[i++] = CanModules.GetMachineSizeType();

  //Size
  tmp_u32 = (uint32_t) (X_MAX_POS * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_u32, i);
  tmp_u32 = (uint32_t) (Y_MAX_POS * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_u32, i);
  tmp_u32 = (uint32_t) (Z_MAX_POS * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_u32, i);

  //Home Dir
  tmp_i32 = (int32_t) (X_HOME_DIR);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);
  tmp_i32 = (int32_t) (Y_HOME_DIR);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);
  tmp_i32 = (int32_t) (Z_HOME_DIR);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);

  //Dir
  tmp_i32 = X_DIR == true?(int32_t)1:(int32_t)-1;
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);
  tmp_i32 = Y_DIR == true?(int32_t)1:(int32_t)-1;
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);
  tmp_i32 = Z_DIR == true?(int32_t)1:(int32_t)-1;
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);

  //Offset
  tmp_i32 = (int32_t) (home_offset[X_AXIS] *1000.0f);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);
  tmp_i32 = (int32_t) (home_offset[Y_AXIS] *1000.0f);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);
  tmp_i32 = (int32_t) (home_offset[Z_AXIS] *1000.0f);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, i);

  event.data = buffer;
  event.length = i;

  return hmi.Send(event);
}


ErrCode StatusControl::CallbackPreQS(QuickStopSource source) {
  switch (source) {
  case QS_SOURCE_PAUSE:

    print_job_timer.pause();

    // reset the status of filament monitor
    runout.reset();

    // make sure laser is off
    // won't turn off laser in powerpanic.SaveEnv(), it's call by stepper ISR
    // because it may call CAN transmisson function
    if (ExecuterHead.MachineType == MACHINE_TYPE_LASER) {
      ExecuterHead.Laser.Off();
    }
    break;

  case QS_SOURCE_STOP:
    PreProcessStop();
    break;

  default:
    break;
  }
}


ErrCode StatusControl::CallbackPostQS(QuickStopSource source) {
  switch (source) {
  case QS_SOURCE_PAUSE:


    if (pause_source_ == TRIGGER_SOURCE_SC) {
      // ack HMI
      FinishSystemStatusChange(SYSCTL_OPC_PAUSE, 0);
    }

    pause_source_ = TRIGGER_SOURCE_NONE;

    LOG_I("Finish pause\n\n");
    cur_status_ = SYSTAT_PAUSE_FINISH;
    break;

  case QS_SOURCE_STOP:
    // tell Screen we finish stop
    if (stop_source_ == TRIGGER_SOURCE_SC) {
      // ack HMI
      if (stop_type_ == STOP_TYPE_ABORTED)
        FinishSystemStatusChange(SYSCTL_OPC_STOP, 0);
      else
        FinishSystemStatusChange(SYSCTL_OPC_FINISH, 0);
    }

    // clear stop type because stage will be changed
    stop_source_ = TRIGGER_SOURCE_NONE;
    cur_status_ = SYSTAT_IDLE;

    LOG_I("Finish stop\n\n");
    break;

  default:
    break;
  }
}

