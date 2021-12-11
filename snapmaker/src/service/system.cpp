/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "../common/config.h"
#include "../common/debug.h"

#include "../module/linear.h"
#include "../module/enclosure.h"
#include "../module/toolhead_3dp.h"
#include "../module/toolhead_cnc.h"
#include "../module/toolhead_laser.h"

#include "../snapmaker.h"

#include "system.h"
#include "bed_level.h"
#include "quick_stop.h"
#include "power_loss_recovery.h"

#include "src/Marlin.h"
#include "src/module/printcounter.h"
#include "src/module/stepper.h"
#include "src/module/configuration_store.h"
#include "src/module/temperature.h"
#include "src/module/planner.h"
#include "src/module/motion.h"
#include "src/gcode/gcode.h"
#include "src/gcode/parser.h"
#include "src/feature/bedlevel/bedlevel.h"
#include "src/feature/runout.h"


SystemService systemservice;

/**
 * Init
 */
void SystemService::Init() {
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
ErrCode SystemService::PauseTrigger(TriggerSource type)
{
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

  // check if we are waiting for heating
  if (xEventGroupGetBits(sm2_handle->event_group) & EVENT_GROUP_WAIT_FOR_HEATING) {
    taskENTER_CRITICAL();

    // if we are waiting for heatup, abort the waiting
    wait_for_heatup = false;
    // save the command line of heating Gcode
    taskEXIT_CRITICAL();
  }
  pl_recovery.SaveCmdLine(CommandLine[cmd_queue_index_r]);

  // clear event in queue to marlin
  xMessageBufferReset(sm2_handle->event_queue);

  quickstop.Trigger(QS_SOURCE_PAUSE);

  return E_SUCCESS;
}


ErrCode SystemService::PreProcessStop() {
  // recover scaling
  feedrate_percentage = 100;

  // set temp to 0
  if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
    thermalManager.setTargetBed(0);
    HOTEND_LOOP() { thermalManager.setTargetHotend(0, e); }

    // if user abort the work, FAN0 will not be closed
    // set target temp to 0 only make FAN1 be closed
    printer1->SetFan(0, 0);
  }

  // diable power panic data
  pl_recovery.cur_data_.Valid = 0;

  print_job_timer.stop();

  if ((ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER) || (ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_10W)) {
    laser->TurnOff();
    is_waiting_gcode = false;
    is_laser_on = false;
  }
  gocde_pack_start_line(0);
  return E_SUCCESS;
}

/**
 * Triggle the stop
 * return: true if stop triggle success, or false
 */
ErrCode SystemService::StopTrigger(TriggerSource source, uint16_t event_opc) {
  if (cur_status_ != SYSTAT_WORK && cur_status_ != SYSTAT_RESUME_WAITING &&
      cur_status_ != SYSTAT_PAUSE_FINISH && source != TRIGGER_SOURCE_STOP_BUTTON) {
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

  case TRIGGER_SOURCE_STOP_BUTTON:
    LOG_I("current working is stopped!");
    PreProcessStop();
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

  // check if we are wating for heating
  if (xEventGroupGetBits(sm2_handle->event_group) & EVENT_GROUP_WAIT_FOR_HEATING) {
    taskENTER_CRITICAL();
    // if we are waiting for heatup, abort the waiting
    wait_for_heatup = false;

    taskEXIT_CRITICAL();
  }

  stop_source_ = source;

  if (source == TRIGGER_SOURCE_STOP_BUTTON) {
    quickstop.Trigger(QS_SOURCE_STOP_BUTTON);
  } else {
    quickstop.Trigger(QS_SOURCE_STOP);
  }

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

void inline SystemService::RestoreXYZ(void) {
  LOG_I("restore ponit: X :%.3f, Y:%.3f, B:%.3f, Z:%.3f, E:%.3f\n", pl_recovery.cur_data_.PositionData[X_AXIS],
    pl_recovery.cur_data_.PositionData[Y_AXIS], pl_recovery.cur_data_.PositionData[Z_AXIS],
    pl_recovery.cur_data_.PositionData[B_AXIS], pl_recovery.cur_data_.PositionData[E_AXIS]);

  // the positions we recorded are logical positions, so cannot use native movement API
  // restore X, Y
  move_to_limited_xy(pl_recovery.cur_data_.PositionData[X_AXIS], pl_recovery.cur_data_.PositionData[Y_AXIS], 60);
  planner.synchronize();

  // restore Z
  if (MODULE_TOOLHEAD_CNC == ModuleBase::toolhead()) {
    move_to_limited_z(pl_recovery.cur_data_.PositionData[Z_AXIS] + 15, 30);
    move_to_limited_z(pl_recovery.cur_data_.PositionData[Z_AXIS], 10);
  }
  else {
    move_to_limited_z(pl_recovery.cur_data_.PositionData[Z_AXIS], 30);
  }
  planner.synchronize();
}

void inline SystemService::resume_3dp(void) {
	current_position[E_AXIS] += 20;
	line_to_current_position(5);
	planner.synchronize();

	// try to cut out filament
	current_position[E_AXIS] -= 6;
	line_to_current_position(50);
	planner.synchronize();
}


void inline SystemService::resume_cnc(void) {
  // enable CNC motor
  LOG_I("restore CNC power: %f\n", pl_recovery.cur_data_.cnc_power);
  cnc.SetOutput(pl_recovery.cur_data_.cnc_power);
}

void inline SystemService::resume_laser(void) {

}

/**
 * Resume Process
 */
ErrCode SystemService::ResumeTrigger(TriggerSource source) {
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

  switch (ModuleBase::toolhead()) {
  case MODULE_TOOLHEAD_3DP:
    if (runout.is_filament_runout()) {
      LOG_E("No filemant!\n");
      fault_flag_ |= FAULT_FLAG_FILAMENT;
      return E_NO_FILAMENT;
    }
    break;

  case MODULE_TOOLHEAD_CNC:
  case MODULE_TOOLHEAD_LASER:
  case MODULE_TOOLHEAD_LASER_10W:
    if (enclosure.DoorOpened()) {
      LOG_E("Door is opened!\n");
      fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
      return E_DOOR_OPENED;
    }
    break;

  default:
    break;
  }

  // send event to marlin task to handle
  if (xMessageBufferSend(sm2_handle->event_queue, event, 2, pdMS_TO_TICKS(100))) {
    cur_status_ = SYSTAT_RESUME_TRIG;
    return E_SUCCESS;
  }
  else
    return E_FAILURE;
}


ErrCode SystemService::ResumeProcess() {
  switch(ModuleBase::toolhead()) {
  case MODULE_TOOLHEAD_3DP:
    set_bed_leveling_enabled(true);
    resume_3dp();
    break;

  case MODULE_TOOLHEAD_CNC:
    resume_cnc();
    break;

  case MODULE_TOOLHEAD_LASER:
  case MODULE_TOOLHEAD_LASER_10W:
    resume_laser();
    break;

  default:
    break;
  }

  RestoreXYZ();

  // restore speed
  saved_g0_feedrate_mm_s = pl_recovery.cur_data_.TravelFeedRate;
  saved_g1_feedrate_mm_s = pl_recovery.cur_data_.PrintFeedRate;

  // clear command queue
  clear_command_queue();
  clear_hmi_gcode_queue();

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
ErrCode SystemService::ResumeOver() {
  // if exception happened duration resuming work
  // give a opportunity to stop working
  if (action_ban & ACTION_BAN_NO_WORKING) {
    LOG_E("System Fault! Now cannot start working!\n");
    StopTrigger(TRIGGER_SOURCE_EXCEPTION, SSTP_INVALID_OP_CODE);
    return E_NO_WORKING;
  }

  if (cur_status_ != SYSTAT_RESUME_WAITING)
    return E_NO_SWITCHING_STA;

  switch (ModuleBase::toolhead()) {
  case MODULE_TOOLHEAD_3DP:
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
    current_position[E_AXIS] = pl_recovery.cur_data_.PositionData[E_AXIS];
    sync_plan_position_e();
    break;

  case MODULE_TOOLHEAD_CNC:
  case MODULE_TOOLHEAD_LASER:
  case MODULE_TOOLHEAD_LASER_10W:
    if (enclosure.DoorOpened()) {
      LOG_E("Door is opened, please close the door!\n");
      PauseTrigger(TRIGGER_SOURCE_DOOR_OPEN);
      return E_DOOR_OPENED;
    }

    if ((MODULE_TOOLHEAD_LASER == ModuleBase::toolhead()) || (MODULE_TOOLHEAD_LASER_10W == ModuleBase::toolhead())) {
      if (pl_recovery.cur_data_.laser_pwm > 0)
        laser->TurnOn();
    }
    break;

  default:
    LOG_E("invalid machine type: %d\n", ModuleBase::toolhead());
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
SysStage SystemService::GetCurrentStage() {
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
uint8_t SystemService::MapCurrentStatusForSC() {
  uint8_t status = 0;

  switch (cur_status_) {

  case SYSTAT_WORK:
  case SYSTAT_RESUME_TRIG:
  case SYSTAT_RESUME_MOVING:
  case SYSTAT_RESUME_WAITING:
  case SYSTAT_PAUSE_TRIG:
    status = 3;
    break;

  case SYSTAT_PAUSE_STOPPED:
  case SYSTAT_PAUSE_FINISH:
    status = 4;
    break;

  default:
    status = 0;
    break;
  }

  return status;
}

/**
 * Get periph device status
 * return:periph device status
 */
uint8_t SystemService::SystemService::GetPeriphDeviceStatus()
{
  return PeriphDeviceStatus;
}


/**
 * SystemService:Get Faults
 */
uint32_t SystemService::GetSystemFault()
{
  return fault_flag_;
}

/**
 * SystemService:Get Faults
 */
void SystemService::ClearSystemFaultBit(uint32_t BitsToClear)
{
  fault_flag_ &= ~BitsToClear;
}

/**
 * SystemService:Set Faults
 */
void SystemService::SetSystemFaultBit(uint32_t BitsToSet)
{
  fault_flag_ |= BitsToSet;
}

/**
 * screen start a work
 */
ErrCode SystemService::StartWork(TriggerSource s) {

  if (action_ban & ACTION_BAN_NO_WORKING) {
    LOG_E("System Fault! Now cannot start working!\n");
    return E_NO_WORKING;
  }

  if (cur_status_ != SYSTAT_IDLE) {
    LOG_W("cannot start work in current status: %d\n", cur_status_);
    return E_NO_SWITCHING_STA;
  }

  switch (ModuleBase::toolhead()) {
  case MODULE_TOOLHEAD_3DP:
    if (runout.is_filament_runout()) {
      fault_flag_ |= FAULT_FLAG_FILAMENT;
      LOG_E("No filemant!\n");
      return E_NO_FILAMENT;
    }
    break;

  case MODULE_TOOLHEAD_LASER:
  case MODULE_TOOLHEAD_LASER_10W:
    is_laser_on = false;
    is_waiting_gcode = false;
  case MODULE_TOOLHEAD_CNC:
    if (enclosure.DoorOpened()) {
      fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
      LOG_E("Door is opened!\n");
      return E_DOOR_OPENED;
    }

    if (is_homing()) {
      LOG_E("Is homing!\n");
      return E_IS_HOMING;
    } else if (all_axes_homed() == false) {
      LOG_E("No homed!\n");
      return E_NO_HOMED;
    }

    // set to defualt power, but not turn on Motor
    if (MODULE_TOOLHEAD_CNC == ModuleBase::toolhead()) {
      cnc.power(100);
    }
    break;

  default:
    break;
  }

  pl_recovery.Reset();

  if (s == TRIGGER_SOURCE_SC) {
    pl_recovery.cur_data_.GCodeSource = GCODE_SOURCE_SCREEN;
    work_port_ = WORKING_PORT_SC;
  }
  else if (s == TRIGGER_SOURCE_PC) {
    pl_recovery.cur_data_.GCodeSource = GCODE_SOURCE_PC;
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
void SystemService::CheckException() {
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
ErrCode SystemService::ThrowException(ExceptionHost h, ExceptionType t) {
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
    if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP)
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
    // LOG_E("power-loss apeared at last poweroff!\n");
    break;

  case ETYPE_HEAT_FAIL:
    if (ModuleBase::toolhead() != MODULE_TOOLHEAD_3DP)
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
    if (ModuleBase::toolhead() != MODULE_TOOLHEAD_3DP)
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
    if (ModuleBase::toolhead() != MODULE_TOOLHEAD_3DP)
      return E_SUCCESS;
    LOG_E("Not handle exception: TEMP_REDUNDANCY\n");
    return E_FAILURE;
    break;

  case ETYPE_SENSOR_BAD:
    switch (h) {
    case EHOST_HOTEND0:
      if (ModuleBase::toolhead() != MODULE_TOOLHEAD_3DP)
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
    if (ModuleBase::toolhead() != MODULE_TOOLHEAD_3DP)
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
        action |= EACTION_STOP_WORKING;
      }
      LOG_E("current temp of hotend is higher than MAXTEMP: %d!\n", HEATER_0_MAXTEMP);
      break;

    case EHOST_BED:
      if (fault_flag_ & FAULT_FLAG_BED_MAXTEMP)
        return E_SAME_STATE;
      new_fault_flag = FAULT_FLAG_BED_MAXTEMP;
      action = EACTION_STOP_HEATING_BED;
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
    StopTrigger(TRIGGER_SOURCE_EXCEPTION, SSTP_INVALID_OP_CODE);
  }
  else if (action & EACTION_PAUSE_WORKING) {
    PauseTrigger(TRIGGER_SOURCE_EXCEPTION);
  }

  fault_flag_ |= new_fault_flag;
  SendException(fault_flag_);

  return E_SUCCESS;
}


ErrCode SystemService::ThrowExceptionISR(ExceptionHost h, ExceptionType t) {
  if (isr_e_len_ >= EXCEPTION_ISR_BUFFSER_SIZE)
    return E_NO_RESRC;

  isr_exception[isr_e_windex_][0] = h;
  isr_exception[isr_e_windex_][1] = t;

  if (++isr_e_windex_ >= EXCEPTION_ISR_BUFFSER_SIZE)
    isr_e_windex_ = 0;

  isr_e_len_++;

  return E_SUCCESS;
}


ErrCode SystemService::ClearException(ExceptionHost h, ExceptionType t) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
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


void SystemService::MapFaultFlagToException(uint32_t flag, ExceptionHost &host, ExceptionType &type) {
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

ErrCode SystemService::ClearExceptionByFaultFlag(uint32_t flag) {
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

void SystemService::CallbackOpenDoor() {
  if ((SYSTAT_WORK == cur_status_ ) ||(SYSTAT_RESUME_WAITING == cur_status_)) {
    PauseTrigger(TRIGGER_SOURCE_DOOR_OPEN);
  }

  fault_flag_ |= FAULT_FLAG_DOOR_OPENED;
}

void SystemService::CallbackCloseDoor() {
  fault_flag_ &= FAULT_FLAG_DOOR_OPENED;
}

ErrCode SystemService::CheckIfSendWaitEvent() {
  SSTP_Event_t event;
  ErrCode err;

  if (GetCurrentStatus() == SYSTAT_WORK) {
    if (hmi_gcode_pack_mode()) {
      check_and_request_gcode_again();
    }
    else if(!planner.movesplanned()) { // and no movement planned
      if ((hmi_cmd_timeout() + 1000) > millis()) {
        return E_SUCCESS;
      }
      hmi_cmd_timeout(millis());
      // and we have replied screen
      if (current_line() && current_line() == debug.GetSCGcodeLine()) {
        // then we known maybe screen lost out last reply
        LOG_I("waiting HMI command, current line: %u\n", current_line_);
        event.id = EID_SYS_CTRL_ACK;
        event.op_code = SYSCTL_OPC_WAIT_EVENT;
        event.data = &err;
        event.length = 1;
        err = 1;
        hmi.Send(event);
        if (!is_waiting_gcode) {
          is_waiting_gcode = true;
          if (laser->tim_pwm() > 0) {
            is_laser_on = true;
            laser->TurnOff();
          }
        }
      }
      else {
        is_waiting_gcode = false;
      }
    }
    else {
      is_waiting_gcode = false;
    }
  }

  return E_SUCCESS;
}


#if 0
ErrCode SystemService::SendStatus(SSTP_Event_t &event) {
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
  tmp_u32 = laser->power();
  hmi.ToPDUBytes((uint8_t *)&sta.laser_power, (uint8_t *)&tmp_u32, 2);

  // RPM of CNC
  tmp_u32 = cnc.rpm();
  hmi.ToPDUBytes((uint8_t *)&sta.cnc_rpm, (uint8_t *)&tmp_u32, 2);

  // system status
  sta.system_state = (uint8_t)systemservice.MapCurrentStatusForSC();

  // Add-On status
  sta.addon_state = (uint8_t)systemservice.GetPeriphDeviceStatus();

  // executor type
  sta.executor_type = ModuleBase::toolhead();

  return hmi.Send(event);
}
#else
ErrCode SystemService::SendStatus(SSTP_Event_t &event) {
  SystemStatus_t sta;

  int i = 0;
  uint8_t *buff = (uint8_t *)&sta;

  int32_t   tmp_i32;
  int16_t   tmp_i16;
  uint32_t  tmp_u32;
  float     tmp_f32;

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

  tmp_i32 = (int32_t) (current_position[E_AXIS] * 1000);
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

  if (ModuleBase::toolhead() == MACHINE_TYPE_LASER || ModuleBase::toolhead() == MACHINE_TYPE_LASER_10W) {
    // laser power
    tmp_u32 = (uint32_t)(laser->power() * 1000);
  } else if (ModuleBase::toolhead() == MACHINE_TYPE_CNC) {

    // RPM of CNC
    tmp_u32 = cnc.rpm();
  } else {
    // 3DPrint
    tmp_u32 = 0;
  }
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_u32, i);
  // B axis current logical position
  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[B_AXIS], B_AXIS) * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);

  // system status
  sta.system_state = (uint8_t)systemservice.MapCurrentStatusForSC();

  // Add-On status
  sta.addon_state = (uint8_t)systemservice.GetPeriphDeviceStatus();

  // executor type
  sta.executor_type = ModuleBase::toolhead();
  i += 3;
  if (cur_status_ > SYSTAT_IDLE) {
    tmp_u32 = pl_recovery.LastLine();
  } else {
    tmp_u32 = 0;
  }
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_u32, i);
  return hmi.Send(event);
}
#endif


ErrCode SystemService::SendException(SSTP_Event_t &event) {
  LOG_I("SC req Exception\n");

  return SendException(fault_flag_);
}


ErrCode SystemService::SendException(uint32_t fault) {
  SSTP_Event_t event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_GET_EXCEPTION};
  uint8_t buff[4];

  event.length = 4;
  event.data = buff;

  WORD_TO_PDU_BYTES(buff, fault);

  return hmi.Send(event);
}

ErrCode SystemService::SendSecurityStatus () {
  if (ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_10W) {
    laser->SendSecurityStatus();
  }

  return E_SUCCESS;
}

ErrCode SystemService::SendPause() {
  SSTP_Event_t event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_PAUSE};
  uint8_t buff[1] = {0};

  event.length = 1;
  event.data = buff;

  return hmi.Send(event);
}

ErrCode SystemService::ChangeSystemStatus(SSTP_Event_t &event) {
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
    if (xTaskGetCurrentTaskHandle() == sm2_handle->hmi) {
      LOG_I("SC req RESUME\n");
      err = ResumeTrigger(TRIGGER_SOURCE_SC);
      if (err == E_SUCCESS)
        need_ack = false;
    }
    else {
      err = ResumeProcess();
      if (err == E_SUCCESS) {
        pl_recovery.SaveCmdLine(pl_recovery.cur_data_.FilePosition);
        if (pl_recovery.cur_data_.FilePosition > 0) {
          current_line_ =  pl_recovery.cur_data_.FilePosition - 1;
        }
        else {
          current_line_ = 0;
        }
        gocde_pack_start_line(current_line_);
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
    LOG_I("SC req -> Success\n");
  }
  else {
    LOG_I("SC req -> Failed\n");
  }

  if (need_ack)
    return hmi.Send(event);
  else
    return err;
}


ErrCode SystemService::FinishSystemStatusChange(uint8_t op_code, uint8_t result) {
  SSTP_Event_t event = {EID_SYS_CTRL_ACK, op_code};

  event.data = &result;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode SystemService::SendLastLine(SSTP_Event_t &event) {
  uint8_t buff[6];

  if (GetCurrentStage() != SYSTAGE_PAUSE) {
    LOG_I("SC req last line: %u\n", pl_recovery.pre_data_.FilePosition);

    buff[0] = pl_recovery.pre_data_.Valid;
    buff[1] = pl_recovery.pre_data_.GCodeSource;
    WORD_TO_PDU_BYTES(buff + 2, pl_recovery.pre_data_.FilePosition);
  }
  else {
    LOG_I("SC req last line: %u\n", pl_recovery.cur_data_.FilePosition);

    buff[0] = pl_recovery.cur_data_.Valid;
    buff[1] = pl_recovery.cur_data_.GCodeSource;
    WORD_TO_PDU_BYTES(buff + 2, pl_recovery.cur_data_.FilePosition);
  }

  event.data = buff;
  event.length = 6;

  return hmi.Send(event);
}


ErrCode SystemService::ClearException(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  if (event.length == 0) {
    LOG_I("SC req clear power loss bits\n");
    systemservice.ClearExceptionByFaultFlag(FAULT_FLAG_POWER_LOSS);
    if (pl_recovery.pre_data_.Valid == 1) {
      // clear flash data
      LOG_I("mask flash data ...");
      pl_recovery.MaskPowerPanicData();
      pl_recovery.pre_data_.Valid = 0;
      LOG_I("Done!\n");
    }
  }
  else if (event.length == 4) {
    uint32_t bit_to_clear = 0;

    PDU_TO_LOCAL_WORD(bit_to_clear, event.data);
    LOG_I("SC req clear exception, fault bits: 0x%08X\n", bit_to_clear);

    bit_to_clear &= FAULT_FLAG_SC_CLEAR_MASK;
    systemservice.ClearExceptionByFaultFlag(bit_to_clear);
  }
  else {
    LOG_E("too many data: %d\n", event.length);
    err = E_FAILURE;
  }

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode SystemService::RecoverFromPowerLoss(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  LOG_I("SC trigger restore from power-loss\n");

  event.data = &err;
  event.length = 1;

  SysStatus cur_status = systemservice.GetCurrentStatus();

  if (cur_status != SYSTAT_IDLE) {
    LOG_E("cannot trigger recovery at current status: %d\n", cur_status);
    err = E_NO_SWITCHING_STA;
  }
  else {
    // screen bug: why will we receive two consecutive recovery command @TODO
    systemservice.SetCurrentStatus(SYSTAT_RESUME_TRIG);
    err = pl_recovery.ResumeWork();
    if (err == E_SUCCESS) {
      systemservice.SetCurrentStatus(SYSTAT_RESUME_WAITING);
      systemservice.SetWorkingPort(WORKING_PORT_SC);
      pl_recovery.cur_data_.FilePosition = pl_recovery.pre_data_.FilePosition;
      if (pl_recovery.cur_data_.FilePosition > 0)
        current_line_ =  pl_recovery.cur_data_.FilePosition - 1;
      else
        current_line_ = 0;
      // Batch sending requires the controller to actively request the next line
      gocde_pack_start_line(current_line_);
      SNAP_DEBUG_SET_GCODE_LINE(current_line_);
      pl_recovery.SaveCmdLine(pl_recovery.cur_data_.FilePosition);
      LOG_I("trigger RESTORE: ok\n");
    }
    else {
      LOG_I("trigger RESTORE: failed, err = %d\n", err);
      systemservice.SetCurrentStatus(cur_status);
    }
  }

  return hmi.Send(event);
}


ErrCode SystemService::SendHomeAndCoordinateStatus(SSTP_Event_t &event) {
  uint8_t buff[20] = {0};
  int32_t pos_shift[XN];

  int i = 0;

  event.data = buff;

  if (is_homing()) {
    buff[i++] = 2;
  } else if (all_axes_homed()) {
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
    pos_shift[B_AXIS] = (int32_t)(position_shift[B_AXIS] * 1000);
  }
  else {
    buff[i++] = gcode.active_coordinate_system + 1;
    // check state
    if ((position_shift[X_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS]) &&
        (position_shift[Y_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS]) &&
        (position_shift[Z_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS]) &&
        (position_shift[B_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][B_AXIS])) {
      buff[i++] = 0;
    }
    else {
      buff[i++] = 1;
    }
    pos_shift[X_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS] * 1000);
    pos_shift[Y_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS] * 1000);
    pos_shift[Z_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS] * 1000);
    pos_shift[B_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][B_AXIS] * 1000);
  }

  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, pos_shift[X_AXIS], i);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, pos_shift[Y_AXIS], i);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, pos_shift[Z_AXIS], i);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, pos_shift[B_AXIS], i);

  event.length = i;

  return hmi.Send(event);
}


ErrCode SystemService::ChangeRuntimeEnv(SSTP_Event_t &event) {
  ErrCode ret = E_SUCCESS;

  float param;

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
    if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead()) {
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
    if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead()) {
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
    if ((MODULE_TOOLHEAD_LASER != ModuleBase::toolhead()) && (MODULE_TOOLHEAD_LASER_10W != ModuleBase::toolhead())) {
      ret = E_INVALID_STATE;
      break;
    }

    if (param > 100 || param < 0)
      ret = E_PARAM;
    else {
      if (laser->tim_pwm() > 0)
        laser->SetOutput(param);
      else
        laser->SetPower(param);
    }
    break;

  case RENV_TYPE_ZOFFSET:
    ret = levelservice.UpdateLiveZOffset(param);
    break;

  case RENV_TYPE_CNC_POWER:
    if (MODULE_TOOLHEAD_CNC != ModuleBase::toolhead()) {
      ret = E_INVALID_STATE;
      LOG_E("Not CNC toolhead!\n");
      break;
    }

    if (param > 100 || param < 0) {
      ret = E_PARAM;
      LOG_E("out of range: %.2f\n", param);
    }
    else {
      cnc.power(param);
      // change current output when it was turned on
      if (cnc.rpm() > 0)
        cnc.TurnOn();
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


ErrCode SystemService::GetRuntimeEnv(SSTP_Event_t &event) {
  int   tmp_i32;
  uint8_t buff[5];

  LOG_I("SC get env: %u\n", event.data[0]);

  buff[0] = E_SUCCESS;

  switch (event.data[0]) {
  case RENV_TYPE_FEEDRATE:
    tmp_i32 = (int)(pl_recovery.pre_data_.feedrate_percentage * 1000.0f);
    WORD_TO_PDU_BYTES(buff+1, (int)tmp_i32);
    LOG_I("feedrate_percentage: %d\n", pl_recovery.pre_data_.feedrate_percentage);
    break;

  case RENV_TYPE_LASER_POWER:
    tmp_i32 = (int)(pl_recovery.pre_data_.laser_percent * 1000.0f);
    WORD_TO_PDU_BYTES(buff+1, (int)tmp_i32);
    LOG_I("laser power: %.2f\n", pl_recovery.pre_data_.laser_percent);
    break;

  case RENV_TYPE_ZOFFSET:
    tmp_i32 = (int)(levelservice.live_z_offset() * 1000);
    WORD_TO_PDU_BYTES(buff+1, tmp_i32);
    LOG_I("live z offset: %.3f\n", levelservice.live_z_offset());
    break;

  case RENV_TYPE_CNC_POWER:
    tmp_i32 = (int)(pl_recovery.pre_data_.cnc_power * 1000);
    WORD_TO_PDU_BYTES(buff+1, tmp_i32);
    LOG_I("laser power: %.2f\n", pl_recovery.pre_data_.cnc_power);
    break;

  default:
    buff[0] = E_FAILURE;
    LOG_I("cannot get this env: %u\n", event.data[0]);
    break;
  }

  event.data = buff;
  event.length = 5;

  return hmi.Send(event);
}


ErrCode SystemService::GetMachineSize(SSTP_Event_t &event) {
  uint8_t buffer[64];
  uint16_t i = 0;

  int32_t  tmp_i32;
  uint32_t tmp_u32;


  buffer[i++] = 0;

  //Machine size type
  buffer[i++] = linear_p->machine_size();

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


ErrCode SystemService::CallbackPreQS(QuickStopSource source) {
  switch (source) {
  case QS_SOURCE_PAUSE:

    print_job_timer.pause();

    // reset the status of filament monitor
    runout.reset();
    // make sure laser is off
    if ((ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER) || (ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_10W)) {
      laser->TurnOff();
    }
    break;

  case QS_SOURCE_STOP:
    PreProcessStop();
    break;

  case QS_SOURCE_SECURITY:
    PreProcessStop();
    SendSecurityStatus();
    break;

  default:
    break;
  }

  return E_SUCCESS;
}


ErrCode SystemService::CallbackPostQS(QuickStopSource source) {
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

  case QS_SOURCE_SECURITY:
    FinishSystemStatusChange(SYSCTL_OPC_PAUSE, 0);
    pause_source_ = TRIGGER_SOURCE_NONE;

    LOG_I("Finish pause\n\n");
    cur_status_ = SYSTAT_PAUSE_FINISH;
    // notify HMI
    // SendPause();

    LOG_I("Finish handle protection\n");
    break;

  default:
    break;
  }

  return E_SUCCESS;
}

