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
#include "quick_stop.h"

#include "power_loss_recovery.h"
#include "system.h"

#include "../module/toolhead_cnc.h"
#include "../module/toolhead_3dp.h"
#include "../module/toolhead_laser.h"
#include "../module/enclosure.h"

#include "src/module/stepper.h"
#include "src/module/temperature.h"
#include "src/module/printcounter.h"
#include "src/feature/bedlevel/bedlevel.h"
#include <src/gcode/gcode.h>
#include HAL_PATH(src/HAL, HAL_watchdog_STM32F1.h)

#define CNC_SAFE_HIGH_DIFF 30  // Bed to CNC head height. mm


QuickStopService quickstop;


void QuickStopService::Init() {

}


void QuickStopService::Trigger(QuickStopSource new_source, bool from_isr /*=false*/) {

  // power-loss will be check in temperature time isr
  // if we are not in working, won't handle power-loss
  if (from_isr) {
    if (systemservice.GetCurrentStage() != SYSTAGE_WORK && systemservice.GetCurrentStage() != SYSTAGE_PAUSE)
      return;
  }
  else {
    taskENTER_CRITICAL();
    DISABLE_TEMPERATURE_INTERRUPT();
  }
  homing_is_interrupted_ = is_homing();
  if (new_source == QS_SOURCE_STOP_BUTTON) {
    source_ = new_source;
    state_ = QS_STA_STOPPED;
    // to stop planning movement
    planner.cleaning_buffer_counter = 0;
  } else if (source_ == QS_SOURCE_IDLE) {
    source_ = new_source;
    state_ = QS_STA_TRIGGERED;

    // to stop planning movement
    planner.cleaning_buffer_counter = 100;
  } else {
    if (new_source == QS_SOURCE_POWER_LOSS) {
      pre_source_ = source_;
      source_ = new_source;
    }
  }

  if (!from_isr) {
    taskEXIT_CRITICAL();
    ENABLE_TEMPERATURE_INTERRUPT();
  }
}


/*
 * will be called by stepper ISR
 * return:
 *    true  - stop outputing steps
 *    false - outputint steps
 */
bool QuickStopService::CheckInISR(block_t *blk) {

  bool ret = false;

  switch (state_) {
  /*
   * normal state
   */
  case QS_STA_IDLE:
    break;

  /*
   * triggered by some one
   */
  case QS_STA_TRIGGERED:
    // need sync count position from stepper to planner
    // otherwise, it may park in unexpected position
    current_position[E_AXIS] = planner.get_axis_position_mm(E_AXIS);
    set_current_from_steppers_for_axis(ALL_AXES);

    switch (source_) {
    /*
    * triggered by power-loss, turn off some power domains
    * before save env and write flash, will run through
    * next case to save env and write flash
    */
    case QS_SOURCE_POWER_LOSS:
      TurnOffPower();

    case QS_SOURCE_SECURITY:
      HandleProtection();

    /*
    * triggered by PAUSE, just save env and switch to next state
    */
    case QS_SOURCE_PAUSE:
      if (blk) {
        pl_recovery.SaveCmdLine(blk->filePos);
        // pl_recovery.SaveLaserInlineState(blk->laser.status.isEnabled, blk->laser.status.trapezoid_power);
        pl_recovery.SaveLaserPowerInfo(blk->laser);
      }
      else {
        block_inline_laser_t info;
        info.status = planner.laser_inline.status;
        info.power = planner.laser_inline.power;
        info.sync_power = planner.laser_inline.sync_power;
        pl_recovery.SaveLaserPowerInfo(info);
        // pl_recovery.SaveLaserInlineState(planner.laser_inline.status.isEnabled, planner.laser_inline.status.trapezoid_power);
      }

      // if power-loss appear atfer finishing PAUSE, won't save env again
      if (systemservice.GetCurrentStatus() != SYSTAT_PAUSE_FINISH)
        pl_recovery.SaveEnv();

      // write flash only power-loss appear
      if (source_ == QS_SOURCE_POWER_LOSS) {
        pl_recovery.WriteFlash();
        wrote_flash_ = true;
      }
      break;

    default:
      break;
    }

    /* to make sure block buffer is cleaned before QS_STA_STOPPED / QS_STA_SAVED_ENV
     * we add a middle state QS_STA_CLEAN_MOVES
     * */
    state_ = QS_STA_CLEAN_MOVES;
    ret = true;
    break;

  case QS_STA_CLEAN_MOVES:
    switch(source_) {
    /*
    * triggered by STOP, save nothing, switch to next state directly
    */
    case QS_SOURCE_STOP:
      state_ = QS_STA_STOPPED;
      break;

    case QS_SOURCE_POWER_LOSS:
    case QS_SOURCE_SECURITY:
    case QS_SOURCE_PAUSE:
      state_ = QS_STA_SAVED_ENV;
      break;

    default:
      break;
    }
    ret = true;
    break;

  /*
   * waiting state to be changed to QS_STA_PARKING in Process()
   */
  case QS_STA_STOPPED:
    ret = true;
    break;

  /*
   * triggered by pause/power-loss, and env has been saved.
   * if power loss, env has been written into flash, here
   * won't break, just run to case QS_STA_PARKING, to check if
   * power-loss happen in PAUSE
   */
  case QS_STA_SAVED_ENV:
    ret = true;

  /*
   * ok, Process() is called, the state has been changed to QS_STA_PARKING
   * if power loss happened when parking for PAUSE, we need to write env into flash
   */
  case QS_STA_PARKING:
    if ((source_ == QS_SOURCE_POWER_LOSS) && ((pre_source_ == QS_SOURCE_PAUSE) || (pre_source_ == QS_SOURCE_SECURITY)) && !wrote_flash_) {
      TurnOffPower();
      pl_recovery.WriteFlash();
      wrote_flash_ = true;
    }
    break;

  default:
    break;
  }

  return ret;
}


void QuickStopService::Park() {
  bool leveling_active = planner.leveling_active;
  float retract = 0;
  // make sure we are in absolute position mode
  relative_mode = false;

  // we need to move to Z max
  if (leveling_active)
    set_bed_leveling_enabled(false);

  switch (ModuleBase::toolhead()) {
  case MODULE_TOOLHEAD_3DP:
  case MODULE_TOOLHEAD_DUALEXTRUDER:
    // PREVENT_COLD_EXTRUSION protection is enabled without further temperature determination
    // if(thermalManager.temp_hotend[0].current > 180) {
      if (ModuleBase::toolhead() ==  MODULE_TOOLHEAD_DUALEXTRUDER)
        retract = DUAL_EXTRUDER_RESUME_RETRACT_E_LENGTH;
      else
        retract = SINGLE_RESUME_RETRACT_E_LENGTH;
    // }

    // for power loss, we don't have enough time
    if (source_ == QS_SOURCE_POWER_LOSS) {
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_DUALEXTRUDER)
        current_position[E_AXIS] -= DUAL_EXTRUDER_POWER_LOSS_RETRACT_E_LENGTH;
      else
        current_position[E_AXIS] -= SINGLE_POWER_LOSS_RETRACT_E_LENGTH;

      line_to_current_position(60);
      Z_enable;
      move_to_limited_ze(current_position[Z_AXIS] + 5, current_position[E_AXIS] - retract + 1, 20);
    }
    else {
      current_position[E_AXIS] -= retract;
      line_to_current_position(60);
      // if we are not in power loss, retrace E quickly
      move_to_limited_z(Z_MAX_POS, 20);
    }

    // move X to max position of home dir
    // move Y to max position
    if (X_HOME_DIR > 0)
      move_to_limited_xy(soft_endstop[X_AXIS].max, Y_MAX_POS, 60);
    else
      move_to_limited_xy(soft_endstop[X_AXIS].min, Y_MAX_POS, 60);
    break;

  case MODULE_TOOLHEAD_LASER:
  case MODULE_TOOLHEAD_LASER_10W:
  case MODULE_TOOLHEAD_LASER_20W:
  case MODULE_TOOLHEAD_LASER_40W:
    // In the case of laser, we don't raise Z.
    if (source_ == QS_SOURCE_STOP || laser->security_status_) {
      move_to_limited_z(Z_MAX_POS, 30);
    }
    laser->SetAirPumpSwitch(false, false);
    break;

  case MODULE_TOOLHEAD_CNC:
    if (source_ != QS_SOURCE_POWER_LOSS) {
      if (current_position[Z_AXIS] + CNC_SAFE_HIGH_DIFF < Z_MAX_POS) {
        move_to_limited_z(current_position[Z_AXIS] + CNC_SAFE_HIGH_DIFF, 30);
        while (planner.has_blocks_queued()) {
          idle();
        }
        cnc.SetOutput(0);
      }

      move_to_limited_z(Z_MAX_POS, 30);
      while (planner.has_blocks_queued()) {
          idle();
      }
      cnc.SetOutput(0);
    }
    break;

  default:
    break;
  }

  while (planner.has_blocks_queued()) {
    idle();
  }

  if (leveling_active)
    set_bed_leveling_enabled(true);
}


/*
 * disable other unused peripherals in ISR
 */
void QuickStopService::TurnOffPower() {
  disable_power_domain(POWER_DOMAIN_0 | POWER_DOMAIN_2);

  X_disable;
  Y_disable;
  Z_disable;

  BreathLightClose();

  if ((ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER) ||
      (ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_10W) ||
      (ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_20W) ||
      (ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_40W)
  ) {
    // The laser movement stops immediately with no risk of damaging the model
    disable_power_domain(POWER_DOMAIN_1);
  }
}

void QuickStopService::HandleProtection() {
  if (source_ != QS_SOURCE_SECURITY) return;

  if (ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_10W ||
      ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_20W ||
      ModuleBase::toolhead() == MODULE_TOOLHEAD_LASER_40W
  ) {
      // don't do nothing
  }
}

void QuickStopService::EmergencyStop() {
  // module stop after module version 1.9.1
  canhost.SendEmergencyStop();
  // compatible old module
  switch (ModuleBase::toolhead()) {
    case MACHINE_TYPE_CNC:
      cnc.TurnOff();
      break;
    case MACHINE_TYPE_DUALEXTRUDER:
      printer1->SetFan(1, 0);
    case MACHINE_TYPE_3DPRINT:
      printer1->SetFan(0, 0);
      // printer1->SetFan(2, 0);
      break;
    case MACHINE_TYPE_LASER:
    case MACHINE_TYPE_LASER_10W:
    case MACHINE_TYPE_LASER_20W:
    case MACHINE_TYPE_LASER_40W:
      laser->SetCameraLight(0);
      laser->SetFanPower(0);
      break;
    case MACHINE_TYPE_UNDEFINE:
      break;
  }
  enclosure.SetFanSpeed(0);
  enclosure.SetLightBar(0);
}

void QuickStopService::Process() {
  if (state_ == QS_STA_IDLE)
    return;

  // Waiting state_ to run over QS_STA_CLEAN_MOVES to make sure
  // env has been saved and current_block in stepper was clean
  while (state_ <= QS_STA_CLEAN_MOVES) {
    idle();
  }

  // tell system manager we start to handle QS in Non-ISR
  systemservice.CallbackPreQS(source_);

  // clean the counter to recover planner
  // tmeperature ISR maybe subtract the counter
  // so disable the interrupt before changing it
  DISABLE_TEMPERATURE_INTERRUPT();
  planner.cleaning_buffer_counter = 0;
  ENABLE_TEMPERATURE_INTERRUPT();

  // restore current position from stepper again,
  // because current position maybe changed between
  // disabling stepper output and reach this function
  set_current_from_steppers_for_axis(ALL_AXES);
  sync_plan_position();

  // switch to QS_STA_PARKING, to recover stepper output
  state_ = QS_STA_PARKING;

  if (source_ != QS_SOURCE_POWER_LOSS) {
    // logical position
    LOG_I("QS recorded pos X: %.3f, Y: %.3f, Z: %.3f, E: %.3f\n", pl_recovery.cur_data_.PositionData[X_AXIS],
        pl_recovery.cur_data_.PositionData[Y_AXIS], pl_recovery.cur_data_.PositionData[Z_AXIS], pl_recovery.cur_data_.PositionData[E_AXIS]);
    LOG_I("QS at logical pos: X: %.3f, Y: %.3f, Z: %.3f, E: %.3f\n", LOGICAL_X_POSITION(current_position[X_AXIS]),
        LOGICAL_Y_POSITION(current_position[Y_AXIS]), LOGICAL_Z_POSITION(current_position[Z_AXIS]), current_position[E_AXIS]);
  }

  if (source_ == QS_SOURCE_STOP_BUTTON) {
    LOG_I("Trigger emergency stop!\n");
    EmergencyStop();
  } else {
    // parking
    if (homing_is_interrupted_) {
      process_cmd_imd("G28");
    }
    Park();
  }

  if (source_ == QS_SOURCE_POWER_LOSS) {
    // for normal power-loss, CPU cannot arrive here
    // but for exception, this may be performed, so we output log
    // and reboot the machine to make it be able to recover from power-loss
    LOG_I("saved power-loss!\n");
    // reboot machine
    WatchDogInit();
    while (1);
  }

  // idle() will get new command during parking, clean again
  clear_command_queue();
  clear_hmi_gcode_queue();

  // tell system controller we have parked
  systemservice.CallbackPostQS(source_);
  homing_is_interrupted_ = false;
  state_ = QS_STA_IDLE;
  source_ = QS_SOURCE_IDLE;
  pre_source_ = QS_SOURCE_IDLE;
  wrote_flash_ = false;
}
