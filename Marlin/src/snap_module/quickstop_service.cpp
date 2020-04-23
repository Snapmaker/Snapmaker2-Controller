#include "quickstop_service.h"
#include "../module/PowerPanic.h"
#include "../module/stepper.h"
#include "../module/temperature.h"
#include "../module/printcounter.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../module/StatusControl.h"

QuickStopService quickstop;

void QuickStopService::Init() {

}


void QuickStopService::Trigger(QuickStopSource new_source, bool from_isr /*=false*/) {
  // power-loss will be check in temperature isr
  if (!from_isr) {
    taskENTER_CRITICAL();
    DISABLE_TEMPERATURE_INTERRUPT();
  }

  if (source_ == QS_SOURCE_IDLE) {
    source_ = new_source;
    state_ = QS_STA_TRIGGERED;

    // to stop planning movement
    planner.cleaning_buffer_counter = 2000;
  }
  else {
    if (new_source == QS_SOURCE_POWER_LOSS) {
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

  switch (state_) {
  /*
   * normal state
   */
  case QS_STA_IDLE:
    return false;

  /*
   * triggered by some one
   */
  case QS_STA_TRIGGERED:

    switch (source_) {
    case QS_SOURCE_STOP:
      state_ = QS_STA_STOPPED;
      break;

    case QS_SOURCE_POWER_LOSS:
      powerpanic.TurnOffPower(state_);

    case QS_SOURCE_PAUSE:
      if (blk)
        powerpanic.SaveCmdLine(blk->filePos);
      set_current_from_steppers_for_axis(ALL_AXES);
      powerpanic.SaveEnv();

      if (source_ == QS_SOURCE_POWER_LOSS) {
        powerpanic.WriteFlash();
        wrote_flash_ = true;
        state_ = QS_STA_WROTE_FLASH;
      }
      else {
        state_ = QS_STA_SAVED_ENV;
      }
      break;

    default:
      break;
    }

    return true;

  /*
   * triggered by pause, and env has been saved.
   * if power loss, just write env into flash,
   * or the state_ will be changed to QS_STA_PARKING
   */
  case QS_STA_SAVED_ENV:
    if (source_ == QS_SOURCE_POWER_LOSS) {
      powerpanic.TurnOffPower(state_);
      powerpanic.WriteFlash();
      wrote_flash_ = true;
      state_ = QS_STA_WROTE_FLASH;
    }
    break;

  /*
   * waiting state to be changed to QS_STA_PARKING
   */
  case QS_STA_STOPPED:
  case QS_STA_WROTE_FLASH:
    return true;

  /*
   * ok, Process() is called, the state has been changed to QS_STA_PARKING
   * and if power loss during parking, will save env to flash if it was not be saved
   */
  case QS_STA_PARKING:
    if (!wrote_flash_) {
      powerpanic.TurnOffPower(state_);
      powerpanic.WriteFlash();
      wrote_flash_ = true;
    }
    return false;

  default:
    return true;
  }
}


void QuickStopService::Park() {
  bool leveling_active = planner.leveling_active;
  float retract = 0;
  // make sure we are in absolute position mode
  relative_mode = false;

  // we need to move to Z max
  if (leveling_active)
    set_bed_leveling_enabled(false);

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    if(thermalManager.temp_hotend[0].current > 180)
      retract = 3;

    // for power loss, we don't have enough time
    if (source_ == QS_SOURCE_POWER_LOSS) {
      current_position[E_AXIS] -= 2;
      line_to_current_position(60);
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
      move_to_limited_xy(X_MAX_POS, Y_MAX_POS, 30);
    else
      move_to_limited_xy(0, Y_MAX_POS, 35);
    break;

  case MACHINE_TYPE_LASER:
    // In the case of laser, we don't raise Z.
    if (source_ == QS_SOURCE_STOP) {
      move_to_limited_z(Z_MAX_POS, 20);
    }
    break;

  case MACHINE_TYPE_CNC:
    move_to_limited_z(Z_MAX_POS, 20);
    break;

  default:
    break;
  }

  while (planner.has_blocks_queued()) {
    if (source_ != QS_SOURCE_POWER_LOSS)
      idle();
  }

  if (leveling_active)
    set_bed_leveling_enabled(true);
}


void QuickStopService::Process() {
  if (state_ == QS_STA_IDLE)
    return;

  // pending the HMI task
  vTaskSuspend(snap_tasks->hmi);
  xMessageBufferReset(snap_tasks->event_queue);
  clear_command_queue();

  // waiting for the block queue to be clear by stepper ISR
  while (planner.has_blocks_queued()) {
    idle();
  }

  planner.cleaning_buffer_counter = 0;

  state_ = QS_STA_PARKING;

  Park();

  if (source_ == QS_SOURCE_POWER_LOSS)
    while (1);

  vTaskResume(snap_tasks->hmi);

  state_ = QS_STA_IDLE;
  source_ = QS_SOURCE_IDLE;
  wrote_flash_ = false;
}


ErrCode QuickStopService::RegisterCB(QuickStopSource s, QSCallback_t cb, QuickStopCallbackType t) {
  if (s >= QS_SOURCE_INVALID || !cb)
    return E_PARAM;

  if (t == QS_CB_TYPE_PRE) {
    if (cb_pre_[s]) {
      LOG_E("already register callback for source[%d]\n", s);
      return E_NO_RESRC;
    }

    cb_pre_[s] = cb;
  }
  else {
    if (cb_post_[s]) {
      LOG_E("already register callback for source[%d]\n", s);
      return E_NO_RESRC;
    }

    cb_post_[s] = cb;
  }

  return E_SUCCESS;
}

