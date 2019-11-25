
#include "quickstop.h"
#include "../module/PeriphDevice.h"
#include "../module/PowerPanic.h"
#include "../module/stepper.h"
#include "../module/temperature.h"
#include "../module/printcounter.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../module/StatusControl.h"
#include "snap_dbg.h"

QuickStop quickstop;

/*
 * check if event of quick stop happened
 * if need to stop stepper output, return true
 * otherwise return false
 */
bool QuickStop::CheckISR(block_t *blk) {
  QuickStopEvent new_event = QS_EVENT_NONE;
  static millis_t last_powerloss = 0;

  uint8_t powerstat = READ(POWER_DETECT_PIN);

  // debounce for power loss, will delay 10ms for responce
  if (powerstat != POWER_LOSS_STATE)
    last_powerloss = millis();
  else {
    if ((millis() - last_powerloss) < POWERPANIC_DEBOUNCE)
      powerstat = POWER_NORMAL_STATE;
  }

  // here may have 4 conditions:
  // 1. no event happened and no power-loss for now:
  //      just return
  // 2. no event happened but power-loss happen just now:
  //      perform power-loss procedures
  // 3. one event happened and no power-loss for now:
  //      see if we need to abort current block
  // 4. one event happened and detected power-loss:
  //      need to see if previous event is power-loss.
  //      if yes: judge if need to abort current block
  //      if no: turn off power and write flash
  if (event_ == QS_EVENT_NONE) {
    if (powerstat != POWER_LOSS_STATE) {
      // power loss doesn't appear
      return false;
    }
    else {
      // power loss happened
      event_ = QS_EVENT_ISR_POWER_LOSS;
      new_event = QS_EVENT_ISR_POWER_LOSS;
      // delay movement to be planned when power loss
      planner.cleaning_buffer_counter = 1000;
    }
  }
  else {
    if (powerstat == POWER_LOSS_STATE) {
      if (event_ != QS_EVENT_ISR_POWER_LOSS) {
        // arive here, event_ should be QS_EVENT_PAUSE or QS_EVENT_STOP
        // we change event_ to QS_EVENT_ISR_POWER_LOSS, then will not arive here again next ISR
        // NOTE: at the moment, sync_flag_ maybe QS_SYNC_TRIGGER
        event_ = QS_EVENT_ISR_POWER_LOSS;
        new_event = QS_EVENT_ISR_POWER_LOSS;
      }
    }
    else if (sync_flag_ == QS_SYNC_TRIGGER) {
      // arive here, power-loss doesn't appear, but other
      // event trigger the quickstop
      new_event = event_;
    }
  }

  // here are the common handle for above branchs
  // which has no new event
  if (new_event == QS_EVENT_NONE) {
    if (disable_stepper_)
      return true;
    else
      return false;
  }

  if (new_event == QS_EVENT_ISR_POWER_LOSS)
    powerpanic.TurnOffPowerISR();

  // we save env only for two conditions:
  // 1. non-powerloss triggered quickstop excpet STOP
  if ((sync_flag_ == QS_SYNC_TRIGGER && event_ != QS_EVENT_STOP) ||
      (new_event == QS_EVENT_ISR_POWER_LOSS && SystemStatus.GetCurrentStatus() == SYSTAT_WORK)) {
    if (blk)
      powerpanic.SaveCmdLine(blk->filePos);
    set_current_from_steppers_for_axis(ALL_AXES);
    powerpanic.SaveEnv();
  }

  if (new_event == QS_EVENT_ISR_POWER_LOSS) {
    if (SystemStatus.GetCurrentStage() == SYSTAGE_WORK ||
        SystemStatus.GetCurrentStage() == SYSTAGE_PAUSE) {
      powerpanic.WriteFlash();
    }
  }

  // so whether event source is power-loss or not, we change sync_flag_ to QS_SYNC_ISR_END
  // then non-ISR env is able to know if we have done ISR
  sync_flag_ = QS_SYNC_ISR_END;

  // disable_stepper_ to be true, CPU will not run to here until next new event happen
  disable_stepper_ = true;

  return true;
}

ErrCode QuickStop::Trigger(QuickStopEvent e) {
  ErrCode ret = E_SUCCESS;

  // only stepper ISR may read/write event_ at the same time
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  if (event_ != QS_EVENT_NONE) {
    ret = E_BUSY;
    LOG_W("pre event[%d] is not none\n", event_);
  }
  else {
    event_ = e;
    sync_flag_ = QS_SYNC_TRIGGER;
    // delay movement to be planned when quick stop is triggered
    planner.cleaning_buffer_counter = 1000;
  }
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  clear_command_queue();

  return ret;
}

void QuickStop::CleanMoves() {
  millis_t timeout = millis() + 1000UL;

  clear_command_queue();

  // make sure stepper ISR is enabled
  // need it to save data
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  // waiting sync_flag_ to become QS_SYNC_ISR_END
  while (sync_flag_ != QS_SYNC_ISR_END) {
    if ((int32_t)(timeout - millis()) < 0) {
      timeout = millis() + 1000UL;
      if (event_ != QS_EVENT_ISR_POWER_LOSS)
        LOG_I("wait sync flag timeout\n");
    }
  }

  // stepper will clean blocks safely, we had to wait buffer to be empty
  while (planner.has_blocks_queued()) {
    planner.block_buffer_nonbusy = planner.block_buffer_tail = \
      planner.block_buffer_planned = planner.block_buffer_head;
  }

  // make it false, will not abort block, then we can output moves
  disable_stepper_ = false;

  // these two variables will clean the latency in planning commands
  // and outputing blocks
  planner.delay_before_delivering = 0;
  planner.cleaning_buffer_counter = 0;
}

void QuickStop::TowardStop() {
  bool leveling_active = planner.leveling_active;
  float retract = 0;
  // make sure we are in absolute position mode
  relative_mode = false;

  set_current_from_steppers_for_axis(ALL_AXES);
  sync_plan_position();

  // we need to move to Z max
  if (leveling_active)
    set_bed_leveling_enabled(false);

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    if(thermalManager.temp_hotend[0].current > 180)
      retract = 3;

    // for power loss, we don't have enough time
    if (event_ == QS_EVENT_ISR_POWER_LOSS) {
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
    if (event_ == QS_EVENT_STOP) {
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
    if (event_ != QS_EVENT_ISR_POWER_LOSS)
      idle();
  }

  if (leveling_active)
    set_bed_leveling_enabled(true);
}


void QuickStop::Process() {
  if (event_ == QS_EVENT_NONE)
    return;

  if (event_ == QS_EVENT_ISR_POWER_LOSS) {
    powerpanic.TurnOffPower();
  }

  CleanMoves();

  if (event_ != QS_EVENT_ISR_POWER_LOSS) {
    LOG_I("\nProcess: start ponit\n");
    LOG_I("X: %.2f, Y:%.2f, Z:%.2f, E: %.2f\n", current_position[0],
          current_position[1], current_position[2], current_position[3]);
  }

  // (power-loss && working) or (not power-loss)
  if ((event_ != QS_EVENT_ISR_POWER_LOSS) ||
      (SystemStatus.GetCurrentStatus() == SYSTAT_WORK))
    TowardStop();

  if (ExecuterHead.MachineType == MACHINE_TYPE_CNC)
    ExecuterHead.CNC.SetPower(0);

  if (event_ == QS_EVENT_ISR_POWER_LOSS)
    while (1);

  if (SystemStatus.GetCurrentStatus() == SYSTAT_PAUSE_TRIG)
    SystemStatus.SetCurrentStatus(SYSTAT_PAUSE_STOPPED);

  if (SystemStatus.GetCurrentStatus() == SYSTAT_END_TRIG)
    SystemStatus.SetCurrentStatus(SYSTAT_END_FINISH);

  // we have stopped, so clear previous event
  event_ = QS_EVENT_NONE;
  sync_flag_ = QS_SYNC_NONE;
}

void QuickStop::Reset() {
}
