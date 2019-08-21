
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

void QuickStop::CheckISR(block_t *blk) {
  QuickStopEvent new_event = QS_EVENT_NONE;
  static millis_t last_powerloss = 0;

  uint8_t powerstat = (debug_ == QS_EVENT_NONE)?  READ(POWER_DETECT_PIN) : POWER_LOSS_STATE;

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
      return;
    }
    else {
      // power loss happened
      event_ = QS_EVENT_ISR_POWER_LOSS;
      new_event = QS_EVENT_ISR_POWER_LOSS;
    }
  }
  else {
    if (powerstat == POWER_LOSS_STATE) {
      if (event_ != QS_EVENT_ISR_POWER_LOSS) {
        event_ = QS_EVENT_ISR_POWER_LOSS;
        new_event = QS_EVENT_ISR_POWER_LOSS;
      }
    }
    else if (sync_flag_ == QS_SYNC_TRIGGER) {
      new_event = event_;
    }
  }

  // here are the common handle for above branchs
  // which has no new event
  if (new_event == QS_EVENT_NONE) {
    if (disable_stepper_)
      stepper.quick_stop();
    return;
  }

  if (new_event == QS_EVENT_ISR_POWER_LOSS)
    powerpanic.TurnOffPowerISR();


  if ((sync_flag_ == QS_SYNC_TRIGGER) ||
      (new_event == QS_EVENT_ISR_POWER_LOSS && sync_flag_ != QS_SYNC_ISR_END)) {
    if (blk)
      powerpanic.SaveCmdLine(blk->filePos);
    set_current_from_steppers_for_axis(ALL_AXES);
    powerpanic.SaveEnv();
  }

  if (new_event == QS_EVENT_ISR_POWER_LOSS)
    powerpanic.WriteFlash();

  sync_flag_ = QS_SYNC_ISR_END;

  stepper.quick_stop();

  disable_stepper_ = true;
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
  }
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  return ret;
}

void QuickStop::CleanMoves() {
  millis_t timeout = millis() + 1000UL;

  clear_command_queue();

  DISABLE_STEPPER_DRIVER_INTERRUPT();

  planner.block_buffer_nonbusy = planner.block_buffer_planned = \
      planner.block_buffer_head = planner.block_buffer_tail;

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

  while (planner.movesplanned()) {
    if ((int32_t)(timeout - millis()) < 0) {
      timeout = millis() + 1000UL;
      if (event_ != QS_EVENT_ISR_POWER_LOSS)
        LOG_I("wait moves empty timeout\n");
    }
  }

  while (stepper.get_current_block()) {
    if ((int32_t)(timeout - millis()) < 0) {
      timeout = millis() + 1000UL;
      if (event_ != QS_EVENT_ISR_POWER_LOSS)
        LOG_I("wait block to NULL timeout!\n");
    }
  }

  // make it false, will not abort block, then we can output moves
  disable_stepper_ = false;

  // maker sure 'abort_current_block' is false
  stepper.allow_current_block();

  // these two variables will clean the latency in planning commands
  // and outputing blocks
  planner.delay_before_delivering = 0;
  planner.cleaning_buffer_counter = 0;
}

void QuickStop::TowardStop() {
  // make sure we are in absolute position mode
  relative_mode = false;

  set_current_from_steppers_for_axis(ALL_AXES);
  sync_plan_position();

  if (event_ != QS_EVENT_ISR_POWER_LOSS) {
    LOG_I("\nTowardStop: start ponit\n");
    LOG_I("X: %.2f, Y:%.2f, Z:%.2f, E: %.2f\n", current_position[0],
          current_position[1], current_position[2], current_position[3]);
  }

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    // if temperature permitted, will raise Z with retracting E
    if(thermalManager.temp_hotend[0].current > 180) {
      //current_position[E_AXIS] -= 6.5;
      //line_to_current_position(60);

      move_to_limited_ze(current_position[Z_AXIS] + 1, current_position[E_AXIS] - 6.5, 30);
    }

    move_to_limited_z(Z_MAX_POS, 15);
    if (event_ != QS_EVENT_ISR_POWER_LOSS) {
      set_current_from_steppers_for_axis(ALL_AXES);
      sync_plan_position();
    }

    // if runout, move X to max
    if (X_HOME_DIR > 0)
      move_to_limited_x(home_offset[X_AXIS] + X_MAX_POS, 30);
    else
      move_to_limited_x(0, 35);

    // move Y to max position
    move_to_limited_xy(current_position[X_AXIS], home_offset[Y_AXIS] + Y_MAX_POS, 30);
    break;

  case MACHINE_TYPE_CNC:
  case MACHINE_TYPE_LASER:
    move_to_limited_z(Z_MAX_POS, 15);
    break;

  default:
    break;
  }

  while (planner.has_blocks_queued()) {
    if (event_ != QS_EVENT_ISR_POWER_LOSS)
      idle();
  }

  // actually, Z_MAX_POS is larger than position of endstop when level is enable
  // so moving will stop in endstop, we must recover stepper's position to
  // current position.
  if (event_ != QS_EVENT_ISR_POWER_LOSS) {
    set_current_from_steppers_for_axis(ALL_AXES);
    sync_plan_position();
  }
}


void QuickStop::Process() {
  if (event_ == QS_EVENT_NONE || stopped_)
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

  TowardStop();

  if (ExecuterHead.MachineType == MACHINE_TYPE_CNC)
    ExecuterHead.CNC.SetPower(0);

  stopped_ = true;

  if (event_ == QS_EVENT_ISR_POWER_LOSS)
    while (1);

  if (SystemStatus.GetCurrentStatus() == SYSTAT_PAUSE_TRIG)
    SystemStatus.SetCurrentStatus(SYSTAT_PAUSE_STOPPED);

  if (SystemStatus.GetCurrentStatus() == SYSTAT_END_TRIG)
    SystemStatus.SetCurrentStatus(SYSTAT_END_FINISH);
}

void QuickStop::Reset() {
  stopped_ = false;
  event_ = QS_EVENT_NONE;
  sync_flag_ = QS_SYNC_NONE;
  disable_stepper_ = false;

  debug_ = QS_EVENT_NONE;
}
