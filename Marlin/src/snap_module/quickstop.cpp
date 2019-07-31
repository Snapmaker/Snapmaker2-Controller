
#include "quickstop.h"
#include "../module/PeriphDevice.h"
#include "../module/PowerPanic.h"
#include "../module/stepper.h"
#include "../module/temperature.h"
#include "../module/printcounter.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../module/StatusControl.h"

QuickStop quickstop;

ErrCode QuickStop::SetEvent(QuickStopEvent e) {
  ErrCode ret = E_SUCCESS;

  DISABLE_ISRS();

  // priority of power-loss is highest
  if (e == QS_EVENT_ISR_POWER_LOSS) {
    // if previous event is not power-loss, override it.
    if (event_ != QS_EVENT_ISR_POWER_LOSS) {
      event_ = e;
      ret = E_SUCCESS;
    }
    else {
      // if previous event is power-loss, return busy
      // indicates we are handling pwoer-loss
      ret = E_BUSY;
    }
  }
  else {
    // if new event is not power-loss, just see if current event is none
    if (event_ != QS_EVENT_NONE) {
      ret = E_BUSY;
    }
    else {
      event_ = e;
      ret = E_SUCCESS;
    }
  }

  ENABLE_ISRS();

  return ret;
}

void QuickStop::CheckISR(block_t *blk) {
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
      return;
    }
    else {
      // power loss happened
      if (SetEvent(QS_EVENT_ISR_POWER_LOSS) == E_SUCCESS)
        new_event = QS_EVENT_ISR_POWER_LOSS;
    }
  }
  else {
    if (powerstat == POWER_LOSS_STATE) {
      if (SetEvent(QS_EVENT_ISR_POWER_LOSS) == E_SUCCESS)
        new_event = QS_EVENT_ISR_POWER_LOSS;
    }
  }

  // here are the common handle for above branchs
  // which has no new event
  if (new_event == QS_EVENT_NONE) {
    if (disable_stepper_ && blk)
      stepper.quick_stop();
    return;
  }

  if (new_event == QS_EVENT_ISR_POWER_LOSS)
    PowerPanicData.TurnOffPower();


  if (sync_flag_ == QS_SYNC_TRIGGER ||
      (sync_flag_ == QS_SYNC_NONE && new_event == QS_EVENT_ISR_POWER_LOSS)) {
    if (blk)
      PowerPanicData.Data.FilePosition = blk->filePos;
    set_current_from_steppers_for_axis(ALL_AXES);
    PowerPanicData.SaveEnv();
  }

  if (new_event == QS_EVENT_ISR_POWER_LOSS)
    PowerPanicData.WriteFlash();

  sync_flag_ = QS_SYNC_ISR_END;

  disable_stepper_ = true;
}

ErrCode QuickStop::Trigger(QuickStopEvent e) {
  if (SetEvent(e) != E_SUCCESS)
    return E_BUSY;

  sync_flag_ = QS_SYNC_TRIGGER;

  return E_SUCCESS;
}

void QuickStop::CleanMoves() {
  clear_command_queue();

  DISABLE_ISRS();

  planner.block_buffer_nonbusy = planner.block_buffer_planned = \
      planner.block_buffer_head = planner.block_buffer_tail;

  ENABLE_ISRS();
}

void QuickStop::TowardStop() {
  // these two variables will clean the latency in planning commands
  // and outputing blocks
  planner.delay_before_delivering = 0;
  planner.cleaning_buffer_counter = 0;

  while (stepper.get_current_block());

  // make it false, will not abort block, then we can output moves
  disable_stepper_ = false;

  stepper.allow_current_block();

  // make sure we are in absolute position mode
  relative_mode = false;

  // Disable the leveling to reduce execution time
  #if HAS_LEVELING
    set_bed_leveling_enabled(false);
    planner.leveling_active = false;
  #endif

  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    // if temperature permitted, will raise Z with retracting E
    if(thermalManager.temp_hotend[0].current > 180) {
      current_position[E_AXIS] -= 4;
    }

    if (event_ == QS_EVENT_ISR_POWER_LOSS) {
      //to avoid pre-block has been aborted, we input a block again
      move_to_limited_z(current_position[Z_AXIS] + 5, 10);
    }
    else
      move_to_limited_z(current_position[Z_AXIS] + 30, 10);

    // move X to original point
    move_to_limited_x(0, 35);
    // move Y to max position
    move_to_limited_xy(current_position[X_AXIS], home_offset[Y_AXIS] + Y_MAX_POS, 30);
    break;

  case MACHINE_TYPE_CNC:
    // close CNC motor
    ExecuterHead.CNC.SetPower(0);

    if (event_ == QS_EVENT_ISR_POWER_LOSS) {
      move_to_limited_z(current_position[Z_AXIS] + 5, 10);
    }
    else
      move_to_limited_z(current_position[Z_AXIS] + 30, 10);

    while (planner.has_blocks_queued()) {
      if (!event_ == QS_EVENT_ISR_POWER_LOSS)
        idle();
    }

    // move to original point
    move_to_limited_xy(0, 0, 50);
    break;

  case MACHINE_TYPE_LASER:

    break;

  default:
    break;
  }
}


void QuickStop::Process() {
  if (event_ == QS_EVENT_NONE || stopped_)
    return;

  CleanMoves();

  // waiting sync_flag_ to become QS_SYNC_ISR_END
  while (sync_flag_ != QS_SYNC_ISR_END);

  TowardStop();

  stopped_ = true;

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
}
