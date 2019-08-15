
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

ErrCode QuickStop::SetEvent(QuickStopEvent e) {
  ErrCode ret = E_SUCCESS;

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

  return ret;
}

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
      if (SetEvent(QS_EVENT_ISR_POWER_LOSS) == E_SUCCESS)
        new_event = QS_EVENT_ISR_POWER_LOSS;
    }
  }
  else {
    if (powerstat == POWER_LOSS_STATE) {
      if (SetEvent(QS_EVENT_ISR_POWER_LOSS) == E_SUCCESS)
        new_event = QS_EVENT_ISR_POWER_LOSS;
    }
    else if (sync_flag_ == QS_SYNC_TRIGGER) {
      new_event = event_;
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
    powerpanic.TurnOffPower();


  if (new_event != QS_EVENT_NONE) {
    if (blk)
      powerpanic.Data.FilePosition = blk->filePos;
    set_current_from_steppers_for_axis(ALL_AXES);
    powerpanic.SaveEnv();
  }

  if (new_event == QS_EVENT_ISR_POWER_LOSS)
    powerpanic.WriteFlash();

  sync_flag_ = QS_SYNC_ISR_END;

  disable_stepper_ = true;
}

ErrCode QuickStop::Trigger(QuickStopEvent e) {
  // only stepper ISR may call SetEvent() at the same time
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  ErrCode ret = SetEvent(e);
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  if (ret != E_SUCCESS) {
    LOG_W("set quick stop event failed, err = %d\n", ret);
    return E_BUSY;
  }

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

  while (stepper.get_current_block()) {
    ENABLE_STEPPER_DRIVER_INTERRUPT();
    stepper.quick_stop();
  }

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
      line_to_current_position(60);
    }

    if (event_ == QS_EVENT_ISR_POWER_LOSS) {
      //to avoid pre-block has been aborted, we input a block again
      move_to_limited_z(current_position[Z_AXIS] + 5, 10);
    }
    else
      move_to_limited_z(current_position[Z_AXIS] + 30, 10);

    // if runout, move X to max
    if (event_ == QS_EVENT_RUNOUT) {
      move_to_limited_x(X_MAX_POS, 35);
    }
    else {
      // move X to original point
      move_to_limited_x(0, 35);
    }

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

    // move to original point
    move_to_limited_xy(0, 0, 50);
    break;

  case MACHINE_TYPE_LASER:

    break;

  default:
    break;
  }

  while (planner.has_blocks_queued()) {
    if (event_ != QS_EVENT_ISR_POWER_LOSS)
      idle();
  }
}


void QuickStop::Process() {
  if (event_ == QS_EVENT_NONE || stopped_)
    return;

  // make sure stepper ISR is enabled
  // need it to save data
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  CleanMoves();

  // waiting sync_flag_ to become QS_SYNC_ISR_END
  while (sync_flag_ != QS_SYNC_ISR_END);

  LOG_I("last position: X: %f, Y: %f, Z: %f, E: %f\n", powerpanic.Data.PositionData[X_AXIS],
      powerpanic.Data.PositionData[Y_AXIS], powerpanic.Data.PositionData[Z_AXIS], powerpanic.Data.PositionData[E_AXIS]);
  LOG_I("Last line number: %d\n", powerpanic.Data.FilePosition);
  LOG_I("last position shift:\n");
  LOG_I("X: %f, Y: %f, Z: %f\n", powerpanic.Data.position_shift[0],
      powerpanic.Data.position_shift[1], powerpanic.Data.position_shift[2]);

  if (event_ == QS_EVENT_ISR_POWER_LOSS) {
    BreathLightClose();

    // disble timer except the stepper's
    rcc_clk_disable(TEMP_TIMER_DEV->clk_id);
    rcc_clk_disable(TIMER7->clk_id);

    // disalbe ADC
    rcc_clk_disable(ADC1->clk_id);
    rcc_clk_disable(ADC2->clk_id);

    //disble DMA
    rcc_clk_disable(DMA1->clk_id);
    rcc_clk_disable(DMA2->clk_id);

    // disable other unnecessary soc peripherals
    // disable usart
    //rcc_clk_disable(MSerial1.c_dev()->clk_id);
    //rcc_clk_disable(MSerial2.c_dev()->clk_id);
    //rcc_clk_disable(MSerial3.c_dev()->clk_id);

#if ENABLED(EXECUTER_CANBUS_SUPPORT)
  // turn off hot end and FAN
	// if (ExecuterHead.MachineType == MACHINE_TYPE_3DPRINT) {
	// 	ExecuterHead.SetTemperature(0, 0);
	// }
#endif

    }

  TowardStop();

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
