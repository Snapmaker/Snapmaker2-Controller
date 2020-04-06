#include "level_service.h"
#include "M1028.h"

#include "../Marlin.h"
#include "../module/planner.h"
#include "../module/endstops.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"

#include "../gcode/gcode.h"
#include "../gcode/parser.h"

#include "../feature/bedlevel/abl/abl.h"
#include "../feature/bedlevel/bedlevel.h"

LevelService levelservice;

extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;

ErrCode LevelService::DoAutoLeveling(Event_t &event) {
  ErrCode err = E_FAILURE;

  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  LOG_I("e temp: %.2f / %d\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
  LOG_I("b temp: %.2f / %d\n", thermalManager.degBed(), thermalManager.degTargetBed());

  if (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType) {
    // Turn off the heaters
    thermalManager.disable_all_heaters();

    if (!go_home_before_cali && all_axes_homed() &&
      (!position_shift[X_AXIS] && !position_shift[Y_AXIS] && !position_shift[Z_AXIS])) {
      if (current_position[Z_AXIS] < z_limit_in_cali)
        move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
      move_to_limited_x(0, XY_PROBE_FEEDRATE_MM_S);
      planner.synchronize();
    }
    else
      process_cmd_imd("G28");
    process_cmd_imd("G1029 P3"); // set the default probe points, hardcoded

    set_bed_leveling_enabled(false);

    // change the Z max feedrate
    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    endstops.enable_z_probe(true);

    // move quicky firstly to decrease the time
    do_blocking_move_to_z(z_position_before_calibration, speed_in_calibration[Z_AXIS]);
    planner.synchronize();

    if (event.op_code == SETTINGS_OPC_DO_AUTO_LEVELING)
      err = auto_probing(true, false);
    else
      err = auto_probing(true, true);

    endstops.enable_z_probe(false);

    // Recover the Z max feedrate to 20mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = orig_max_z_speed;

    level_mode_ = LEVEL_MODE_AUTO;
  }

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode LevelService::DoManualLeveling(Event_t &event) {
  ErrCode err = E_FAILURE;
  int i, j;
  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  // when user do manual leveling, clear this var to disable fast-calibration
  nozzle_height_probed = 0;

  LOG_I("SC req manual level\n");

  if (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType) {

    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    // Disable all heaters
    thermalManager.disable_all_heaters();
    if (!go_home_before_cali && all_axes_homed() &&
      (!position_shift[X_AXIS] && !position_shift[Y_AXIS] && !position_shift[Z_AXIS])) {
      if (current_position[Z_AXIS] < z_limit_in_cali)
        move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
      move_to_limited_x(0, XY_PROBE_FEEDRATE_MM_S);
      planner.synchronize();
    }
    else
      process_cmd_imd("G28");
    process_cmd_imd("G1029 P3"); // set the default probe points, hardcoded

    set_bed_leveling_enabled(false);

    bilinear_grid_manual();

    // Move Z to 20mm height
    do_blocking_move_to_z(z_position_before_calibration, speed_in_calibration[Z_AXIS]);

    do_blocking_move_to_z(15, 10);

    for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
      for (i = 0; i < GRID_MAX_POINTS_X; i++) {
        MeshPointZ[j * GRID_MAX_POINTS_X + i] = z_values[i][j];
      }
    }

    planner.settings.max_feedrate_mm_s[Z_AXIS] = orig_max_z_speed;

    // save the leveling mode
    level_mode_ = LEVEL_MODE_MANUAL;
    // Preset the index to 99 for initial status
    manual_level_index_ = MESH_POINT_SIZE;

    err = E_SUCCESS;
  }

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode LevelService::SetManualLevelingPoint(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t index;

  if (!event.length) {
    LOG_E("Need to specify point index!\n");
    event.length = 1;
    event.data = &err;
    return hmi.Send(event);
  }
  else {
    index = event.data[0];
    LOG_I("SC req move to pont: %d\n", index);
  }

  if ((index < 10) && (index > 0)) {
    // check point index
    if (manual_level_index_ < 10) {
      // save point index
      MeshPointZ[manual_level_index_] = current_position[Z_AXIS];
      LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", manual_level_index_, current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

      // if got new point, raise Z firstly
      if ((manual_level_index_ != index -1) && current_position[Z_AXIS] < z_position_before_calibration)
        do_blocking_move_to_z(current_position[Z_AXIS] + 3, speed_in_calibration[Z_AXIS]);
    }

    // move to new point
    manual_level_index_ = index -1;
    do_blocking_move_to_logical_xy(_GET_MESH_X(manual_level_index_ % GRID_MAX_POINTS_X),
                    _GET_MESH_Y(manual_level_index_ / GRID_MAX_POINTS_Y), speed_in_calibration[X_AXIS]);
  }

  event.data[0] = E_SUCCESS;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode LevelService::AdjustZOffsetInLeveling(Event_t &event) {
  ErrCode err = E_FAILURE;
  float   offset = 0;

  if (event.length < 4) {
    LOG_E("Need to specify z offset!\n");
    event.length = 1;
    event.data = &err;
    return hmi.Send(event);
  }

  PDU_TO_LOCAL_WORD(offset, event.data);

  offset /= 1000;

  LOG_I("SC req Z offset: %.2f\n", offset);

  // sometimes the bed plane will be under the low limit point
  // to make z can move down always by user, we don't use limited API
  do_blocking_move_to_z(current_position[Z_AXIS] + offset, speed_in_calibration[Z_AXIS]);

  planner.synchronize();

  event.length = 1;
  event.data[0] = 0;

  return hmi.Send(event);
}


ErrCode LevelService::SaveAndExitLeveling(Event_t &event) {
  ErrCode err = E_SUCCESS;

  int i, j;

  event.data = &err;
  event.length = 1;

  LOG_I("SC req save data of leveling\n");

  planner.synchronize();

  if (level_mode_ == LEVEL_MODE_MANUAL && manual_level_index_ < 10) {
    // save the last point
    MeshPointZ[manual_level_index_] = current_position[Z_AXIS];
    LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", manual_level_index_,
        current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
    for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
      for (i = 0; i < GRID_MAX_POINTS_X; i++) {
        z_values[i][j] = MeshPointZ[j * GRID_MAX_POINTS_X + i];
      }
    }

    bed_level_virt_interpolate();
    settings.save();
  }
  else if (level_mode_ == LEVEL_MODE_AUTO) {
    process_cmd_imd("G1029 S0");
  }
  else {
    LOG_E("didn't start leveling!\n");
    err = E_FAILURE;
    return hmi.Send(event);
  }

  LOG_I("new leveling data:\n");
  for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
    for (i = 0; i < GRID_MAX_POINTS_X; i++) {
      LOG_I("%.2f ", z_values[i][j]);
    }
    LOG_I("\n");
  }

  set_bed_leveling_enabled(true);

  // move to stop
  move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
  planner.synchronize();

  // make sure we are in absolute mode
  relative_mode = false;

  // clear flag
  level_mode_ = LEVEL_MODE_INVALD;

  return hmi.Send(event);
}


ErrCode LevelService::ExitLeveling(Event_t &event) {
  ErrCode err = E_SUCCESS;

  LOG_I("SC req exit level\n");

  event.data = &err;
  event.length = 1;

  if (level_mode_ == LEVEL_MODE_INVALD) {
    err = E_FAILURE;
    return hmi.Send(event);
  }

  planner.synchronize();

  //Load
  settings.load();

  set_bed_leveling_enabled(true);

  // move to stop
  move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
  planner.synchronize();

  level_mode_ = LEVEL_MODE_INVALD;
  manual_level_index_ = MANUAL_LEVEL_INDEX_INVALID;

  // make sure we are in absolute mode
  relative_mode = false;

  return hmi.Send(event);
}


ErrCode LevelService::ResetLeveling(Event_t &event) {
  ErrCode err = E_SUCCESS;

  event.data = &err;
  event.length = 1;

  LOG_I("SC req clear leveling data\n");

  planner.synchronize();

  set_bed_leveling_enabled(false);

  for (int i = 0; i < GRID_MAX_POINTS_X; i++)
    for (int j = 0; j < GRID_MAX_POINTS_Y; j++)
      z_values[i][j] = DEFAUT_LEVELING_HEIGHT;

  bed_level_virt_interpolate();

  nozzle_height_probed = 0;

  set_bed_leveling_enabled(true);

  return hmi.Send(event);
}


ErrCode LevelService::SyncPointIndex(uint8_t index) {
  Event_t event = {EID_SETTING_ACK, SETTINGS_OPC_SYNC_LEVEL_POINT};

  uint8_t buffer[2];

  event.data = buffer;
  event.length = 2;

  buffer[0] = 0;
  buffer[1] = index;

  return hmi.Send(event);
}

