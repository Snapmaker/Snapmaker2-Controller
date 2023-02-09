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
#include "bed_level.h"
#include "../gcode/M1028.h"
#include "src/Marlin.h"
#include "src/module/planner.h"
#include "src/module/endstops.h"
#include "src/module/temperature.h"
#include "src/module/probe.h"
#include "src/module/configuration_store.h"
#include "src/gcode/gcode.h"
#include "src/gcode/parser.h"
#include "src/feature/bedlevel/abl/abl.h"
#include "src/feature/bedlevel/bedlevel.h"

#define CALIBRATION_PAPER_THICKNESS 0.1

#define INIT_Z_FOR_DUAL_EXTRUDER    (50)  // mm
#define Z_SPEED_FOR_DUAL_EXTRUDER   (30)  // mm/s
#define XY_SPEED_FOR_DUAL_EXTRUDER  (80)  // mm/s

BedLevelService levelservice;

extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;

ErrCode BedLevelService::DoAutoLeveling(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t grid = 3;
  char cmd[16];

  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  LOG_I("SC req auto level\n");

  if (event.length > 0) {
    if (event.data[0] > 7 || event.data[0] < 2) {
      LOG_E("grid [%u] from SC is out of range [2:7], set to default: 3\n", event.data[0]);
      goto OUT;
    }
    else {
      grid = event.data[0];
    }
  }

  LOG_I("e temp: %.2f / %d\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
  LOG_I("b temp: %.2f / %d\n", thermalManager.degBed(), thermalManager.degTargetBed());

  if (MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) {

    // MUST clear live z offset before G28
    // otherwise will apply live z after homing
    live_z_offset_[0] = 0;
    live_z_offset_[1] = 0;

    process_cmd_imd("G28");

    snprintf(cmd, 16, "G1029 P%u\n", grid);
    process_cmd_imd(cmd);

    set_bed_leveling_enabled(false);

    current_position[Z_AXIS] = Z_MAX_POS;
    sync_plan_position();

    // change the Z max feedrate
    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    endstops.enable_z_probe(true);

    // move quicky firstly to decrease the time
    // move to the first calibration mesh point allow the sensor to detect the bed if the bed
    // is on an unexpected height
    do_blocking_move_to_xy(_GET_MESH_X(0) - (X_PROBE_OFFSET_FROM_EXTRUDER),
                                   _GET_MESH_Y(0) - (Y_PROBE_OFFSET_FROM_EXTRUDER),
                                   speed_in_calibration[X_AXIS]);
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

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode BedLevelService::DoManualLeveling(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;
  uint32_t i, j;
  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  uint8_t grid = 3;
  char cmd[16];

  if (event.length > 0) {
    if (event.data[0] > 7 || event.data[0] < 2) {
      LOG_E("grid [%u] from SC is out of range [2:7], set to default: 3\n", event.data[0]);
      goto OUT;
    }
    else {
      grid = event.data[0];
    }
  }

  // when user do manual leveling, clear this var to disable fast-calibration
  nozzle_height_probed = 0;

  LOG_I("SC req manual level\n");

  if (MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) {

    // MUST clear live z offset before G28
    // otherwise will apply live z after homing
    live_z_offset_[0] = 0;
    live_z_offset_[1] = 0;

    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    process_cmd_imd("G28");

    snprintf(cmd, 16, "G1029 P%u\n", grid);
    process_cmd_imd(cmd);

    set_bed_leveling_enabled(false);

    current_position[Z_AXIS] = Z_MAX_POS;
    sync_plan_position();

    // change the Z max feedrate
    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    // Move Z to 20mm height
    do_blocking_move_to_z(z_position_before_calibration, speed_in_calibration[Z_AXIS]);

    // increase 3mm for first leveling point
    // to avoid nozzle gouging the surface when user place glass on the steel sheet
    do_blocking_move_to_z(12, 10);

    for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
      for (i = 0; i < GRID_MAX_POINTS_X; i++) {
        MeshPointZ[j * GRID_MAX_POINTS_X + i] = z_values[i][j];
      }
    }

    planner.settings.max_feedrate_mm_s[Z_AXIS] = orig_max_z_speed;

    // save the leveling mode
    level_mode_ = LEVEL_MODE_MANUAL;
    // Preset the index to 121 for initial status
    manual_level_index_ = GRID_MAX_POINTS;

    err = E_SUCCESS;
  }

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode BedLevelService::SetManualLevelingPoint(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t index;

  if (!event.length) {
    LOG_E("Need to specify point index!\n");
    goto out;
  }
  else {
    index = event.data[0];
    LOG_I("SC req move to pont: %d\n", index);
  }

  if ((index <= GRID_MAX_POINTS_INDEX) && (index > 0)) {
    // check point index
    if (manual_level_index_ <= GRID_MAX_POINTS_INDEX) {
      // save point index
      MeshPointZ[manual_level_index_] = current_position[Z_AXIS];
      LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", manual_level_index_, current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

      // if got new point, raise Z firstly
      if ((manual_level_index_ != index -1) && current_position[Z_AXIS] < z_position_before_calibration)
        do_blocking_move_to_z(current_position[Z_AXIS] + 3, speed_in_calibration[Z_AXIS]);
    }

    // move to new point
    manual_level_index_ = index -1;
    do_blocking_move_to_xy(_GET_MESH_X(manual_level_index_ % GRID_MAX_POINTS_X),
                    _GET_MESH_Y(manual_level_index_ / GRID_MAX_POINTS_Y), speed_in_calibration[X_AXIS]);
  }

  err = E_SUCCESS;

out:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode BedLevelService::AdjustZOffsetInLeveling(SSTP_Event_t &event) {
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


ErrCode BedLevelService::SaveAndExitLeveling(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  uint32_t i, j;

  event.data = &err;
  event.length = 1;

  LOG_I("SC req save data of leveling\n");

  planner.synchronize();

  if (level_mode_ == LEVEL_MODE_MANUAL && manual_level_index_ <= GRID_MAX_POINTS_INDEX) {
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


ErrCode BedLevelService::ExitLeveling(SSTP_Event_t &event) {
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

  if (ModuleBase::toolhead() == MODULE_TOOLHEAD_DUALEXTRUDER) {
    printer1->ModuleCtrlProximitySwitchPower(0);
    printer1->SelectProbeSensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  }

  // make sure we are in absolute mode
  relative_mode = false;

  return hmi.Send(event);
}

ErrCode BedLevelService::IsLeveled(SSTP_Event_t &event) {
  uint8_t level_status = z_values[0][0] != DEFAUT_LEVELING_HEIGHT;

  LOG_I("SC req is leveled:%d\n", level_status);

  event.data = &level_status;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode BedLevelService::SyncPointIndex(uint8_t index) {
  SSTP_Event_t event = {EID_SETTING_ACK, SETTINGS_OPC_SYNC_LEVEL_POINT};

  uint8_t buffer[2];

  event.data = buffer;
  event.length = 2;

  buffer[0] = 0;
  buffer[1] = index;

  return hmi.Send(event);
}


ErrCode BedLevelService::UpdateLiveZOffset(float offset, uint8_t e/* = 0*/) {
  if (e >= EXTRUDERS) {
    return E_PARAM;
  }

  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead() && MODULE_TOOLHEAD_DUALEXTRUDER != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return E_FAILURE;
  }

  if (axis_is_homing) {
    LOG_I("refuse to set live_z_offset when machine is homing");
    return E_BUSY;
  }

  if (offset < LIVE_Z_OFFSET_MIN || offset > LIVE_Z_OFFSET_MAX) {
    LOG_E("offset is out of range: %.2f\n", offset);
    return E_PARAM;
  }

  if (offset == live_z_offset_[e]) {
    LOG_W("offset is same with old\n");
    return E_PARAM;
  }

  if (e == active_extruder) {
    // PauseTrigger(TRIGGER_SOURCE_SC);
    // ResumeTrigger(TRIGGER_SOURCE_SC);
    planner.synchronize();
    float cur_z = current_position[Z_AXIS];
    do_blocking_move_to_z(current_position[Z_AXIS] + (offset - live_z_offset_[e]), 5);
    planner.synchronize();
    current_position[Z_AXIS] = cur_z;
    sync_plan_position();
  }

  LOG_I("extruder: %d, new live Z: %.3f, delta: %.3f\n", e, offset, (offset - live_z_offset_[e]));

  live_z_offset_[e] = offset;
  live_z_offset_updated_ = true;
  return E_SUCCESS;
}


void BedLevelService::ApplyLiveZOffset(uint8_t e) {
  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead() && MODULE_TOOLHEAD_DUALEXTRUDER != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return;
  }

  if ((MODULE_TOOLHEAD_DUALEXTRUDER == ModuleBase::toolhead()) && (e != active_extruder)) {
    return;
  }

  planner.synchronize();
  float cur_z = current_position[Z_AXIS];

  do_blocking_move_to_z(current_position[Z_AXIS] + live_z_offset_[e], 5);
  planner.synchronize();

  current_position[Z_AXIS] = cur_z;
  sync_plan_position();

  LOG_I("Apply hotend[%u] Z offset: %.2f\n", e, live_z_offset_[e]);
}

void BedLevelService::UnapplyLiveZOffset(uint8_t e) {
  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead() && MODULE_TOOLHEAD_DUALEXTRUDER != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return;
  }

  if ((MODULE_TOOLHEAD_DUALEXTRUDER == ModuleBase::toolhead()) && (e != active_extruder)) {
    LOG_E("cannot unapply live z offset of another extruder!\n");
    return;
  }

  planner.synchronize();
  float cur_z = current_position[Z_AXIS];

  do_blocking_move_to_z(current_position[Z_AXIS] - live_z_offset_[e], 5);
  planner.synchronize();

  current_position[Z_AXIS] = cur_z;
  sync_plan_position();

  LOG_I("Unapply hotend[%u] Z offset: %.2f\n", e, live_z_offset_[e]);
}


void BedLevelService::SaveLiveZOffset() {
  if (live_z_offset_updated_) {
    live_z_offset_updated_ = false;
    settings.save();
  }
}

// for dualextruder
ErrCode BedLevelService::ProbeSensorCalibrationLeftExtruderAutoProbe() {
  ErrCode err = E_SUCCESS;

  LOG_I("ProbeSensorCalibrationLeftExtruder\n");

  live_z_offset_temp_[0] = live_z_offset_[0];
  live_z_offset_temp_[1] = live_z_offset_[1];
  live_z_offset_[0] = live_z_offset_[1] = 0;

  feedrate_percentage = 100;
  // go home will make sure active left extruder
  process_cmd_imd("G28 N");
  printer1->SelectProbeSensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);

  planner.synchronize();

  set_bed_leveling_enabled(false);

  float x, y;
  get_center_coordinates_of_bed(x, y);
  endstops.enable_z_probe(true);
  do_blocking_move_to_xy(x, y, XY_SPEED_FOR_DUAL_EXTRUDER);
  do_blocking_move_to_z(INIT_Z_FOR_DUAL_EXTRUDER, Z_SPEED_FOR_DUAL_EXTRUDER);
  planner.synchronize();
  // disable checking
  printer1->ModuleCtrlSetExtruderChecking(false);
  left_extruder_auto_probe_position_ = probe_pt(x, y, PROBE_PT_RAISE, 0, false);
  // enable checking
  printer1->ModuleCtrlSetExtruderChecking(true);

  if (isnan(left_extruder_auto_probe_position_)) {
    err = E_FAILURE;
    left_extruder_auto_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;
    left_extruder_manual_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;
  }

  LOG_I("probed z value: %.2f\n", left_extruder_auto_probe_position_);
  return err;
}

ErrCode BedLevelService::ProbeSensorCalibrationRightExtruderAutoProbe() {
  ErrCode err = E_SUCCESS;
  LOG_I("hmi request right probe sensor auto calibration\n");

  printer1->ToolChange(1, false);
  printer1->SelectProbeSensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);

  float x, y;
  get_center_coordinates_of_bed(x, y);

  endstops.enable_z_probe(true);

  do_blocking_move_to_xy(x, y, XY_SPEED_FOR_DUAL_EXTRUDER);
  if (current_position[Z_AXIS] > INIT_Z_FOR_DUAL_EXTRUDER) {
    do_blocking_move_to_z(INIT_Z_FOR_DUAL_EXTRUDER, Z_SPEED_FOR_DUAL_EXTRUDER);
  }

  printer1->ModuleCtrlSetExtruderChecking(false);
  right_extruder_auto_probe_position_ = probe_pt(x, y, PROBE_PT_RAISE, 0, false);
  printer1->ModuleCtrlSetExtruderChecking(true);

  if (isnan(right_extruder_auto_probe_position_)) {
    err = E_FAILURE;
    right_extruder_auto_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;
    right_extruder_manual_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;
  }

  endstops.enable_z_probe(false);
  LOG_I("probed z value: %.2f\n", right_extruder_auto_probe_position_);
  return err;
}

ErrCode BedLevelService::ProbeSensorCalibrationLeftExtruderManualProbe() {
  ErrCode err = E_SUCCESS;

  LOG_I("hmi request left extruder manual probe\n");
  printer1->ToolChange(0, false);

  return err;
}

ErrCode BedLevelService::ProbeSensorCalibrationRightExtruderManualProbe() {
  ErrCode err = E_SUCCESS;

  LOG_I("hmi request right extruder manual probe\n");
  printer1->ToolChange(1, false);

  return err;
}

ErrCode BedLevelService::ProbeSensorCalibraitonLeftExtruderPositionConfirm() {
  ErrCode err = E_SUCCESS;

  left_extruder_manual_probe_position_ = current_position[Z_AXIS];
  LOG_I("confirm left extruder manual probe position: %.3f\n", left_extruder_manual_probe_position_);

  float left_z_compensation  = left_extruder_manual_probe_position_ - CALIBRATION_PAPER_THICKNESS - left_extruder_auto_probe_position_;
  float right_z_compensation = right_extruder_manual_probe_position_ - CALIBRATION_PAPER_THICKNESS - right_extruder_auto_probe_position_;
  LOG_I("z_compensation: %.2f, %.2f\n", left_z_compensation, right_z_compensation);
  printer1->SetZCompensation(left_z_compensation, 0);
  printer1->SetZCompensation(right_z_compensation, 1);

  hotend_offset[Z_AXIS][1] = left_extruder_manual_probe_position_ - right_extruder_manual_probe_position_;
  LOG_I("hotend_offset_z: %.2f\n", hotend_offset[Z_AXIS][1]);
  printer1->ModuleCtrlSaveHotendOffset(hotend_offset[Z_AXIS][1], Z_AXIS);

  if (current_position[Z_AXIS] + 100 < soft_endstop[Z_AXIS].max)
    do_blocking_move_to_z(current_position[Z_AXIS] + 100, Z_SPEED_FOR_DUAL_EXTRUDER);
  else
    do_blocking_move_to_z(soft_endstop[Z_AXIS].max, Z_SPEED_FOR_DUAL_EXTRUDER);

  set_bed_leveling_enabled(true);
  endstops.enable_z_probe(false);

  settings.save();

  return err;
}

ErrCode BedLevelService::ProbeSensorCalibraitonRightExtruderPositionConfirm() {
  ErrCode err = E_SUCCESS;

  right_extruder_manual_probe_position_ = current_position[Z_AXIS];
  LOG_I("confirm right extruder manual probe position: %.3f\n", right_extruder_manual_probe_position_);
  return err;
}

ErrCode BedLevelService::ProbeSensorCalibraitonAbort() {
  live_z_offset_[0] = live_z_offset_temp_[0];
  live_z_offset_[1] = live_z_offset_temp_[1];
  LOG_I("ProbeSensorCalibraitonAbort, live z0: %.2f, z1: %.2f\n", live_z_offset_[0], live_z_offset_[1]);
  left_extruder_auto_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;
  right_extruder_auto_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;
  left_extruder_manual_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;
  right_extruder_manual_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;

  printer1->ToolChange(0, false);
  set_bed_leveling_enabled(true);
  printer1->ModuleCtrlSetExtruderChecking(true);

  return E_SUCCESS;
}

ErrCode BedLevelService::DoDualExtruderAutoLeveling(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  uint8_t grid = 3;
  char cmd[16];

  uint32_t fault = systemservice.GetFaultFlag();
  if (fault & (FAULT_FLAG_3DP2E_EXTRUDER_MISMATCH | FAULT_FLAG_3DP2E_UNKNOWN_NOZZLE)) {
    LOG_E("cannot start leveling cause exception: 0x%x\n", fault);
    err = E_HARDWARE;
    goto EXIT;
  }

  if (event.length > 0) {
    if (event.data[0] > 11 || event.data[0] < 2) {
      LOG_E("grid [%u] is out of range [2:11], set to default: 3\n", event.data[0]);
      err = E_PARAM;
      goto EXIT;
    }
    else {
      grid = event.data[0];
    }
  }
  else {
    err = E_PARAM;
    goto EXIT;
  }

  LOG_I("hmi req 3dp2e auto leveling, grid: %u\n", event.data[0]);

  feedrate_percentage = 100;

  live_z_offset_[0] = 0;
  live_z_offset_[1] = 0;

  // clear temporary buffer
  for (int x = 0; x < GRID_MAX_NUM; x++) {
    for (int y = 0; y < GRID_MAX_NUM; y++) {
      z_values_tmp[x][y] = 0;
    }
  }

  // go home will make sure active left extruder
  process_cmd_imd("G28 N\n");

  snprintf(cmd, 16, "G1029 P%u\n", grid);
  process_cmd_imd(cmd);

  set_bed_leveling_enabled(false);

  // make sure PROXIMITY_SWITCH is active
  printer1->ModuleCtrlProximitySwitchPower(1);
  printer1->SelectProbeSensor(PROBE_SENSOR_PROXIMITY_SWITCH);

  endstops.enable_z_probe(true);

EXIT:
  event.data   = &err;
  event.length = 1;
  return hmi.Send(event);
}

ErrCode BedLevelService::DualExtruderAutoLevelingProbePoint(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  float probe_x, probe_y;
  uint8_t x_index, y_index;

  uint32_t fault = systemservice.GetFaultFlag();
  if (fault & (FAULT_FLAG_3DP2E_EXTRUDER_MISMATCH | FAULT_FLAG_3DP2E_UNKNOWN_NOZZLE)) {
    LOG_E("cannot start leveling cause exception: 0x%x\n", fault);
    err = E_HARDWARE;
    goto EXIT;
  }

  probe_point_ = event.data[0] - 1;
  if (probe_point_ > GRID_MAX_POINTS_INDEX) {
    LOG_E("got invalid probe point: %u\n", probe_point_);
    err = E_PARAM;
    goto EXIT;
  }

  x_index = probe_point_ % GRID_MAX_POINTS_X;
  y_index = probe_point_ / GRID_MAX_POINTS_Y;
  probe_x = _GET_MESH_X(x_index);
  probe_y = _GET_MESH_Y(y_index);

  LOG_I("hmi req auto probe %u point, x: %.3f, y: %.3f\n", probe_point_, probe_x, probe_y);

  probe_x -= (DUALEXTRUDER_X_PROBE_OFFSET_FROM_EXTRUDER);                     // Get the nozzle position
  probe_y -= (DUALEXTRUDER_Y_PROBE_OFFSET_FROM_EXTRUDER);

  if (probe_point_ == 0) {
    do_blocking_move_to_xy(probe_x, probe_y, XY_SPEED_FOR_DUAL_EXTRUDER);
    do_blocking_move_to_z(INIT_Z_FOR_DUAL_EXTRUDER, Z_SPEED_FOR_DUAL_EXTRUDER);
  } else {
    do_blocking_move_to_xy(probe_x, probe_y, XY_SPEED_FOR_DUAL_EXTRUDER);
  }
  planner.synchronize();

  // make sure power of probe sensor is turned on
  printer1->ModuleCtrlProximitySwitchPower(1);

  printer1->ModuleCtrlSetExtruderChecking(false);
  z_values_tmp[x_index][y_index]  = probe_pt(probe_x, probe_y, PROBE_PT_RAISE, 0, false);
  printer1->ModuleCtrlSetExtruderChecking(true);

  if (isnan(z_values_tmp[x_index][y_index])) {
    LOG_E("got a non number!\n");
    err = E_FAILURE;
  }

EXIT:
  event.data   = &err;
  event.length = 1;
  return hmi.Send(event);
}

ErrCode BedLevelService::FinishDualExtruderAutoLeveling(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  float probe_x, probe_y;
  uint8_t x_index, y_index;

  LOG_I("hmi req exit 3dp2e auto leveling\n");

  uint32_t fault = systemservice.GetFaultFlag();
  if (fault & (FAULT_FLAG_3DP2E_EXTRUDER_MISMATCH | FAULT_FLAG_3DP2E_UNKNOWN_NOZZLE)) {
    LOG_E("cannot do leveling cause exception: 0x%x\n", fault);
    err = E_HARDWARE;
    goto EXIT;
  }

  // move to center or left-front of Bed
  x_index     = (uint8_t)(GRID_MAX_POINTS_X / 2);
  if (!(GRID_MAX_POINTS_X % 2) && (x_index > 0))
    x_index--;

  y_index     = (uint8_t)(GRID_MAX_POINTS_Y / 2);
  if (!(GRID_MAX_POINTS_Y % 2) && (y_index > 0))
    y_index--;

  probe_x     = _GET_MESH_X(x_index);
  probe_y     = _GET_MESH_Y(y_index);

  do_blocking_move_to_xy(probe_x, probe_y, XY_SPEED_FOR_DUAL_EXTRUDER);
  planner.synchronize();

  printer1->SelectProbeSensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  printer1->ModuleCtrlSetExtruderChecking(false);
  left_extruder_auto_probe_position_ = probe_pt(probe_x, probe_y, PROBE_PT_RAISE, 0, false);
  printer1->ModuleCtrlSetExtruderChecking(true);
  if (isnan(left_extruder_auto_probe_position_)) {
    err = E_FAILURE;
    goto EXIT;
  }

  {
    float left_z_compensation = 1.0, right_z_compensation = 1.0;
    printer1->GetZCompensation(left_z_compensation, right_z_compensation);
    float left_extruder_touch_bed_position  = left_extruder_auto_probe_position_ + left_z_compensation;
    float z_offset = z_values_tmp[x_index][y_index] - left_extruder_touch_bed_position;
    for (uint32_t i = 0; i < GRID_MAX_POINTS_X; i++) {
      for (uint32_t j = 0; j < GRID_MAX_POINTS_Y; j++) {
        // save temp data to actual position
        z_values[i][j] = z_values_tmp[i][j] - z_offset;
      }
    }
    bed_level_virt_interpolate();
    settings.save();
    print_bilinear_leveling_grid();
    print_bilinear_leveling_grid_virt();
  }

EXIT:
  if (current_position[Z_AXIS] + 100 < soft_endstop[Z_AXIS].max)
    do_blocking_move_to_z(current_position[Z_AXIS] + 100, Z_SPEED_FOR_DUAL_EXTRUDER);
  else
    do_blocking_move_to_z(soft_endstop[Z_AXIS].max, Z_SPEED_FOR_DUAL_EXTRUDER);

  endstops.enable_z_probe(false);
  set_bed_leveling_enabled(true);
  printer1->ModuleCtrlProximitySwitchPower(0);
  event.data   = &err;
  event.length = 1;

  return hmi.Send(event);
}

ErrCode BedLevelService::DoDualExtruderManualLeveling(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;
  uint8_t grid = 3;
  char cmd[16];
  uint32_t i, j;

  LOG_I("hmi req dual extruder manual leveling\n");
  if (event.length > 0) {
    if (event.data[0] > 11 || event.data[0] < 2) {
      LOG_E("grid [%u] from hmi is out of range [2:11], set to default: 3\n", event.data[0]);
      goto EXIT;
    } else {
      grid = event.data[0];
    }
  }

  i = systemservice.GetFaultFlag();
  if (i & (ETYPE_3DP2E_EXTRUDER_MISMATCH | ETYPE_3DP2E_UNKNOWN_NOZZLE)) {
    err = E_HARDWARE;
    goto EXIT;
  }

  live_z_offset_[0] = 0;
  live_z_offset_[1] = 0;

  feedrate_percentage = 100;

  process_cmd_imd("G28 N");
  snprintf(cmd, 16, "G1029 P%u\n", grid);
  process_cmd_imd(cmd);
  set_bed_leveling_enabled(false);

  {
    float probe_x = _GET_MESH_X(0);
    float probe_y = _GET_MESH_Y(0);
    do_blocking_move_to_xy(probe_x, probe_y, XY_SPEED_FOR_DUAL_EXTRUDER);
    do_blocking_move_to_z(INIT_Z_FOR_DUAL_EXTRUDER, Z_SPEED_FOR_DUAL_EXTRUDER);
    planner.synchronize();
  }

  manual_level_index_ = GRID_MAX_POINTS;
  for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
    for (i = 0; i < GRID_MAX_POINTS_X; i++) {
      MeshPointZ[j * GRID_MAX_POINTS_X + i] = z_values[i][j];
    }
  }

EXIT:
  event.data   = &err;
  event.length = 1;
  return hmi.Send(event);
}

ErrCode BedLevelService::DualExtruderManualLevelingProbePoint(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  uint8_t index;
  uint32_t fault;

  if (!event.length) {
    LOG_E("Need to specify point index!\n");
    err = E_PARAM;
    goto out;
  } else {
    index = event.data[0];
    LOG_I("SC req move to pont: %d\n", index);
  }

  fault = systemservice.GetFaultFlag();
  if (fault & (FAULT_FLAG_3DP2E_EXTRUDER_MISMATCH | FAULT_FLAG_3DP2E_UNKNOWN_NOZZLE)) {
    LOG_E("cannot do leveling cause exception: 0x%x\n", fault);
    err = E_EXCEPTION;
    goto out;
  }

  if ((index <= GRID_MAX_POINTS_INDEX) && (index > 0)) {
    // check point index
    if (manual_level_index_ <= GRID_MAX_POINTS_INDEX) {
      // save point index
      MeshPointZ[manual_level_index_] = current_position[Z_AXIS] - CALIBRATION_PAPER_THICKNESS;
      LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", manual_level_index_, current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

      // if got new point, raise Z firstly
      if ((manual_level_index_ != index -1) && current_position[Z_AXIS] < INIT_Z_FOR_DUAL_EXTRUDER)
        do_blocking_move_to_z(current_position[Z_AXIS] + 3, Z_SPEED_FOR_DUAL_EXTRUDER / 2);
    }

    // move to new point
    manual_level_index_ = index -1;
    do_blocking_move_to_xy(_GET_MESH_X(manual_level_index_ % GRID_MAX_POINTS_X),
                    _GET_MESH_Y(manual_level_index_ / GRID_MAX_POINTS_Y), XY_SPEED_FOR_DUAL_EXTRUDER);
  } else {
    err = E_PARAM;
  }

out:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

ErrCode BedLevelService::FinishDualExtruderManualLeveling(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  uint32_t i, j;

  LOG_I("hmi req exit 3dp2e manual leveling\n");

  MeshPointZ[manual_level_index_] = current_position[Z_AXIS] - CALIBRATION_PAPER_THICKNESS;
  for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
    for (i = 0; i < GRID_MAX_POINTS_X; i++) {
      z_values[i][j] = MeshPointZ[j * GRID_MAX_POINTS_X + i];
    }
  }

  bed_level_virt_interpolate();
  settings.save();
  print_bilinear_leveling_grid();
  print_bilinear_leveling_grid_virt();

  if (current_position[Z_AXIS] + 100 < soft_endstop[Z_AXIS].max)
    do_blocking_move_to_z(current_position[Z_AXIS] + 100, Z_SPEED_FOR_DUAL_EXTRUDER);
  else
    do_blocking_move_to_z(soft_endstop[Z_AXIS].max, Z_SPEED_FOR_DUAL_EXTRUDER);

  set_bed_leveling_enabled(true);
  event.data = &err;
  event.length = 1;
  return hmi.Send(event);
}

ErrCode BedLevelService::DualExtruderAutoBedDetect(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  uint32_t fault = systemservice.GetFaultFlag();
  if (fault & (FAULT_FLAG_3DP2E_EXTRUDER_MISMATCH | FAULT_FLAG_3DP2E_UNKNOWN_NOZZLE)) {
    LOG_E("cannot do leveling cause exception: 0x%x\n", fault);
    err = E_HARDWARE;
    goto EXIT;
  }

  feedrate_percentage = 100;

  switch (event.data[0]) {
    case 0:
      err = DualExtruderLeftExtruderAutoBedDetect();
      break;
    case 1:
      err = DualExtruderRightExtruderAutoBedDetect();
      break;
    default:
      err = E_PARAM;
      break;
  }

  LOG_I("auto bed detect ret = %u\n", err);

EXIT:
  event.data = &err;
  event.length = 1;
  return hmi.Send(event);
}

ErrCode BedLevelService::DualExtruderLeftExtruderAutoBedDetect() {
  ErrCode err = E_SUCCESS;
  LOG_I("hmi request left auto bed detect\n");

  live_z_offset_[0] = 0;
  live_z_offset_[1] = 0;

  feedrate_percentage = 100;

  // go home will make sure active left extruder
  process_cmd_imd("G28 N");

  planner.synchronize();

  set_bed_leveling_enabled(false);

  float x, y;
  get_center_coordinates_of_bed(x, y);
  printer1->SelectProbeSensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  endstops.enable_z_probe(true);
  do_blocking_move_to_xy(x, y, XY_SPEED_FOR_DUAL_EXTRUDER);
  do_blocking_move_to_z(INIT_Z_FOR_DUAL_EXTRUDER, Z_SPEED_FOR_DUAL_EXTRUDER);
  planner.synchronize();
  printer1->ModuleCtrlSetExtruderChecking(false);
  left_extruder_auto_probe_position_ = probe_pt(x, y, PROBE_PT_RAISE, 0, false);
  printer1->ModuleCtrlSetExtruderChecking(true);
  endstops.enable_z_probe(false);

  if (isnan(left_extruder_auto_probe_position_)) {
    err = E_FAILURE;
    left_extruder_auto_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;

    endstops.enable_z_probe(false);
    set_bed_leveling_enabled(true);
  }

  LOG_I("probed z value: %.3f\n", left_extruder_auto_probe_position_);

  return err;
}

ErrCode BedLevelService::DualExtruderRightExtruderAutoBedDetect() {
  ErrCode err = E_SUCCESS;
  LOG_I("hmi request right auto bed detect\n");

  // TODO: need to check if leveling disable
  printer1->ToolChange(1, false);

  printer1->SelectProbeSensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
  endstops.enable_z_probe(true);

  float x, y;
  get_center_coordinates_of_bed(x, y);
  do_blocking_move_to_xy(x, y, XY_SPEED_FOR_DUAL_EXTRUDER);
  if (current_position[Z_AXIS] > INIT_Z_FOR_DUAL_EXTRUDER) {
    do_blocking_move_to_z(INIT_Z_FOR_DUAL_EXTRUDER, Z_SPEED_FOR_DUAL_EXTRUDER);
  }

  printer1->ModuleCtrlSetExtruderChecking(false);
  right_extruder_auto_probe_position_ = probe_pt(x, y, PROBE_PT_RAISE, 0, false);
  printer1->ModuleCtrlSetExtruderChecking(true);
  endstops.enable_z_probe(false);

  if (isnan(right_extruder_auto_probe_position_)) {
    err = E_FAILURE;
    right_extruder_auto_probe_position_ = INIT_Z_FOR_DUAL_EXTRUDER;
    return err;
  }
  LOG_I("probed z value: %.3f\n", right_extruder_auto_probe_position_);

  float left_z_compensation = 1, right_z_compensation = 1;
  printer1->GetZCompensation(left_z_compensation, right_z_compensation);

  float left_extruder_touch_bed_position  = left_extruder_auto_probe_position_ + left_z_compensation;
  float right_extruder_touch_bed_position = right_extruder_auto_probe_position_ + right_z_compensation;
  hotend_offset[Z_AXIS][1] = left_extruder_touch_bed_position - right_extruder_touch_bed_position;

  LOG_I("compensation: [%.3f, %.3f], bed pos:[%.3f, %.3f]\n", left_z_compensation, right_z_compensation,
    left_extruder_touch_bed_position, right_extruder_touch_bed_position);

  float z_offset = z_values[GRID_MAX_POINTS_X/2][GRID_MAX_POINTS_Y/2] - left_extruder_touch_bed_position;
  for (uint32_t i = 0; i < GRID_MAX_POINTS_X; i++) {
    for (uint32_t j = 0; j < GRID_MAX_POINTS_Y; j++) {
      z_values[i][j] -= z_offset;
    }
  }

  LOG_I("hotend z offset: %.3f, level z offset: %.3f\n", hotend_offset[Z_AXIS][1], z_offset);
  bed_level_virt_interpolate();
  settings.save();
  print_bilinear_leveling_grid();
  print_bilinear_leveling_grid_virt();

  if (current_position[Z_AXIS] + 100 < soft_endstop[Z_AXIS].max)
    do_blocking_move_to_z(current_position[Z_AXIS] + 100, Z_SPEED_FOR_DUAL_EXTRUDER);
  else
    do_blocking_move_to_z(soft_endstop[Z_AXIS].max, Z_SPEED_FOR_DUAL_EXTRUDER);

  set_bed_leveling_enabled(true);

  printer1->ToolChange(0, false);

  return err;
}

ErrCode BedLevelService::DualExtruderManualBedDetect(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  uint32_t fault = systemservice.GetFaultFlag();
  if (fault & (FAULT_FLAG_3DP2E_EXTRUDER_MISMATCH | FAULT_FLAG_3DP2E_UNKNOWN_NOZZLE)) {
    LOG_E("cannot do leveling cause exception: 0x%x\n", fault);
    err = E_HARDWARE;
    goto EXIT;
  }

  feedrate_percentage = 100;

  switch (event.data[0]) {
    case 0:
      err = DualExtruderLeftExtruderManualBedDetect();
      break;
    case 1:
      err = DualExtruderRightExtruderManualBedDetect();
      break;
    case 2:
      err = FinishDualExtruderManualBedDetect();
      break;
    default:
      err = E_PARAM;
      break;
  }

EXIT:
  event.data = &err;
  event.length = 1;
  return hmi.Send(event);
}

ErrCode BedLevelService::DualExtruderLeftExtruderManualBedDetect() {
  ErrCode err = E_SUCCESS;
  LOG_I("hmi request left manual bed detect\n");

  live_z_offset_[0] = 0;
  live_z_offset_[1] = 0;

  feedrate_percentage = 100;

  // make active left extruder
  process_cmd_imd("G28 N");
  planner.synchronize();

  set_bed_leveling_enabled(false);

  float x, y;
  get_center_coordinates_of_bed(x, y);
  do_blocking_move_to_xy(x, y, XY_SPEED_FOR_DUAL_EXTRUDER);
  do_blocking_move_to_z(INIT_Z_FOR_DUAL_EXTRUDER, Z_SPEED_FOR_DUAL_EXTRUDER);
  planner.synchronize();

  return err;
}

ErrCode BedLevelService::DualExtruderRightExtruderManualBedDetect() {
  left_extruder_manual_probe_position_ = current_position[Z_AXIS] - CALIBRATION_PAPER_THICKNESS;

  do_blocking_move_to_z(current_position[Z_AXIS] + 3, Z_SPEED_FOR_DUAL_EXTRUDER / 2);
  printer1->ToolChange(1, false);

  return E_SUCCESS;
}

ErrCode BedLevelService::FinishDualExtruderManualBedDetect() {
  right_extruder_manual_probe_position_ = current_position[Z_AXIS] - CALIBRATION_PAPER_THICKNESS;

  hotend_offset[Z_AXIS][1] = left_extruder_manual_probe_position_ - right_extruder_manual_probe_position_;
  float z_offset = z_values[GRID_MAX_POINTS_X/2][GRID_MAX_POINTS_Y/2] - left_extruder_manual_probe_position_;
  for (uint32_t i = 0; i < GRID_MAX_POINTS_X; i++) {
    for (uint32_t j = 0; j < GRID_MAX_POINTS_Y; j++) {
      z_values[i][j] -= z_offset;
    }
  }
  bed_level_virt_interpolate();
  settings.save();
  print_bilinear_leveling_grid();
  print_bilinear_leveling_grid_virt();

  if (current_position[Z_AXIS] + 100 < soft_endstop[Z_AXIS].max)
    do_blocking_move_to_z(current_position[Z_AXIS] + 100, Z_SPEED_FOR_DUAL_EXTRUDER);
  else
    do_blocking_move_to_z(soft_endstop[Z_AXIS].max, Z_SPEED_FOR_DUAL_EXTRUDER);

  set_bed_leveling_enabled(true);

  printer1->ToolChange(0,false);

  return E_SUCCESS;
}

