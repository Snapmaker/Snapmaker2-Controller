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
#include "src/module/configuration_store.h"
#include "src/gcode/gcode.h"
#include "src/gcode/parser.h"
#include "src/feature/bedlevel/abl/abl.h"
#include "src/feature/bedlevel/bedlevel.h"
#include "src/module/tool_change.h"
#include "../module/toolhead_3dp.h"
#include "../../../Marlin/src/module/motion.h"
#include "../../../Marlin/src/module/probe.h"
#include "../module/linear.h"

BedLevelService levelservice;

extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;

ErrCode BedLevelService::CalibrateExtruderTriggerStroke(float x, float y) {

  if (x > DUAL_EXTRUDER_X_MAX_POS || x < DUAL_EXTRUDER_X_MIN_POS || y > DUAL_EXTRUDER_Y_MAX_POS || y < DUAL_EXTRUDER_Y_MIN_POS) {
    return E_PARAM;
  }

  if (!all_axes_homed()) {
    process_cmd_imd("G28");
  }

  set_bed_leveling_enabled(false);
  do_blocking_move_to_logical_xy(x, y, 80);
  hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = 0;
  endstops.enable_z_probe(true);

  tool_change(TOOLHEAD_3DP_EXTRUDER0);
  printer1->SetProbeSensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  do_blocking_move_to_z(50, 30);
  hotend_triggered_z[TOOLHEAD_3DP_EXTRUDER0] = probe_pt(RAW_X_POSITION(x), RAW_Y_POSITION(y), PROBE_PT_RAISE, 0, false);
  LOG_I("hotend0 triggered z: %f\n", hotend_triggered_z[TOOLHEAD_3DP_EXTRUDER0]);

  tool_change(TOOLHEAD_3DP_EXTRUDER1);
  printer1->SetProbeSensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
  hotend_triggered_z[TOOLHEAD_3DP_EXTRUDER1] = probe_pt(RAW_X_POSITION(x), RAW_Y_POSITION(y), PROBE_PT_RAISE, 0, false);
  do_blocking_move_to_z(current_position[Z_AXIS] + 10, 10);

  endstops.enable_z_probe(false);

  return E_SUCCESS;
}

ErrCode BedLevelService::ConfirmExtruderTriggerStroke(uint8_t extruder_index) {
  float stroke_tmp = current_position[Z_AXIS] - hotend_triggered_z[extruder_index];
  LOG_I("extruder%d-stroke: %f\n", extruder_index, stroke_tmp);

  if (stroke_tmp > SWITCH_STROKE_EXTRUDER + SWITCH_STROKE_EXTRUDER_MAX_DEVIATION) {
    return E_FAILURE;
  }

  if (extruder_index == TOOLHEAD_3DP_EXTRUDER0) {
    switch_stroke_extruder0 = stroke_tmp;
    set_bed_leveling_enabled(true);
    settings.save();
    do_blocking_move_to_z(current_position[Z_AXIS]+20, 20);
  } else if (extruder_index == TOOLHEAD_3DP_EXTRUDER1) {
    switch_stroke_extruder1 = stroke_tmp;
    tool_change(TOOLHEAD_3DP_EXTRUDER0);
  }

  return E_SUCCESS;
}

ErrCode BedLevelService::DoXCalibration(xy_calibration_param_t &cal_param) {
  ErrCode err = E_SUCCESS;

  if (MODULE_TOOLHEAD_DUAL_EXTRUDER != ModuleBase::toolhead()) {
    return E_FAILURE;
  }

  thermalManager.setTargetHotend(cal_param.extruder0_temp, TOOLHEAD_3DP_EXTRUDER0);
  thermalManager.setTargetHotend(cal_param.extruder1_temp, TOOLHEAD_3DP_EXTRUDER1);
  thermalManager.setTargetBed(cal_param.bed_temp);
  process_cmd_imd("M106 P2 S255");

  if (!all_axes_homed()) {
    process_cmd_imd("G28");
  }

  relative_mode = false; //absolute positioning

  tool_change(TOOLHEAD_3DP_EXTRUDER0);

  thermalManager.wait_for_hotend(TOOLHEAD_3DP_EXTRUDER0, true);
  thermalManager.wait_for_hotend(TOOLHEAD_3DP_EXTRUDER1, true);
  thermalManager.wait_for_bed(true);

  feedrate_mm_s = MMM_TO_MMS(1080.0f);

  float destination_position_logic[X_TO_E];
  float start_point_A350[XYZ] = X_CALIBRATION_A350_START_POINT_XYZ;
  float start_point_A250[XYZ] = X_CALIBRATION_A250_START_POINT_XYZ;
  float start_point_A150[XYZ] = X_CALIBRATION_A150_START_POINT_XYZ;
  float start_point[XYZ];
  switch (linear_p->machine_size()) {
    case MACHINE_SIZE_A350:
      start_point[X_AXIS] = start_point_A350[X_AXIS];
      start_point[Y_AXIS] = start_point_A350[Y_AXIS];
      start_point[Z_AXIS] = cal_param.layer_height;
      break;
    case MACHINE_SIZE_A250:
      start_point[X_AXIS] = start_point_A250[X_AXIS];
      start_point[Y_AXIS] = start_point_A250[Y_AXIS];
      start_point[Z_AXIS] = cal_param.layer_height;
      break;
    case MACHINE_SIZE_A150:
      start_point[X_AXIS] = start_point_A150[X_AXIS];
      start_point[Y_AXIS] = start_point_A150[Y_AXIS];
      start_point[Z_AXIS] = cal_param.layer_height;
      break;
    default:
      return E_FAILURE;
      break;
  }

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS];
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
  destination_position_logic[Z_AXIS] = 30;
  destination_position_logic[B_AXIS] = current_position[B_AXIS];
  destination_position_logic[E_AXIS] = 0;
  do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 30);
  float saved_feedrate_mm_s;
  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[E_AXIS] = PRE_EXTRUSION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  saved_feedrate_mm_s = feedrate_mm_s;
  feedrate_mm_s = PRE_EXTRUSION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 30);

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS];
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  destination_position_logic[E_AXIS] = X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + X_CALIBRATION_UP_DOWN_LINE_LENGTH;
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  destination_position_logic[E_AXIS] = X_CALIBRATION_UP_DOWN_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  // retractiong
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  destination_position_logic[Z_AXIS] += Z_LIFT_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  // draw left nozzle middle line
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH/2 + LEFT_NOZZLE_MIDDLE_LINE_SHIFT;
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + X_CALIBRATION_UP_DOWN_LINE_LENGTH;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  destination_position_logic[E_AXIS] += E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = PRE_EXTRUSION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS];
  destination_position_logic[E_AXIS] = X_CALIBRATION_UP_DOWN_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  // retractiong
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  uint32_t i;
  float main_line_y_start_postion = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH/2 + LEFT_NOZZLE_MIDDLE_LINE_SHIFT + MAIN_SUB_SAFE_DISTANCE;
  for (i = 0; i < MAIN_SCALE_LINES; i++) {
    float print_line_length = 0.0;
    if (i == MAIN_SCALE_0_LINE_NUMBER) {
      print_line_length = SCALE_0_LINE_LENGTH;
    }
    else if (i%SCALE_LONGER_LINE_SEQUENCE == 0) {
      print_line_length = SCALE_LINE_LENGTH_LONGER;
    }
    else {
      print_line_length = SCALE_LINE_LENGHT_NORMAL;
    }

    destination_position_logic[X_AXIS] = start_point[X_AXIS] + FIRST_SCALE_LINE_TO_BORDER + i * MAIN_SCALE_LINE_INTERVAL;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
    destination_position_logic[Y_AXIS] = main_line_y_start_postion + print_line_length;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    destination_position_logic[E_AXIS] += E_RETRACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    feedrate_mm_s = PRE_EXTRUSION_SPEED;
    prepare_move_to_destination();
    feedrate_mm_s = saved_feedrate_mm_s;

    current_position[E_AXIS] = 0;
    sync_plan_position_e();
    destination_position_logic[Y_AXIS] = main_line_y_start_postion;
    destination_position_logic[E_AXIS] = print_line_length * cal_param.e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // retraction
    saved_feedrate_mm_s = feedrate_mm_s;
    destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    feedrate_mm_s = RETRACTION_SPEED;
    prepare_move_to_destination();
    feedrate_mm_s = saved_feedrate_mm_s;

    // move to middle
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH/2 - 1;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();
  }

  // retractiong
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= (E_RETRACTION_PAUSE_LENGTH-E_RETRACTION_LENGTH);
  destination_position_logic[Z_AXIS] += Z_LIFT_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  destination_position_logic[Z_AXIS] = 30;
  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 10);
  tool_change(TOOLHEAD_3DP_EXTRUDER1);

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[E_AXIS] = PRE_EXTRUSION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  saved_feedrate_mm_s = feedrate_mm_s;
  feedrate_mm_s = PRE_EXTRUSION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  destination_position_logic[X_AXIS] = start_point[X_AXIS];
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 10);

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + X_CALIBRATION_UP_DOWN_LINE_LENGTH;
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  destination_position_logic[E_AXIS] = X_CALIBRATION_UP_DOWN_LINE_LENGTH * cal_param.e_factor ;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + X_CALIBRATION_UP_DOWN_LINE_LENGTH;
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  destination_position_logic[E_AXIS] = X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  // retraction
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  destination_position_logic[Z_AXIS] += Z_LIFT_LENGTH;
  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 30);

  // draw right nozzle middle line
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH/2 - LEFT_NOZZLE_MIDDLE_LINE_SHIFT;
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + X_CALIBRATION_UP_DOWN_LINE_LENGTH;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  destination_position_logic[E_AXIS] += E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = PRE_EXTRUSION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS];
  destination_position_logic[E_AXIS] = X_CALIBRATION_UP_DOWN_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  // retractiong
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  float sub_line_y_star_position = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH/2 - RIGHT_NOZZLE_MIDDLE_LINE_SHIFT - MAIN_SUB_SAFE_DISTANCE;
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + FIRST_SCALE_LINE_TO_BORDER + MAIN_SCALE_0_LINE_NUMBER * MAIN_SCALE_LINE_INTERVAL - SUB_SCALE_0_LINE_NUMBER * SUB_SCALE_LINE_INTERVAL;
  float sub_line_x_start_postion = destination_position_logic[X_AXIS];
  destination_position_logic[Y_AXIS] = sub_line_y_star_position - SCALE_LINE_LENGTH_LONGER;
  get_destination_from_logic(destination_position_logic);
  saved_feedrate_mm_s = feedrate_mm_s;
  feedrate_mm_s = 80;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  for (i = 0; i < SUB_SCALE_LINES; i++) {
    float print_line_length = 0.0;
    if (i == SUB_SCALE_0_LINE_NUMBER) {
      print_line_length = SCALE_0_LINE_LENGTH;
    } else if (i%SCALE_LONGER_LINE_SEQUENCE == 0) {
      print_line_length = SCALE_LINE_LENGTH_LONGER;
    } else {
      print_line_length = SCALE_LINE_LENGHT_NORMAL;
    }

    destination_position_logic[X_AXIS] = sub_line_x_start_postion + i * SUB_SCALE_LINE_INTERVAL;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
    destination_position_logic[Y_AXIS] = sub_line_y_star_position - print_line_length;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    destination_position_logic[E_AXIS] += E_RETRACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    feedrate_mm_s = PRE_EXTRUSION_SPEED;
    prepare_move_to_destination();
    feedrate_mm_s = saved_feedrate_mm_s;

    current_position[E_AXIS] = 0;
    sync_plan_position_e();
    destination_position_logic[Y_AXIS] = sub_line_y_star_position;
    destination_position_logic[E_AXIS] = print_line_length * cal_param.e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // retraction
    saved_feedrate_mm_s = feedrate_mm_s;
    destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    feedrate_mm_s = RETRACTION_SPEED;
    prepare_move_to_destination();
    feedrate_mm_s = saved_feedrate_mm_s;

    // move to middle
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH/2 - RIGHT_NOZZLE_MIDDLE_LINE_SHIFT + 1;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();
  }

  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= (E_RETRACTION_PAUSE_LENGTH-E_RETRACTION_LENGTH);
  destination_position_logic[Z_AXIS] += Z_LIFT_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  do_blocking_move_to_logical_z(100, 40);
  do_blocking_move_to_xy(current_position[X_AXIS], 290, 50);

  return err;
}

ErrCode BedLevelService::ApplyXCalibration(float lines) {
  hotend_offset[X_AXIS][TOOLHEAD_3DP_EXTRUDER1] += lines * SCALE_MEASUREMENT_ACCURACY;
  settings.save();

  return E_SUCCESS;
}

ErrCode BedLevelService::DoYCalibration(xy_calibration_param_t &cal_param) {
  ErrCode err = E_SUCCESS;

  if (MODULE_TOOLHEAD_DUAL_EXTRUDER != ModuleBase::toolhead()){
    return E_FAILURE;
  }

  thermalManager.setTargetHotend(cal_param.extruder0_temp, TOOLHEAD_3DP_EXTRUDER0);
  thermalManager.setTargetHotend(cal_param.extruder1_temp, TOOLHEAD_3DP_EXTRUDER1);
  thermalManager.setTargetBed(cal_param.bed_temp);
  process_cmd_imd("M106 P2 S255");

  if (!all_axes_homed()) {
    process_cmd_imd("G28");
  }

  relative_mode = false; //absolute positioning

  tool_change(TOOLHEAD_3DP_EXTRUDER0);

  thermalManager.wait_for_hotend(TOOLHEAD_3DP_EXTRUDER0, true);
  thermalManager.wait_for_hotend(TOOLHEAD_3DP_EXTRUDER1, true);
  thermalManager.wait_for_bed(true);

  feedrate_mm_s = MMM_TO_MMS(1080.0f);

  float destination_position_logic[X_TO_E];
  float start_point_A350[XYZ] = Y_CALIBRATION_A350_START_POINT_XYZ;
  float start_point_A250[XYZ] = Y_CALIBRATION_A250_START_POINT_XYZ;
  float start_point_A150[XYZ] = Y_CALIBRATION_A150_START_POINT_XYZ;
  float start_point[XYZ];
  switch (linear_p->machine_size()) {
    case MACHINE_SIZE_A350:
      start_point[X_AXIS] = start_point_A350[X_AXIS];
      start_point[Y_AXIS] = start_point_A350[Y_AXIS];
      start_point[Z_AXIS] = cal_param.layer_height;
      break;
    case MACHINE_SIZE_A250:
      start_point[X_AXIS] = start_point_A250[X_AXIS];
      start_point[Y_AXIS] = start_point_A250[Y_AXIS];
      start_point[Z_AXIS] = cal_param.layer_height;
      break;
    case MACHINE_SIZE_A150:
      start_point[X_AXIS] = start_point_A150[X_AXIS];
      start_point[Y_AXIS] = start_point_A150[Y_AXIS];
      start_point[Z_AXIS] = cal_param.layer_height;
      break;
    default:
      return E_FAILURE;
      break;
  }

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS];
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
  destination_position_logic[Z_AXIS] = 30;
  destination_position_logic[B_AXIS] = current_position[B_AXIS];
  destination_position_logic[E_AXIS] = 0;
  do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 30);
  float saved_feedrate_mm_s;
  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[E_AXIS] = PRE_EXTRUSION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  saved_feedrate_mm_s = feedrate_mm_s;
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 30);

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS];
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  destination_position_logic[E_AXIS] = Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH;
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  destination_position_logic[E_AXIS] = Y_CALIBRATION_UP_DOWN_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  // retraction
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  destination_position_logic[Z_AXIS] += Z_LIFT_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  // draw left nozzle middle line
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH/2 - LEFT_NOZZLE_MIDDLE_LINE_SHIFT;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  destination_position_logic[E_AXIS] += E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = PRE_EXTRUSION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
  destination_position_logic[E_AXIS] = Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  // retractiong
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  uint32_t i;
  float main_line_x_start_position = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH/2 - LEFT_NOZZLE_MIDDLE_LINE_SHIFT - MAIN_SUB_SAFE_DISTANCE;
  for (i = 0; i < MAIN_SCALE_LINES; i++) {
    float print_line_length = 0.0;
    if (i == MAIN_SCALE_0_LINE_NUMBER) {
      print_line_length = SCALE_0_LINE_LENGTH;
    }
    else if (i%SCALE_LONGER_LINE_SEQUENCE == 0) {
      print_line_length = SCALE_LINE_LENGTH_LONGER;
    }
    else {
      print_line_length = SCALE_LINE_LENGHT_NORMAL;
    }

    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + FIRST_SCALE_LINE_TO_BORDER + i * MAIN_SCALE_LINE_INTERVAL;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
    destination_position_logic[X_AXIS] = main_line_x_start_position - print_line_length;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    destination_position_logic[E_AXIS] += E_RETRACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    feedrate_mm_s = RETRACTION_SPEED;
    prepare_move_to_destination();
    feedrate_mm_s = saved_feedrate_mm_s;

    current_position[E_AXIS] = 0;
    sync_plan_position_e();
    destination_position_logic[X_AXIS] = main_line_x_start_position;
    destination_position_logic[E_AXIS] = print_line_length * cal_param.e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // retraction
    saved_feedrate_mm_s = feedrate_mm_s;
    destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    feedrate_mm_s = RETRACTION_SPEED;
    prepare_move_to_destination();
    feedrate_mm_s = saved_feedrate_mm_s;

    // move to middle
    destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH/2 + 1;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();
  }

  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= (E_RETRACTION_PAUSE_LENGTH-E_RETRACTION_LENGTH);
  destination_position_logic[Z_AXIS] += Z_LIFT_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  destination_position_logic[Z_AXIS] = 30;
  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 10);
  tool_change(TOOLHEAD_3DP_EXTRUDER1);

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[E_AXIS] = PRE_EXTRUSION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  saved_feedrate_mm_s = feedrate_mm_s;
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  destination_position_logic[X_AXIS] = start_point[X_AXIS];
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

  do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 10);

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH;
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  destination_position_logic[E_AXIS] = Y_CALIBRATION_UP_DOWN_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH;
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  destination_position_logic[E_AXIS] = Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  // retration
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  do_blocking_move_to_logical_z(10, 10);

  // draw right nozzle middle line
  destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH/2 + RIGHT_NOZZLE_MIDDLE_LINE_SHIFT;
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  destination_position_logic[E_AXIS] += E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = PRE_EXTRUSION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  current_position[E_AXIS] = 0;
  sync_plan_position_e();
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
  destination_position_logic[E_AXIS] = Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * cal_param.e_factor;
  get_destination_from_logic(destination_position_logic);
  prepare_move_to_destination();

  // retractiong
  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  float sub_line_x_start_position = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH/2 + RIGHT_NOZZLE_MIDDLE_LINE_SHIFT + MAIN_SUB_SAFE_DISTANCE;

  destination_position_logic[X_AXIS] = sub_line_x_start_position + SCALE_LINE_LENGHT_NORMAL;
  destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + FIRST_SCALE_LINE_TO_BORDER + MAIN_SCALE_0_LINE_NUMBER * MAIN_SCALE_LINE_INTERVAL - SUB_SCALE_0_LINE_NUMBER*SUB_SCALE_LINE_INTERVAL;
  float sub_line_y_start_position = destination_position_logic[Y_AXIS];
  get_destination_from_logic(destination_position_logic);
  saved_feedrate_mm_s = feedrate_mm_s;
  feedrate_mm_s = 80;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  for (i = 0; i < SUB_SCALE_LINES; i++) {
    float print_line_length = 0.0;
    if (i == SUB_SCALE_0_LINE_NUMBER) {
      print_line_length = SCALE_0_LINE_LENGTH;
    } else if (i%SCALE_LONGER_LINE_SEQUENCE == 0) {
      print_line_length = SCALE_LINE_LENGTH_LONGER;
    } else {
      print_line_length = SCALE_LINE_LENGHT_NORMAL;
    }

    destination_position_logic[Y_AXIS] = sub_line_y_start_position + i * SUB_SCALE_LINE_INTERVAL;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
    destination_position_logic[X_AXIS] = sub_line_x_start_position + print_line_length;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    destination_position_logic[E_AXIS] += E_RETRACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    feedrate_mm_s = RETRACTION_SPEED;
    prepare_move_to_destination();
    feedrate_mm_s = saved_feedrate_mm_s;

    current_position[E_AXIS] = 0;
    sync_plan_position_e();
    destination_position_logic[X_AXIS] = sub_line_x_start_position;
    destination_position_logic[E_AXIS] = print_line_length * cal_param.e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // retration
    saved_feedrate_mm_s = feedrate_mm_s;
    destination_position_logic[E_AXIS] -= E_RETRACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    feedrate_mm_s = RETRACTION_SPEED;
    prepare_move_to_destination();
    feedrate_mm_s = saved_feedrate_mm_s;

    // move to middle
    destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH/2 - 1;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();
  }

  saved_feedrate_mm_s = feedrate_mm_s;
  destination_position_logic[E_AXIS] -= (E_RETRACTION_PAUSE_LENGTH-E_RETRACTION_LENGTH);
  destination_position_logic[Z_AXIS] += Z_LIFT_LENGTH;
  get_destination_from_logic(destination_position_logic);
  feedrate_mm_s = RETRACTION_SPEED;
  prepare_move_to_destination();
  feedrate_mm_s = saved_feedrate_mm_s;

  do_blocking_move_to_logical_z(100, 40);
  do_blocking_move_to_xy(current_position[X_AXIS], 290, 50);

  return err;
}

ErrCode BedLevelService::ApplyYCalibration(float lines) {
  hotend_offset[Y_AXIS][TOOLHEAD_3DP_EXTRUDER1] += lines * SCALE_MEASUREMENT_ACCURACY;
  settings.save();

  return E_SUCCESS;
}

ErrCode BedLevelService::DoAutoLeveling(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t grid = 3;
  char cmd[16];

  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  LOG_I("SC req auto level\n");

  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead() && MODULE_TOOLHEAD_DUAL_EXTRUDER != ModuleBase::toolhead()) {
    goto OUT;
  }

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

  // MUST clear live z offset before G28
  // otherwise will apply live z after homing
  live_z_offset_[0] = 0;
  live_z_offset_[1] = 0;

  process_cmd_imd("G28");
  planner.synchronize();
  tool_change(TOOLHEAD_3DP_EXTRUDER0);

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
  do_blocking_move_to_xy(_GET_MESH_X(0) - (xprobe_offset_from_extruder),
                                  _GET_MESH_Y(0) - (yprobe_offset_from_extruder),
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

  if (err != E_SUCCESS) {
    goto OUT;
  }

  if (MODULE_TOOLHEAD_DUAL_EXTRUDER == ModuleBase::toolhead()) {
    level_mode_ = LEVEL_MODE_AUTO_NO_ADJUST;
    return SaveAndExitLeveling(event);
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

  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead() && MODULE_TOOLHEAD_DUAL_EXTRUDER != ModuleBase::toolhead()) {
    goto OUT;
  }

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

  do_blocking_move_to_z(z_position_before_calibration, speed_in_calibration[Z_AXIS]);

  // increase 3mm for first leveling point
  // to avoid nozzle gouging the surface when user place glass on the steel sheet
  if (MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) {
    do_blocking_move_to_z(12, 10);
  }

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
      if ((manual_level_index_ != index -1) && current_position[Z_AXIS] < z_position_before_calibration) {
        do_blocking_move_to_z(current_position[Z_AXIS] + 3, speed_in_calibration[Z_AXIS]);
      }
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

  LOG_I("SC req save data of leveling\n");

  planner.synchronize();

  if (level_mode_ == LEVEL_MODE_MANUAL && manual_level_index_ <= GRID_MAX_POINTS_INDEX) {
    if (MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) {
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
    } else if (MODULE_TOOLHEAD_DUAL_EXTRUDER == ModuleBase::toolhead()) {
      if (event.data[0] == 0) {
        MeshPointZ[manual_level_index_] = current_position[Z_AXIS];
        extruder0_manual_level_z_ = current_position[Z_AXIS];
        LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", manual_level_index_,
            current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
        for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
          for (i = 0; i < GRID_MAX_POINTS_X; i++) {
            z_values[i][j] = MeshPointZ[j * GRID_MAX_POINTS_X + i];
          }
        }

        bed_level_virt_interpolate();
        settings.save();
        do_blocking_move_to_z(current_position[Z_AXIS]+3, 10);
        hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = 0;
        tool_change(TOOLHEAD_3DP_EXTRUDER1);
        goto EXIT;
      } else if (event.data[0] == 1) {
        extruder1_manual_level_z_ = current_position[Z_AXIS];
        hotend_offset_z_temp = extruder0_manual_level_z_ - extruder1_manual_level_z_;
        settings.save();
      }
    }
  }
  else if (level_mode_ == LEVEL_MODE_AUTO) {
    process_cmd_imd("G1029 S0");
  }
  else if (level_mode_ == LEVEL_MODE_AUTO_NO_ADJUST) {
    nozzle_height_probed = 1;
    process_cmd_imd("G1029 S1");
  }
  else {
    LOG_E("didn't start leveling!\n");
    err = E_FAILURE;
    goto EXIT;
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

  if (MODULE_TOOLHEAD_DUAL_EXTRUDER == ModuleBase::toolhead()) {
    hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = 0;
    tool_change(TOOLHEAD_3DP_EXTRUDER0);
    hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = hotend_offset_z_temp;
  }

  // make sure we are in absolute mode
  relative_mode = false;

  // clear flag
  level_mode_ = LEVEL_MODE_INVALD;

EXIT:
  event.data = &err;
  event.length = 1;
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

  // make sure we are in absolute mode
  relative_mode = false;

  return hmi.Send(event);
}

ErrCode BedLevelService::IsLeveled(SSTP_Event_t &event) {
  uint8_t level_status = z_values[0][0] != DEFAUT_LEVELING_HEIGHT;;

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


ErrCode BedLevelService::UpdateLiveZOffset(float offset, uint8_t e) {
  if (e >= EXTRUDERS) {
    return E_PARAM;
  }

  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead() && MODULE_TOOLHEAD_DUAL_EXTRUDER != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return E_FAILURE;
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
  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead() && MODULE_TOOLHEAD_DUAL_EXTRUDER != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return;
  }

  if ((MODULE_TOOLHEAD_DUAL_EXTRUDER == ModuleBase::toolhead()) && (e != active_extruder)) {
    return;
  }

  planner.synchronize();
  float cur_z = current_position[Z_AXIS];

  do_blocking_move_to_z(current_position[Z_AXIS] + live_z_offset_[e], 5);
  planner.synchronize();

  current_position[Z_AXIS] = cur_z;
  sync_plan_position();

  LOG_I("Apply Z offset: %.2f\n", live_z_offset_[e]);
}


void BedLevelService::UnapplyLiveZOffset(uint8_t e) {
  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead() && MODULE_TOOLHEAD_DUAL_EXTRUDER != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return;
  }

  if ((MODULE_TOOLHEAD_DUAL_EXTRUDER == ModuleBase::toolhead()) && (e != active_extruder)) {
    return;
  }

  planner.synchronize();
  float cur_z = current_position[Z_AXIS];

  do_blocking_move_to_z(current_position[Z_AXIS] - live_z_offset_[e], 5);
  planner.synchronize();

  current_position[Z_AXIS] = cur_z;
  sync_plan_position();

  LOG_I("Unapply Z offset: %.2f\n", live_z_offset_[e]);
}


void BedLevelService::SaveLiveZOffset() {
  if (live_z_offset_updated_) {
    live_z_offset_updated_ = false;
    settings.save();
  }
}
