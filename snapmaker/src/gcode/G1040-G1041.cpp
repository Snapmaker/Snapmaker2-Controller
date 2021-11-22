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
#include "../service/system.h"

// marlin headers
#include  "src/gcode/gcode.h"

#include "../common/protocol_sstp.h"
#include "../service/bed_level.h"
#include "../common/debug.h"
#include "../module/toolhead_3dp.h"
#include "../../../Marlin/src/module/configuration_store.h"

#if (MOTHERBOARD == BOARD_SNAPMAKER_2_0)

void GcodeSuite::G1040() {
  planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

  bool seen_l, seen_r, seen_b, seen_h, seen_e, seen_a;  // performance, cross, alignment

  seen_l = parser.seen("L");
  seen_r = parser.seen("R");
  seen_h = parser.seen("H");
  seen_e = parser.seen("E");
  seen_b = parser.seen("B");
  if (seen_l && seen_r && seen_b && seen_e && seen_h) {
    xy_calibration_param_t xy_cal_param;
    xy_cal_param.extruder0_temp = (int16_t)parser.ushortval('L', (int16_t)205);
    xy_cal_param.extruder1_temp = (int16_t)parser.ushortval('R', (int16_t)205);
    xy_cal_param.bed_temp = (uint8_t)parser.byteval('B', (uint8_t)50);
    xy_cal_param.layer_height = (float)parser.floatval('H', (float)0.2);
    xy_cal_param.e_factor = (float)parser.floatval('E', (float)E_MOVES_FACTOR);
    levelservice.DoXCalibration(xy_cal_param);
    goto EXIT;
  }

  seen_a = parser.seen("A");
  if (seen_a) {
    float subline = (float)parser.floatval('A', (float)0.0);   // sub alignment line number
    levelservice.ApplyXCalibration(subline);
    goto EXIT;
  }

EXIT:
  return;
}

void GcodeSuite::G1041() {
  planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

  bool seen_l, seen_r, seen_b, seen_h, seen_e, seen_a;  // performance, cross, alignment

  seen_l = parser.seen("L");
  seen_r = parser.seen("R");
  seen_h = parser.seen("H");
  seen_e = parser.seen("E");
  seen_b = parser.seen("B");
  if (seen_l && seen_r && seen_b && seen_e && seen_h) {
    xy_calibration_param_t xy_cal_param;
    xy_cal_param.extruder0_temp = (int16_t)parser.ushortval('L', (int16_t)205);
    xy_cal_param.extruder1_temp = (int16_t)parser.ushortval('R', (int16_t)205);
    xy_cal_param.layer_height = (float)parser.floatval('H', (float)0.2);
    xy_cal_param.bed_temp = (uint8_t)parser.byteval('B', (uint8_t)50);
    xy_cal_param.e_factor = (float)parser.floatval('E', (float)E_MOVES_FACTOR);
    levelservice.DoYCalibration(xy_cal_param);
    goto EXIT;
  }

  seen_a = parser.seen("A");
  if (seen_a) {
    float subline = (float)parser.floatval('A', (float)0.0);   // sub alignment line number
    levelservice.ApplyYCalibration(subline);
    goto EXIT;
  }

EXIT:
  return;
}

#endif // MOTHERBOARD == BOARD_SNAPMAKER_2_0