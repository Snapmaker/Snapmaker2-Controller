/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 *
 */

#include <src/gcode/gcode.h>
#include <src/feature/bedlevel/abl/abl.h>
#include <src/module/planner.h>
#include <src/module/temperature.h>
#include <src/feature/bedlevel/bedlevel.h>
#include <src/module/endstops.h>
#include <src/module/configuration_store.h>
/**
 * G29.cpp - Auto Bed Leveling
 */



extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;
extern uint32_t ABL_GRID_POINTS_VIRT_X;
extern uint32_t ABL_GRID_POINTS_VIRT_Y;
extern uint32_t ABL_TEMP_POINTS_X;
extern uint32_t ABL_TEMP_POINTS_Y;

/**
 * G1029
 *
 * GCode implementation of SM2 auto bed leveling protocol.
 *
 *  P[Size]
 *              set the size of GRID
 *
 *  A
 *              start auto probing
 *
 *  S
 *              tuning and saving the offset
 *              Will move to center point first
 *
 *  M[margin]
 *              Set margin
 *
 *  W
 *      I[X_index]
 *      J[Y_index]
 *
 *              Move to the MESH(i), MESH(j)
 *              1. After G28, enable bed leveling feature,
 *                  We can use this utility to quickly, verify specific location
 *              2. TODO, implement set z value to allow manual probing.
 *  D[delta]
 *
 *          Z axis, move z-offset
 *          delta > 0 => we raise the reference point
 */
void GcodeSuite::G1029() {
  const bool seen_p = parser.seenval('P');
  if (seen_p) {
    int size = parser.value_int();
    if (size < 0 || size > GRID_MAX_NUM) {
      SERIAL_ECHOLNPAIR("Invalid grid size , maximum: ", GRID_MAX_NUM);
      return;
    }

    GRID_MAX_POINTS_X = size;
    GRID_MAX_POINTS_Y = size;

    ABL_GRID_POINTS_VIRT_X = (GRID_MAX_POINTS_X - 1) * (BILINEAR_SUBDIVISIONS) + 1;
    ABL_GRID_POINTS_VIRT_Y = (GRID_MAX_POINTS_Y - 1) * (BILINEAR_SUBDIVISIONS) + 1;
    ABL_TEMP_POINTS_X = (GRID_MAX_POINTS_X + 2);
    ABL_TEMP_POINTS_Y = (GRID_MAX_POINTS_Y + 2);

    SERIAL_ECHOLNPAIR("Set grid size : ", size);
    return;
  }

  const bool seen_a = parser.seen("A");
  if (seen_a) {

    thermalManager.disable_all_heaters();
    process_cmd_imd("G28");
    set_bed_leveling_enabled(false);

    // Set the Z max feedrate to 50mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = 60;

    endstops.enable_z_probe(true);
    auto_probing(false);
    endstops.enable_z_probe(false);

    // Recover the Z max feedrate to 20mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = 30;
    return;
  }

  const bool seen_s = parser.seen("S");
  if (seen_s) {
    compensate_offset();
    do_blocking_move_to_z(15, 50);
    bed_level_virt_interpolate();
    settings.save();
    return;
  }

  const bool seen_m = parser.seenval('M');
  if (seen_m) {
    // set probe margin
    int margin = parser.value_int();
    PROBE_MARGIN = margin;
    SERIAL_ECHOLNPAIR("Set probe margin : ", margin);
  }

  const bool seen_w = parser.seen('W');
  if (seen_w) {
    uint8_t  i = parser.byteval('I', GRID_MAX_POINTS_X / 2);
    uint8_t  j = parser.byteval('J', GRID_MAX_POINTS_Y / 2);

    do_blocking_move_to_xy(_GET_MESH_X(i), _GET_MESH_Y(j), 50);
    do_blocking_move_to_z(7, 50);
    return;
  }

  const bool seen_d = parser.seenval('D');
  if (seen_d) {
    float delta = -parser.value_float();
    if (delta > 1) {
      SERIAL_ECHOLNPAIR("Error, it should be less than 1mm", delta);
    } else {
      set_bed_leveling_enabled(false);
      sync_plan_position();

      compensate_offset(delta);
      bed_level_virt_interpolate();
      
      set_bed_leveling_enabled(true);
    }
    return;
  }
}
