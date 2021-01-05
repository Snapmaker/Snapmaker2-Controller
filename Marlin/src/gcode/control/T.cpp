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

#include "../gcode.h"
#include "../../module/tool_change.h"

#if ENABLED(DEBUG_LEVELING_FEATURE) || EXTRUDERS > 1
  #include "../../module/motion.h"
#endif

#if ENABLED(PRUSA_MMU2)
  #include "../../feature/prusa_MMU2/mmu2.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

#if (MOTHERBOARD == BOARD_SNAPMAKER_2_0)
  #include "../snapmaker/src/module/toolhead_3dp.h"
#endif

/**
 * T0-T<n>: Switch tool, usually switching extruders
 *
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 *
 * For PRUSA_MMU2:
 *   T[n] Gcode to extrude at least 38.10 mm at feedrate 19.02 mm/s must follow immediately to load to extruder wheels.
 *   T?   Gcode to extrude shouldn't have to follow. Load to extruder wheels is done automatically.
 *   Tx   Same as T?, but nozzle doesn't have to be preheated. Tc requires a preheated nozzle to finish filament load.
 *   Tc   Load to nozzle after filament was prepared by Tc and nozzle is already heated.
 */
void GcodeSuite::T(const uint8_t tool_index) {

  if (DEBUGGING(LEVELING)) {
    DEBUG_ECHOLNPAIR(">>> T(", tool_index, ")");
    DEBUG_POS("BEFORE", current_position);
  }

  #if ENABLED(PRUSA_MMU2)
    if (parser.string_arg) {
      mmu2.toolChange(parser.string_arg);   // Special commands T?/Tx/Tc
      return;
    }
  #endif

  #if EXTRUDERS < 2

    tool_change(tool_index);

  #else
    bool seen_m = parser.seen("M");
    bool seen_n = parser.seen("N");
    bool seen_l = parser.seen("L");
    bool seen_r = parser.seen("R");

    if (seen_m) {
      nozzle0_motor_runtime = (uint16_t)parser.ushortval('M', (uint16_t)0);
    }

    if (seen_n) {
      nozzle1_motor_runtime = (uint16_t)parser.ushortval('N', (uint16_t)0);
    }

    if (seen_m || seen_n) {
      tool_change_motor(tool_index, MMM_TO_MMS(parser.linearval('F')), (tool_index == active_extruder) || parser.boolval('S'));
      goto EXIT;
    }

    if (seen_l) {
      lift_switch_left_position = (float)parser.floatval('L', (float)0);
    }

    if (seen_r) {
      lift_switch_right_position = (float)parser.floatval('R', (float)0);
    }

    tool_change(
      tool_index,
      MMM_TO_MMS(parser.linearval('F')),
      (tool_index == active_extruder) || parser.boolval('S')
    );

EXIT:
    active_extruder = tool_index;
  #endif

  if (DEBUGGING(LEVELING)) {
    DEBUG_POS("AFTER", current_position);
    DEBUG_ECHOLNPGM("<<< T()");
  }
}
