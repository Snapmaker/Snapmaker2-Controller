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
#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../module/configuration_store.h"

#if ENABLED(I2C_POSITION_ENCODERS)
  #include "../../feature/I2CPositionEncoder.h"
#endif

/**
 * G92: Set current position to given X Y Z E
 */
void GcodeSuite::G92() {

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    switch (parser.subcode) {
      case 1:
        // Zero the G92 values and restore current position
        #if !IS_SCARA
          LOOP_XN(i) {
            const float v = position_shift[i];
            if (v) {
              position_shift[i] = 0;
              update_workspace_offset((AxisEnum)i);
            }
          }
        #endif // Not SCARA
        return;
    }
  #endif

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    #define IS_G92_0 (parser.subcode == 0)
  #else
    #define IS_G92_0 true
  #endif

  bool didE = false;
  #if IS_SCARA || !HAS_POSITION_SHIFT
    bool didXYZ = false;
  #else
    bool didXYZ = false;
  #endif

  bool onlyE = true;
  if (IS_G92_0) LOOP_X_TO_E(i) {
    if (parser.seenval(axis_codes[i])) {
      const float l = parser.value_axis_units((AxisEnum)i),
                  v = (i == E_AXIS || i == B_AXIS) ? l : LOGICAL_TO_NATIVE(l, i),
                  d = v - current_position[i];
      if (!NEAR_ZERO(d)) {
        #if IS_SCARA || !HAS_POSITION_SHIFT
          if (i == E_AXIS) didE = true; else didXYZ = true;
          current_position[i] = v;        // Without workspaces revert to Marlin 1.0 behavior
        #elif HAS_POSITION_SHIFT
          if (i == E_AXIS) {
            didE = true;
            current_position[E_AXIS] = v; // When using coordinate spaces, only E is set directly
          }
          else {
            onlyE = false;
            if (i == B_AXIS) {
              // Special handling!!!
              // Set the B coordinate directly and use G92 to solve the problem of overflow of
              // the B-axis unidirectional rotation position
              didXYZ = true;
              position_shift[i] = -home_offset[i];
              current_position[i] = v;
            }
            else {
              position_shift[i] += d;       // Other axes simply offset the coordinate space
            }
            update_workspace_offset((AxisEnum)i);
          }
        #endif
      }
    }
  }

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    // will not save coordinate when set only E
    // to reduce rw times to flash
    if (!onlyE) {
      // Apply workspace offset to the active coordinate system
      if (WITHIN(active_coordinate_system, 0, MAX_COORDINATE_SYSTEMS - 1)) {
        COPY(coordinate_system[active_coordinate_system], position_shift);
        settings.save();
      }
    }
  #endif

  if    (didXYZ) sync_plan_position();
  else if (didE) sync_plan_position_e();

  report_current_position();
}
