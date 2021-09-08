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

#include "../common/debug.h"
#include "../common/config.h"

#include "../service/system.h"

#include "src/gcode/gcode.h"
#include "src/gcode/queue.h"
#include "src/core/macros.h"

#if HAS_POSITION_SHIFT
  // The distance that XYZ has been offset by G92. Reset by G28.
  extern float position_shift[XN];
#endif
#if HAS_HOME_OFFSET
  // This offset is added to the configured home position.
  // Set by M206, M428, or menu item. Saved to EEPROM.
  extern float home_offset[XN];
#endif
#if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
  // The above two are combined to save on computes
  extern float workspace_offset[XN];
#endif

extern float current_position[X_TO_E];

void GcodeSuite::M2000() {
  uint8_t l;
  uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0);

  switch (s) {
  case 0:
    SNAP_DEBUG_SHOW_INFO();
    SERIAL_ECHOPAIR("position_shift:\n");
    SERIAL_ECHOPAIR("X: ", position_shift[X_AXIS], ", Y: ", position_shift[Y_AXIS], ", Z: ", position_shift[Z_AXIS], ", B: ", position_shift[B_AXIS], "\n");
    SERIAL_ECHOPAIR("home_offset:\n");
    SERIAL_ECHOPAIR("X: ", home_offset[X_AXIS], ", Y: ", home_offset[Y_AXIS], ", Z: ", home_offset[Z_AXIS], ", B: ", home_offset[B_AXIS], "\n");
    SERIAL_ECHOPAIR("workspace_offset:\n");
    SERIAL_ECHOPAIR("X: ", workspace_offset[X_AXIS], ", Y: ", workspace_offset[Y_AXIS], ", Z: ", workspace_offset[Z_AXIS], ", B: ", workspace_offset[B_AXIS], "\n");
    SERIAL_ECHOPAIR("cur position:\n");
    SERIAL_ECHOPAIR("X: ", current_position[X_AXIS], ", Y: ", current_position[Y_AXIS], ", Z: ", current_position[Z_AXIS], ", B: ", current_position[B_AXIS], "\n");
    break;

  case 1:
    // set PC log level
    l = (uint8_t)parser.byteval('L', (uint8_t)10);
    if (!WITHIN(l, 0, (int)SNAP_DEBUG_LEVEL_MAX)) {
      LOG_E("L out of range (0-%d)\n", (int)SNAP_DEBUG_LEVEL_MAX);
      return;
    }
    SNAP_DEBUG_SET_LEVEL(0, (SnapDebugLevel)l);
    break;

  case 2:
    // set SC log level
    l = (uint8_t)parser.byteval('L', (uint8_t)10);
    if (!WITHIN(l, 0, (int)SNAP_DEBUG_LEVEL_MAX)) {
      LOG_E("L out of range (0-%d)\n", (int)SNAP_DEBUG_LEVEL_MAX);
      return;
    }
    SNAP_DEBUG_SET_LEVEL(1, (SnapDebugLevel)l);
    break;

  case 3:
    SNAP_DEBUG_SHOW_EXCEPTION();
    break;

  case 4:
    l = (uint8_t)parser.byteval('L', (uint8_t)0);
    if (!WITHIN(l, 1, 32)) {
      LOG_E("L is out of range (1-32)\n");
      return;
    }
    systemservice.ClearExceptionByFaultFlag(1<<(l-1));
    break;
  }

}