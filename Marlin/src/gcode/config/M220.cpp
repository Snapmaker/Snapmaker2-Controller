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
#include "../../../../snapmaker/src/module/toolhead_3dp.h"

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
void GcodeSuite::M220() {
  int16_t percentage = 100;
  if (MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) {
    if (parser.seenval('S')) {
      percentage = parser.value_int();
    }
    if (percentage > 500 || percentage < 0) {
      return;
    }
    feedrate_percentage = percentage;
  }

  if (MODULE_TOOLHEAD_DUAL_EXTRUDER == ModuleBase::toolhead()) {
    uint8_t e = 0;
    bool seen_t = parser.seenval('T');
    if (seen_t) {
      e = (uint8_t)parser.byteval('T', (uint8_t)0);
      if (e >= EXTRUDERS) {
        return;
      }
    }

    bool seen_s = parser.seenval('S');
    if (seen_s) {
      percentage = (uint8_t)parser.intval('S', (int16_t)100);
      if (percentage > 500 || percentage < 0) {
        return;
      }
    }
    extruders_feedrate_percentage[e] = percentage;

    if (active_extruder == e) {
      feedrate_percentage = percentage;
    }
  }

}
