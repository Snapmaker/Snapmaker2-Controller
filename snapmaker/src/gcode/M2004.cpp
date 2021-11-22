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

#include "snapmaker.h"
#include "service/system.h"
#include "src/gcode/gcode.h"
#include "../module/toolhead_3dp.h"

void GcodeSuite::M2004() {
  if (ModuleBase::toolhead() != MODULE_TOOLHEAD_DUAL_EXTRUDER) {
    return;
  }

  const bool seen_x = parser.seenval('X');
  const bool seen_y = parser.seenval('Y');
  if (seen_x && seen_y) {
    uint8_t buf[2];
    buf[0] = (uint8_t)parser.byteval('X', (uint8_t)100);
    buf[1] = (uint8_t)parser.byteval('Y', (uint8_t)100);

    SSTP_Event_t event;
    event.data = buf;
    event.length = 2;
    event.id = EID_SETTING_REQ;
    event.op_code = SETTINGS_OPC_START_STROKE_CALIBRATION;
    printer1->ExtruderStrokeCalibration(event);

    return;
  }

  const bool seen_e = parser.seenval('E');
  if (seen_e) {
    uint8_t buf[1];
    buf[0] = (uint8_t)parser.byteval('E', (uint8_t)0);
    SSTP_Event_t event;
    event.data = buf;
    event.length = 1;
    event.id = EID_SETTING_REQ;
    event.op_code = SETTINGS_OPC_STROKE_CONFIRM;
    printer1->ExtruderStrokeConfirm(event);

    return;
  }
}
