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

void GcodeSuite::M2029() {
  if (ModuleBase::toolhead() != MODULE_TOOLHEAD_DUAL_EXTRUDER) {
    return;
  }

  const bool seen_t = parser.seenval('T');
  const bool seen_z = parser.seenval('Z');
  if (seen_t && seen_z) {
    uint8_t extruder_index = (uint8_t)parser.byteval('T', (uint8_t)0);
    int32_t live_z_offset = (int32_t)parser.intval('Z', (int32_t)0);
    if (extruder_index >= EXTRUDERS) {
      return;
    }

    uint8_t buf[5];
    if (extruder_index == 0) {
      buf[0] = RENV_TYPE_ZOFFSET;
    } else if (extruder_index == 1) {
      buf[0] = RENV_TYPE_EXTRUDER1_ZOFFSET;
    }

    buf[1] = (live_z_offset >> 24) & 0xff;
    buf[2] = (live_z_offset >> 16) & 0xff;
    buf[3] = (live_z_offset >> 8) & 0xff;
    buf[4] = live_z_offset & 0xff;
    SSTP_Event_t event;
    event.data = buf;
    event.length = 5;
    event.id = EID_SETTING_REQ;
    event.op_code = SETTINGS_OPC_SET_RUNTIME_ENV;
    systemservice.ChangeRuntimeEnv(event);
    return;
  }
}
