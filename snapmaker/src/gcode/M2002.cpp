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
#include "../module/toolhead_laser.h"

void GcodeSuite::M2002() {
  const bool seen_s = parser.seenval('S');
  if (seen_s) {
    SSTP_Event_t event;
    event.op_code = 2;
    event.data = NULL;
    event.length = 0;
    event.id = 9;
    laser->GetSecurityStatus(event);
  }

  const bool seen_l = parser.seenval('L');
  if (seen_l) {
    uint8_t state = (uint8_t)parser.byteval('L', (uint8_t)0);
    SSTP_Event_t event;
    event.op_code = 2;
    event.data = &state;
    event.length = 1;
    event.id = 9;

    laser->SetAutoFocusLight(event);
  }

  const bool seen_y = parser.seenval('Y');
  if (seen_y) {
    uint32_t id = (uint32_t)parser.byteval('Y', (uint32_t)0);
    SSTP_Event_t event;
    event.op_code = 2;
    event.data = (uint8_t *)&id;
    event.length = 4;
    event.id = 9;

    laser->SetOnlineSyncId(event);
  }

  const bool seen_g = parser.seenval('G');
  if (seen_g) {
    SSTP_Event_t event;
    event.op_code = 2;
    event.data = NULL;
    event.length = 0;
    event.id = 9;

    laser->GetOnlineSyncId(event);
  }

  const bool seen_t = parser.seenval('T');
  if (seen_t) {
    int8_t temp = (int8_t)parser.byteval('T', (int8_t)55);
    SSTP_Event_t event;
    event.op_code = 0x16;
    event.data = (uint8_t *)&temp;
    event.length = 1;
    event.id = 9;

    laser->SetProtectTemp(event);
  }
}
