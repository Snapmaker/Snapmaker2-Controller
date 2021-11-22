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

void GcodeSuite::M2003() {
  if (ModuleBase::toolhead() != MODULE_TOOLHEAD_DUAL_EXTRUDER) {
    return;
  }

  const bool seen_b = parser.seenval('B');
  if (seen_b) {
    uint8_t stopper = (uint8_t)parser.byteval('B', (uint8_t)0);
    if (stopper > 1) return;

    uint8_t action = 0;
    SSTP_Event_t event = {EID_MOTION_REQ};
    event.data = &action;
    event.length = 1;
    if (stopper == 0) {
      event.op_code = MOTION_OPC_LEFT_STOPPER_ASSEMBLE;
      printer1->LeftStopperAssemble(event);
    } else if (stopper == 1) {
      event.op_code = MOTION_OPC_RIGHT_STOPPER_ASSEMBLE;
      printer1->RightStopperAssemble(event);
    }
  }

  const bool seen_m = parser.seenval('M');
  if (seen_m) {
    uint8_t stopper = (uint8_t)parser.byteval('M', (uint8_t)0);
    if (stopper > 1) return;

    uint8_t action = 1;
    SSTP_Event_t event = {EID_MOTION_REQ};
    event.data = &action;
    event.length = 1;
    if (stopper == 0) {
      event.op_code = MOTION_OPC_LEFT_STOPPER_ASSEMBLE;
      printer1->LeftStopperAssemble(event);
    } else if (stopper == 1) {
      event.op_code = MOTION_OPC_RIGHT_STOPPER_ASSEMBLE;
      printer1->RightStopperAssemble(event);
    }
  }
}
