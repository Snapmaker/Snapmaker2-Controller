/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2023 Snapmaker [https://github.com/Snapmaker]
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
#include "../common/debug.h"
#include "../service/system.h"
#include "../module/toolhead_cnc.h"
#include "../module/toolhead_laser.h"

#include  "src/gcode/gcode.h"

#if (MOTHERBOARD == BOARD_SNAPMAKER_2_0)

void GcodeSuite::M7_M8(const bool is_M8) {
  if(MODULE_TOOLHEAD_LASER_20W == ModuleBase::toolhead() || MODULE_TOOLHEAD_LASER_40W == ModuleBase::toolhead()) {
    planner.synchronize();   // Wait for move to arrive
    laser->SetAirPumpSwitch(true);
  }
}

void GcodeSuite::M9(void) {
  if(MODULE_TOOLHEAD_LASER_20W == ModuleBase::toolhead() || MODULE_TOOLHEAD_LASER_40W == ModuleBase::toolhead()) {
    planner.synchronize();   // Wait for move to arrive
    laser->SetAirPumpSwitch(false);
  }
}

#endif // MOTHERBOARD == BOARD_SNAPMAKER_2_0