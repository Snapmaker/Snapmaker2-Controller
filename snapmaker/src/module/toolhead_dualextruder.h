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
#ifndef SNAPMAKER_TOOLHEAD_DUAL_EXTRUDER_H_
#define SNAPMAKER_TOOLHEAD_DUAL_EXTRUDER_H_

#include "module_base.h"
#include "toolhead_3dp.h"
#include "can_host.h"
#include "../common/config.h"

class ToolHeadDualExtruder: public ToolHead3DP {
  public:
    ToolHeadDualExtruder(ModuleDeviceID id): ToolHead3DP(id) {
      // do nothing
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);
};

extern ToolHeadDualExtruder printer_dualextruder;

#endif
