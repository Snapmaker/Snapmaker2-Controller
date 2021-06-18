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

#include "../common/config.h"

#include "../module/purifier.h"

// marlin headers
#include "src/gcode/gcode.h"
#include "src/gcode/queue.h"
#include "src/core/macros.h"
/*
* purifier control and data capture
* F[0-3]: set fan gears
* D[0-65535]: fan delay close time. unit S
* P[0-255]: set fan power(debug)
* R-G-B-[0-255]: set light color. eg. R100 G20 B30 (debug)
*/


void GcodeSuite::M1011() {
  uint8_t f, rgb[3]={0}, rgb_flag=0, forced=0;
  uint16_t delay_close_s = 0;
  if (!purifier.IsOnline()) {
    SERIAL_ECHOLN("Purifier is not exist.");
    return;
  }

  if (parser.seen('F')) {
    f = parser.byteval('F', 0);
    if (parser.seen('S')){
      forced = parser.byteval('S', 0);
    }
    if (parser.seen('D')){
      delay_close_s = parser.intval('D', 0);
    }
    if (f <= PURIFIER_FAN_GEARS_3) {
      purifier.SetFanGears(f);
      purifier.SetFanStatus(!!f, delay_close_s, forced);
    } else {
      SERIAL_ECHOLN("Set purifier Fan err, 0 - 3");
    }
    return;
  }

  if (parser.seen('P')) {
    f = parser.byteval('P', 0);
    purifier.SetFanPower(f);
    SERIAL_ECHOLN("Set purifier Fan power");
    return;
  }

  if (parser.seen('W')) {
    purifier.GetInfo(PURIFIER_INFO_ALL, 500);
    SERIAL_ECHOLN("Set purifier Fan power");
    return;
  }

  if (parser.seen('R')) {
    rgb[0] = parser.byteval('R', 0);
    rgb_flag = 1;
  }
  if (parser.seen('G')) {
    rgb[1] = parser.byteval('G', 0);
    rgb_flag = 1;
  }
  if (parser.seen('B')) {
    rgb[2] = parser.byteval('B', 0);
    rgb_flag = 1;
  }
  if (rgb_flag) {
    purifier.SetLightColor(rgb);
    SERIAL_ECHOLN("Set purifier light");
    return;
  }
  
  purifier.GetInfo(PURIFIER_INFO_ALL, 500);
  purifier.DisplayInfo();
}
