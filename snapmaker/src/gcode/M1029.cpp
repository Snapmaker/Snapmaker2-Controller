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

// marlin headers
#include "src/gcode/gcode.h"
#include "src/gcode/queue.h"
#include "src/module/stepper.h"
/*
* Remap the 8 pin port to any axis
* 8 pin number: 1 - 6
* eg. M1029 X2 Y3 Z4 B5 E6
*/


void GcodeSuite::M1029() {
  LOOP_X_TO_E(i) {
    if (parser.seenval(axis_codes[i])) {
      uint8_t value = parser.byteval(axis_codes[i], 0) - 1;
      stepper.StepperBind8PinPort(i, value);
      stepper.StepperPinRemap();
    }
  }
  stepper.PrintStepperBind();
}
