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

#include "../module/enclosure.h"

// marlin headers
#include "src/gcode/gcode.h"
#include "src/gcode/queue.h"
#include "src/core/macros.h"

/*
* Disable /Enable chamber door event
* S0: disable door event
* S1: enable door event
* S2: release hold of UART for receiving other PC gcodes
* S3: set light power, P 0-100%, eg.M1010 S3 P100
* S4: set fan power, P 0-100%, eg.M1010 S4 P100
*/


void GcodeSuite::M1010() {
  uint8_t s, p;

  if (!parser.seen('S')) {
    enclosure.ReportStatus();
    return;
  }

  // if chamber is not exist, not allow other operation
  if (!enclosure.IsOnline()) {
    SERIAL_ECHOLN("Enclosure is not online.");
    return;
  }

  s = parser.byteval('S', 12);
  p = parser.byteval('P', 0);
  switch (s)
  {
  case 0:
    enclosure.Disable();
    break;

  case 1:
    enclosure.Enable();
    break;

  case 2:
    ModuleBase::UnlockMarlinUart();
    break;

  case 3:
    enclosure.SetLightBar(p);
    break;

  case 4:
    enclosure.SetFanSpeed(p);
    break;

  case 10:
    enclosure.door_state(ENCLOSURE_DOOR_STATE_OPEN);
    SERIAL_ECHOLN("triggered door open");
    break;

  case 11:
    enclosure.door_state(ENCLOSURE_DOOR_STATE_CLOSED);
    SERIAL_ECHOLN("triggered door close");
    break;

  case 12:
    enclosure.PollDoorState();
    break;

  default:
    break;
  }
}
