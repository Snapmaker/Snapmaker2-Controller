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
#include "../common/debug.h"

#include "../service/upgrade.h"
#include "../module/can_host.h"
#include "../module/module_base.h"
#include "../module/linear.h"
#include "../module/toolhead_3dp.h"
#include "../module/toolhead_cnc.h"
#include "../module/toolhead_laser.h"

// marlin headers
#include "src/gcode/gcode.h"
#include "src/module/motion.h"
#include "src/libs/hex_print_routines.h"

void GcodeSuite::M1005() {
  MAC_t       mac;

  char buffer[VERSION_STRING_SIZE + 4];
  int  i;

  // version in code
  SERIAL_ECHOPAIR(MSG_MARLIN," ", SHORT_BUILD_VERSION, "\n");
  SERIAL_ECHOPAIR("Compiled: ", __DATE__, ", ", __TIME__, "\n");

  // version in package
  memcpy(buffer, (char*)(FLASH_BOOT_PARA + 2048), 30);
  SERIAL_ECHOPAIR(MSG_MARLIN_PACK, ": ", buffer, "\n");


  // version of modules
  SERIAL_ECHOPAIR("Module Ver:\n");
  for (i = 0; i < MODULE_SUPPORT_CONNECTED_MAX; i++) {
    mac.val = canhost.mac(i);

    if (mac.val == MODULE_MAC_ID_INVALID)
      break;
    canhost.ShowModuleVersion(mac);
  }

  if (ModuleBase::toolhead() == MACHINE_TYPE_LASER) {
    laser.ReadBluetoothVer();
  }
  SERIAL_ECHO("Machine Size: ");
  switch (linear_p->machine_size()) {
  case MACHINE_SIZE_A150:
    SERIAL_ECHOLN("S");
    break;

  case MACHINE_SIZE_A250:
    SERIAL_ECHOLN("M");
    break;

  case MACHINE_SIZE_A350:
    SERIAL_ECHOLN("L");
    break;

  default:
    SERIAL_ECHOLN("U");
    break;
  }
}


void GcodeSuite::M1006() {
  SERIAL_ECHO("Tool Head: ");

  switch (ModuleBase::toolhead()) {
  case MODULE_TOOLHEAD_3DP:
    SERIAL_ECHOLN("3DP");
    break;

  case MODULE_TOOLHEAD_LASER:
    SERIAL_ECHOLN("LASER");
    SERIAL_ECHO("Current Status: ");
    SERIAL_ECHOLN((laser.state() == TOOLHEAD_LASER_STATE_ON)? "ON" : "OFF");
    SERIAL_ECHOLNPAIR("Current Power: ", laser.power());
    SERIAL_ECHOLNPAIR("Focus Height: ", laser.focus());
    break;

  case MODULE_TOOLHEAD_CNC:
    SERIAL_ECHOLN("CNC");
    SERIAL_ECHOLNPAIR("Current Power: ", cnc.power());
    SERIAL_ECHOLNPAIR("RPM: ", cnc.rpm());
    break;

  default:
    SERIAL_ECHOLN("UNKNOWN");
    break;
  }
}


void GcodeSuite::M1007() {
  SERIAL_ECHO("Homed: ");
  SERIAL_ECHOLN(all_axes_homed()? "YES" : "NO");

  SERIAL_ECHOLNPAIR("Selected origin num: ", active_coordinate_system + 1);

  SERIAL_ECHO("Selected == Current: ");

  if (active_coordinate_system < 0) {
    SERIAL_ECHOLN("YES");
  }
  else if ((position_shift[X_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS]) &&
        (position_shift[Y_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS]) &&
        (position_shift[Z_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS]) &&
        (position_shift[B_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][B_AXIS])) {
    SERIAL_ECHOLN("YES");
  }
  else {
    SERIAL_ECHOLN("NO");
  }

  if (active_coordinate_system < 0) {
    SERIAL_ECHOLNPAIR("Origin offset X: ", position_shift[X_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset Y: ", position_shift[Y_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset Z: ", position_shift[Z_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset B: ", position_shift[B_AXIS]);
  }
  else {
    SERIAL_ECHOLNPAIR("Origin offset X: ", gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset Y: ", gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset Z: ", gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset B: ", gcode.coordinate_system[gcode.active_coordinate_system][B_AXIS]);
  }
}
