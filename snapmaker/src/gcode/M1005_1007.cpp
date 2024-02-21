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
#include "../module/toolhead_cnc_200w.h"

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

  linear_p->ShowAllLinearInfo();

  if (kit_combination_type) {
    LOG_I("kits:");

    if (kit_combination_type & QUICK_CHANGE_ADAPTER_MSK)
      LOG_I(" quick_change_kit");

    if (kit_combination_type & REINFORCEMENT_KIT_MSK)
      LOG_I(" reinforcement_kit");

    LOG_I("\n");
  }

  if (ModuleBase::toolhead() == MACHINE_TYPE_LASER || (ModuleBase::toolhead() == MACHINE_TYPE_LASER_10W)) {
    laser->ReadBluetoothVer();
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
  if (ModuleBase::toolhead() == MODULE_TOOLHEAD_CNC || ModuleBase::toolhead() == MODULE_TOOLHEAD_CNC_200W) {
    if (parser.seenval('L')) {
      bool is_print_rpm_ = parser.boolval('L', false);
      if (ModuleBase::toolhead() == MODULE_TOOLHEAD_CNC)
        cnc.set_is_print_rpm(is_print_rpm_);
      else if (ModuleBase::toolhead() == MODULE_TOOLHEAD_CNC_200W)
        cnc_200w.set_is_print_rpm(is_print_rpm_);
      return;
    }
  }

  SERIAL_ECHO("Tool Head: ");

  switch (ModuleBase::toolhead()) {
  case MODULE_TOOLHEAD_3DP:
  case MODULE_TOOLHEAD_DUALEXTRUDER:
    SERIAL_ECHOLN("3DP");
    if (ModuleBase::toolhead() == MODULE_TOOLHEAD_DUALEXTRUDER) {
      printer1->ShowInfo();
      // show hotend offset
      M218();
    }
    break;

  case MODULE_TOOLHEAD_LASER:
    SERIAL_ECHOLN("LASER");
    SERIAL_ECHO("Current Status: ");
    SERIAL_ECHOLN((laser->state() == TOOLHEAD_LASER_STATE_ON)? "ON" : "OFF");
    SERIAL_ECHOLNPAIR("Current Power: ", laser->power());
    SERIAL_ECHOLNPAIR("Focus Height: ", laser->focus());
    break;

  case MODULE_TOOLHEAD_LASER_10W:
    SERIAL_ECHOLN("10W LASER");
    SERIAL_ECHO("Current Status: ");
    SERIAL_ECHOLN((laser->state() == TOOLHEAD_LASER_STATE_ON)? "ON" : "OFF");
    SERIAL_ECHOLNPAIR("Current Power: ", laser->power());
    SERIAL_ECHOLNPAIR("Focus Height: ", laser->focus());
    break;

  case MODULE_TOOLHEAD_LASER_20W:
    SERIAL_ECHOLN("20W LASER");
    SERIAL_ECHO("Current Status: ");
    SERIAL_ECHOLN((laser->state() == TOOLHEAD_LASER_STATE_ON)? "ON" : "OFF");
    SERIAL_ECHOLNPAIR("Current Power: ", laser->power());
    SERIAL_ECHOLNPAIR("Focus Height: ", laser->focus());
    break;

  case MODULE_TOOLHEAD_LASER_40W:
    SERIAL_ECHOLN("40W LASER");
    SERIAL_ECHO("Current Status: ");
    SERIAL_ECHOLN((laser->state() == TOOLHEAD_LASER_STATE_ON)? "ON" : "OFF");
    SERIAL_ECHOLNPAIR("Current Power: ", laser->power());
    SERIAL_ECHOLNPAIR("Focus Height: ", laser->focus());
    break;

  case MODULE_TOOLHEAD_LASER_RED_2W:
    SERIAL_ECHOLN("2W RED LASER");
    SERIAL_ECHO("Current Status: ");
    SERIAL_ECHOLN((laser->state() == TOOLHEAD_LASER_STATE_ON)? "ON" : "OFF");
    SERIAL_ECHOLNPAIR("Current Power: ", laser->power());
    SERIAL_ECHOLNPAIR("Focus Height: ", laser->focus());
    break;

  case MODULE_TOOLHEAD_CNC:
    SERIAL_ECHOLN("CNC");
    SERIAL_ECHOLNPAIR("Current Power: ", cnc.power());
    SERIAL_ECHOLNPAIR("RPM: ", cnc.rpm());
    break;

  case MODULE_TOOLHEAD_CNC_200W:
    LOG_I("200W CNC\n");
    cnc_200w.PrintInfo();
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
