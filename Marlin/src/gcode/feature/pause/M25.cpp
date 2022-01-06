/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 *
 */

#include "../../gcode.h"
#include "../../../module/printcounter.h"
#include "../../../../../snapmaker/src/service/system.h"
#include "../../../../../snapmaker/src/service/power_loss_recovery.h"
/**
 * M25: Pause hmi print
 */
void GcodeSuite::M25() {
  SSTP_Event_t event;
  event.id = EID_SYS_CTRL_REQ;
  event.op_code = SYSCTL_OPC_PAUSE;
  event.length = 0;
  planner.synchronize();
  SERIAL_ECHOPAIR("MC REQ PAUSE\n");
  systemservice.ChangeSystemStatus(event);
  // To pause recovery, skip the current line
  pl_recovery.SaveCmdLine(pl_recovery.LastLine() + 1);
}
