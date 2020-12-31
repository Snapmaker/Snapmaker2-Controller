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

/*
 * Power domain control:
 *   S: just for screen
 *   T: for all executors and all linear modules
 *   B: for bed and addon
 * 
 * Call with:
 *   M2001 S{0} T{0} B{0}
 */
void GcodeSuite::M2001() {
  SysStatus cur_status = systemservice.GetCurrentStatus();
  if (cur_status != SYSTAT_IDLE) {
    SERIAL_ERROR_MSG("Machine not idle");
    LOG_E("cannot control power at current status: %d\n", cur_status);
    return;
  }

  bool reboot_req = false, val;
  if (parser.seen('S')) {
    if (parser.value_bool()) enable_power_domain(POWER_DOMAIN_0);
    else disable_power_domain(POWER_DOMAIN_0);
  }
  if (parser.seen('T')) {
    val = parser.value_bool();
    reboot_req |=  val && (READ(POWER1_SUPPLY_PIN) != POWER1_SUPPLY_ON);
    if (val) enable_power_domain(POWER_DOMAIN_1);
    else {
      SERIAL_ECHO_MSG("Wait 10 Seconds Before Unplugging Tool Head!");
      disable_power_domain(POWER_DOMAIN_1);
    }
  }
  if (parser.seen('B')) {
    val = parser.value_bool();
    reboot_req |=  val && (READ(POWER2_SUPPLY_PIN) != POWER2_SUPPLY_ON);
    if (val) enable_power_domain(POWER_DOMAIN_2);
    else disable_power_domain(POWER_DOMAIN_2);
  }

  // Output current power domain and ban status:
  SERIAL_ECHOLN("Power Domain Status");
  SERIAL_ECHOLNPAIR("  S: Touchscreen ", (READ(POWER0_SUPPLY_PIN) == POWER0_SUPPLY_ON ? MSG_ON : MSG_OFF));
  SERIAL_ECHOLNPAIR("  T: Tool & Linear Module ", (READ(POWER1_SUPPLY_PIN) == POWER1_SUPPLY_ON ? MSG_ON : MSG_OFF));
  SERIAL_ECHOLNPAIR("  B: Bed & Addon ", (READ(POWER2_SUPPLY_PIN) == POWER2_SUPPLY_ON ? MSG_ON : MSG_OFF));
  SERIAL_ECHOLN("Power Ban Status:");
  if (power_ban == POWER_DOMAIN_NONE)
    SERIAL_ECHOLN("  No Power Domains Banned");
  else {
    if (power_ban & POWER_DOMAIN_0)
      SERIAL_ECHOLN("  S: Screen Ban");
    if (power_ban & POWER_DOMAIN_1)
      SERIAL_ECHOLN("  T: Tool & Linear Module Ban");
    if (power_ban & POWER_DOMAIN_2)
      SERIAL_ECHOLN("  B: Bed & Addon Ban");
  }

  if (reboot_req) {
    SERIAL_ECHO_MSG("Rebooting, please wait...");
    LOG_I("M2001 triggered reboot");
    disable_power_domain(POWER_DOMAIN_0); _delay_ms(5000); enable_power_domain(POWER_DOMAIN_0); // Reboot screen
    M1999(); // Reboot controller
  }
}
