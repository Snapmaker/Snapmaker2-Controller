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
#include "snapmaker.h"
#include "src/inc/MarlinConfig.h"
#include "src/gcode/gcode.h"
#include HAL_PATH(src/HAL, HAL_watchdog_STM32F1.h)


void GcodeSuite::M1999() {
  SERIAL_ECHOLN("will reboot machine");
  disable_power_domain(POWER_DOMAIN_ALL);
  millis_t next_ms = millis() + 1000;
  while (PENDING(millis(), next_ms));
  nvic_sys_reset();
}
