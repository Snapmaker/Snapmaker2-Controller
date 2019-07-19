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
#pragma once

#include <stdint.h>

char* Value32BitToString(uint32_t Value);
char* Value16BitToString(uint16_t Value);
char* Value8BitToString(uint8_t Value);

int TempReport(uint8_t *pBuff);
int LimitReport(uint8_t *pBuff);
int ProbeReport(uint8_t *pBuff);
int FilamentSensor1Report(uint8_t *pBuff);
int CNCRpmReport(uint8_t *pBuff);
int LaserFocusReport(uint8_t *pBuff);
int NoopFunc(uint8_t *pBuff);
int CanDebug(uint8_t *pBuff, uint8_t Len);
int EnclosureDoorReport(uint8_t *pBuff);

extern uint8_t CanDebugBuff[32];
extern uint8_t CanDebugLen;
