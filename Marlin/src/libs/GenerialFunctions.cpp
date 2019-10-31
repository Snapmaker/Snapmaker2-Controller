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

#include "../inc/MarlinConfig.h"
#include "stdio.h"
#include "../module/ExecuterManager.h"
#include "../module/endstops.h"
#include "../module/StatusControl.h"
#include "../module/PeriphDevice.h"

static char ValueString[12]; 

char* Value32BitToString(uint32_t Value) {
  sprintf(ValueString, "%08X", (int)Value);
  return ValueString;
}

char* Value16BitToString(uint16_t Value) {
  sprintf(ValueString, "%04X", Value);
  return ValueString;
}

char* Value8BitToString(uint8_t Value) {
  sprintf(ValueString, "%02X", Value);
  return ValueString;
}

int TempReport(uint8_t *pBuff) {
  ExecuterHead.temp_hotend[0] = (float)(((int16_t)(pBuff[0] << 8) | pBuff[1])) / 10.0f;
  ExecuterHead.CanTempMeasReady = true;
  ExecuterHead.watch.IamAlive();
  return 0;
}

int LimitReport(uint8_t *pBuff) {
  return 0;
}

int ProbeReport(uint8_t *pBuff) {
  CanModules.Endstop |= _BV(Z_MIN_PROBE);
  if(pBuff[0] == 0) CanModules.Endstop &= ~(_BV(Z_MIN_PROBE));
  return 0;
}


static void FilamentSensorReport(uint8_t *pBuff, enum EndstopEnum filament_num) {
  if (pBuff[0] == 0)
    CBI(CanModules.Endstop , FILAMENT1);
  else
    SBI(CanModules.Endstop , FILAMENT1);
}

int FilamentSensor1Report(uint8_t *pBuff) {
  FilamentSensorReport(pBuff, FILAMENT1);
  return 0;
}

int CNCRpmReport(uint8_t *pBuff) {
  uint16_t RPM;
  RPM = (pBuff[0] << 8) | pBuff[1];
  ExecuterHead.CNC.UpdateWorkingRPM(RPM);
  ExecuterHead.watch.IamAlive();
  return 0;
}

int LaserFocusReport(uint8_t *pBuff) {
  uint16_t u16Value;
  u16Value = ((uint16_t)pBuff[0] << 8) | pBuff[1];
  ExecuterHead.Laser.FocusHeight = (float)u16Value / 1000.0f;
  ExecuterHead.watch.IamAlive();
  return 0;
}

int NoopFunc(uint8_t *pBuff) {
  return 0;
}

int EnclosureDoorReport(uint8_t *pBuff) {
  if (pBuff[0] == 0) {
    CBI(CanModules.PeriphSwitch , CAN_IO_ENCLOSURE);
  }
  else {
    SBI(CanModules.PeriphSwitch , CAN_IO_ENCLOSURE);
  }
  return 0;
}

uint8_t CanDebugBuff[32];
uint8_t CanDebugLen;
int CanDebug(uint8_t *pBuff, uint8_t Len) {
  CanDebugLen = 0;
  for(int i=0;i<Len;i++) {
    CanDebugBuff[CanDebugLen++] = pBuff[i];
  }
  return 0;
}



