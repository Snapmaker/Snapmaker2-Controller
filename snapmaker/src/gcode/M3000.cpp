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
#include "../module/toolhead_3dp.h"
#include "../../../Marlin/src/module/endstops.h"
#include "../service/bed_level.h"

void GcodeSuite::M3000() {
  printer1->ShowInfo();
}

void GcodeSuite::M3001() {
  printer1->SelectProbeSensor(PROBE_SENSOR_PROXIMITY_SWITCH);
  printer1->ModuleCtrlProximitySwitchPower(1);
  endstops.enable_z_probe(true);
  do_blocking_move_to_z(current_position[Z_AXIS] - 100);
}

void GcodeSuite::M3002() {
  const bool seen_l = parser.seenval('L');
  if (seen_l) {
    uint8_t stage = (uint8_t)parser.byteval('L', (uint8_t)0);
    // SSTP_Event_t event;
    // uint8_t buf[10];
    // event.data = buf;
    switch (stage) {
      case 0:
        levelservice.ProbeSensorCalibrationLeftExtruderAutoProbe();
        break;
      case 1:
        levelservice.ProbeSensorCalibrationRightExtruderAutoProbe();
        break;
      case 2:
        levelservice.ProbeSensorCalibrationRightExtruderManualProbe();
        break;
      case 3:
        levelservice.ProbeSensorCalibraitonRightExtruderPositionConfirm();
        break;
      case 4:
        levelservice.ProbeSensorCalibrationLeftExtruderManualProbe();
        break;
      case 5:
        levelservice.ProbeSensorCalibraitonLeftExtruderPositionConfirm();
        break;
      default:
        break;
    }
  }

}

void GcodeSuite::M3003() {
  const bool seen_l = parser.seenval('L');
  if (seen_l) {
    uint8_t stage = (uint8_t)parser.byteval('L', (uint8_t)0);
    SSTP_Event_t event;
    uint8_t buf[10];
    event.data = buf;
    switch (stage) {
      case 0:
        buf[0] = (uint8_t)parser.byteval('G', (uint8_t)0);
        levelservice.DoDualExtruderAutoLeveling(event);
        break;
      case 1:
        buf[0] = (uint8_t)parser.byteval('P', (uint8_t)0);
        levelservice.DualExtruderAutoLevelingProbePoint(event);
        break;
      case 2:
        levelservice.FinishDualExtruderAutoLeveling(event);
        break;
      default:
        break;
    }
  }
}

void GcodeSuite::M3004() {
  const bool seen_l = parser.seenval('L');
  if (seen_l) {
    uint8_t stage = (uint8_t)parser.byteval('L', (uint8_t)0);
    SSTP_Event_t event;
    uint8_t buf[10];
    event.data = buf;
    switch (stage) {
      case 0:
        buf[0] = (uint8_t)parser.byteval('G', (uint8_t)0);
        levelservice.DoDualExtruderManualLeveling(event);
        break;
      case 1:
        buf[0] = (uint8_t)parser.byteval('P', (uint8_t)0);
        levelservice.DualExtruderManualLevelingProbePoint(event);
        break;
      case 2:
        levelservice.FinishDualExtruderManualLeveling(event);
        break;
      default:
        break;
    }
  }
}

void GcodeSuite::M3005() {
  const bool seen_l = parser.seenval('L');
  if (seen_l) {
    uint8_t stage = (uint8_t)parser.byteval('L', (uint8_t)0);
    SSTP_Event_t event;
    uint8_t buf[10];
    event.data = buf;
    switch (stage) {
      case 0:
        buf[0] = (uint8_t)parser.byteval('P', (uint8_t)0);
        levelservice.DualExtruderAutoBedDetect(event);
        break;
      case 1:
        buf[0] = (uint8_t)parser.byteval('P', (uint8_t)0);
        levelservice.DualExtruderManualBedDetect(event);
        break;
      default:
        break;
    }
  }
}

// extern ErrCode DoEInfinityMove(SSTP_Event_t &event);
// extern ErrCode StopEMoves(SSTP_Event_t &event);
// void GcodeSuite::M3005() {
//   const bool seen_l = parser.seenval('L');
//   if (seen_l) {
//     uint8_t stage = (uint8_t)parser.byteval('L', (uint8_t)0);
//     SSTP_Event_t event;
//     uint8_t buf[10];
//     event.data = buf;
//     float speed;
//     uint32_t speed_scaled;
//     switch (stage) {
//       case 0:
//         speed = (float)parser.floatval('P', (float)0);
//         speed_scaled = speed * 1000;
//         buf[0] = 0;
//         buf[1] = (speed_scaled >> 24) & 0xff;
//         buf[2] = (speed_scaled >> 16) & 0xff;
//         buf[3] = (speed_scaled >> 8) & 0xff;
//         buf[4] = speed_scaled & 0xff;
//         event.length = 5;
//         DoEInfinityMove(event);
//         break;
//       case 1:
//         speed = (float)parser.floatval('P', (float)0);
//         speed_scaled = speed * 1000;
//         buf[0] = 1;
//         buf[1] = (speed_scaled >> 24) & 0xff;
//         buf[2] = (speed_scaled >> 16) & 0xff;
//         buf[3] = (speed_scaled >> 8) & 0xff;
//         buf[4] = speed_scaled & 0xff;
//         event.length = 5;
//         DoEInfinityMove(event);
//         break;
//       case 2:
//         StopEMoves(event);
//         break;
//       // default:
//       //   break;
//     }
//   }
// }

