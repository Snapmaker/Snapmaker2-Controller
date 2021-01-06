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
#include "M1028.h"

// marlin headers
#include "Configuration.h"
#include "src/gcode/gcode.h"
#include "src/module/motion.h"

#define MAX_CALI_SPEED_XY 100
#define MAX_CALI_SPEED_Z  40

#define CALI_SPEED_XY   50
#define CALI_SPEED_Z    30

#define Z_POS_BEFORE_CALI 20
#define Z_POS_CALI_OFFSET 15
#define Z_POS_AFTER_CALI  100

#define Z_LIMIT_CALI      140

#define LASER_POWER_CALI  70

#define SPEED_IN_DRAWING_RULLER 5

#define GO_HOME_BEFORE_CALIBRATION  1

enum SettingOpt : uint8_t {
  OPT_SHOW_PARAM = 0,
  OPT_SET_HOME_SPEED,
  OPT_SET_HOME_BUMP_SPEED_DIVISOR,
  OPT_SET_MAX_CALI_SPEED,
  OPT_SET_CALI_SPEED,
  OPT_SET_Z_POS,
  OPT_SET_LASER_PWR_CALI,
  OPT_SET_Z_LIMIT_CALI,
  OPT_SET_SPEED_IN_DRAWING_RULER,

  OPT_GO_HOME_BEFORE_CALI,

  OPT_INVALID
};

float sm_homing_feedrate[XN] = {HOMING_FEEDRATE_XY/60, HOMING_FEEDRATE_XY/60, HOMING_FEEDRATE_Z/60, HOMING_FEEDRATE_B/60};
uint8_t sm_homing_bump_divisor[XN] = HOMING_BUMP_DIVISOR;

// speed in calibration
float max_speed_in_calibration[XYZ] = {MAX_CALI_SPEED_XY, MAX_CALI_SPEED_XY, MAX_CALI_SPEED_Z};
float speed_in_calibration[XYZ] = {CALI_SPEED_XY, CALI_SPEED_XY, CALI_SPEED_Z};

// z position before calibrate 3DP
float z_position_before_calibration = Z_POS_BEFORE_CALI;

// z position when adjust the Z offset after probe points in calibrate 3DP
float z_position_in_cali_offset = Z_POS_CALI_OFFSET;

// z position after exit calibrating laser or 3DP when all axes homed
float z_position_after_calibration = Z_POS_AFTER_CALI;

// z Minimum height when enter leveling
float z_limit_in_cali = Z_LIMIT_CALI;

// laser power when calibrating laser
float laser_pwr_in_cali = LASER_POWER_CALI;
float speed_in_draw_ruler = SPEED_IN_DRAWING_RULLER;

bool go_home_before_cali = true;

/**
 * configure the customize parameters of snapmaker
 *  S:
 *    0: show current parameters
 *    1: home feedrate, X<X speed> Y<Y speed> Z<Z speed>
 *    2: home bump speed divisor
 *    3: max feedrate in calibration, X<X speed> Y<Y speed> Z<Z speed>
 *    4: feedrate in calibration, X<X speed> Y<Y speed> Z<Z speed>
 *    5: Z postions in calibration, details please see comments above
 *    6: laser power in calibration
 *    7: z limit in calibration
 *    8: speed in drawing ruler
 */

void GcodeSuite::M1028() {
  uint8_t s;

  s = (uint8_t)parser.byteval('S', (uint8_t)0);

  switch (s) {
  case OPT_SHOW_PARAM:
    SERIAL_ECHOLNPAIR("\nS1: homing speed: X:", sm_homing_feedrate[X_AXIS], ", Y:", sm_homing_feedrate[Y_AXIS], ", Z:", sm_homing_feedrate[Z_AXIS]);
    SERIAL_ECHOLNPAIR("\nS2: homing bump speed divisor: X:", sm_homing_bump_divisor[X_AXIS], ", Y:", sm_homing_bump_divisor[Y_AXIS], ", Z:", sm_homing_bump_divisor[Z_AXIS]);
    SERIAL_ECHOLNPAIR("\nS3: max caliration speed: X:", max_speed_in_calibration[X_AXIS], ", Y:", max_speed_in_calibration[Y_AXIS], ", Z:", max_speed_in_calibration[Z_AXIS]);
    SERIAL_ECHOLNPAIR("\nS4: calibration speed: X:", speed_in_calibration[X_AXIS], ", Y:", speed_in_calibration[Y_AXIS], ", Z:", speed_in_calibration[Z_AXIS]);
    SERIAL_ECHOLNPAIR("\nS5: z pos: before cali: ", z_position_before_calibration, ", cali offset: ", z_position_in_cali_offset, ", after cali: ", z_position_after_calibration);
    SERIAL_ECHOLNPAIR("\nS6: laser power in calibration: ", laser_pwr_in_cali);
    SERIAL_ECHOLNPAIR("\nS7: Z limit in calibration: ", z_limit_in_cali);
    SERIAL_ECHOLNPAIR("\nS8: speed in drawing ruler: ", speed_in_draw_ruler);
    break;

  case OPT_SET_HOME_SPEED:
    sm_homing_feedrate[X_AXIS] = (float)parser.floatval('X', (float)sm_homing_feedrate[X_AXIS]);
    sm_homing_feedrate[Y_AXIS] = (float)parser.floatval('Y', (float)sm_homing_feedrate[Y_AXIS]);
    sm_homing_feedrate[Z_AXIS] = (float)parser.floatval('Z', (float)sm_homing_feedrate[Z_AXIS]);
    break;

  case OPT_SET_HOME_BUMP_SPEED_DIVISOR:
    sm_homing_bump_divisor[X_AXIS] = (uint8_t)parser.byteval('I', (uint8_t)sm_homing_bump_divisor[X_AXIS]);
    sm_homing_bump_divisor[Y_AXIS] = (uint8_t)parser.byteval('J', (uint8_t)sm_homing_bump_divisor[Y_AXIS]);
    sm_homing_bump_divisor[Z_AXIS] = (uint8_t)parser.byteval('K', (uint8_t)sm_homing_bump_divisor[Z_AXIS]);
    break;

  case OPT_SET_MAX_CALI_SPEED:
    max_speed_in_calibration[X_AXIS] = (float)parser.floatval('X', (float)max_speed_in_calibration[X_AXIS]);
    max_speed_in_calibration[Y_AXIS] = (float)parser.floatval('Y', (float)max_speed_in_calibration[Y_AXIS]);
    max_speed_in_calibration[Z_AXIS] = (float)parser.floatval('Z', (float)max_speed_in_calibration[Z_AXIS]);
    break;

  case OPT_SET_CALI_SPEED:
    speed_in_calibration[X_AXIS] = (float)parser.floatval('X', (float)speed_in_calibration[X_AXIS]);
    speed_in_calibration[Y_AXIS] = (float)parser.floatval('Y', (float)speed_in_calibration[Y_AXIS]);
    speed_in_calibration[Z_AXIS] = (float)parser.floatval('Z', (float)speed_in_calibration[Z_AXIS]);
    break;

  case OPT_SET_Z_POS:
    z_position_before_calibration = (float)parser.floatval('I', (float)z_position_before_calibration);
    z_position_in_cali_offset = (float)parser.floatval('J', (float)z_position_in_cali_offset);
    z_position_after_calibration = (float)parser.floatval('K', (float)z_position_after_calibration);
    break;

  case OPT_SET_LASER_PWR_CALI:
    laser_pwr_in_cali = (float)parser.floatval('P', (float)laser_pwr_in_cali);
    break;

  case OPT_SET_Z_LIMIT_CALI:
    z_limit_in_cali = (float)parser.floatval('I', (float)z_limit_in_cali);
    break;

  case OPT_SET_SPEED_IN_DRAWING_RULER:
    speed_in_draw_ruler = (float) parser.floatval('D', (float) speed_in_draw_ruler);
    break;

  case OPT_GO_HOME_BEFORE_CALI:
    go_home_before_cali = (uint8_t)parser.byteval('I', (uint8_t)GO_HOME_BEFORE_CALIBRATION);
    break;

  default:
    SERIAL_ECHOLNPAIR("Error: invalid option: ", s);
    break;
  }
}
