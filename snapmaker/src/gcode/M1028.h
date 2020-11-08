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
#ifndef SNAPMAKER_M1028_H_
#define SNAPMAKER_M1028_H_

#include "../common/config.h"

#include "src/Marlin.h"
#include "src/core/macros.h"

extern float sm_homing_feedrate[XYZ];
extern uint8_t sm_homing_bump_divisor[XYZ];

// speed in calibration, mm/s
extern float max_speed_in_calibration[XYZ];
extern float speed_in_calibration[XYZ];
extern float z_position_before_calibration;
extern float z_position_in_cali_offset;
extern float z_position_after_calibration;
extern float speed_in_draw_ruler;

extern float laser_pwr_in_cali;

extern float z_limit_in_cali;

extern bool go_home_before_cali;

#endif  // #ifndef SNAPMAKER_M1028_H_
