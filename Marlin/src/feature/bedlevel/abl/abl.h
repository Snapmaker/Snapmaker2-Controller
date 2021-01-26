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

#include "../../../inc/MarlinConfigPre.h"

#define GRID_MAX_POINTS_INDEX (GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y)

extern int bilinear_grid_spacing[2], bilinear_start[2];
extern float bilinear_grid_factor[2],
             z_values[GRID_MAX_NUM][GRID_MAX_NUM];
extern float z_values_virt[VIRTUAL_GRID_MAX_NUM][VIRTUAL_GRID_MAX_NUM];
extern float extruders_z_values[EXTRUDERS][GRID_MAX_NUM][GRID_MAX_NUM];
extern float extruders_z_values_virt[EXTRUDERS][VIRTUAL_GRID_MAX_NUM][VIRTUAL_GRID_MAX_NUM];
float bilinear_z_offset(const float raw[XYZ]);
void bilinear_grid_manual();

void extrapolate_unprobed_bed_level();
void print_bilinear_leveling_grid();
void refresh_bed_level();
#if ENABLED(ABL_BILINEAR_SUBDIVISION)
  void print_bilinear_leveling_grid_virt();
  void bed_level_virt_interpolate();
  void bed_level_virt_interpolate(uint8_t extruder_index);
#endif

#if IS_CARTESIAN && DISABLED(SEGMENT_LEVELED_MOVES)
  void bilinear_line_to_destination(const float fr_mm_s, uint16_t x_splits=0xFFFF, uint16_t y_splits=0xFFFF);
#endif

#define _GET_MESH_X(I) (bilinear_start[X_AXIS] + (I) * bilinear_grid_spacing[X_AXIS])
#define _GET_MESH_Y(J) (bilinear_start[Y_AXIS] + (J) * bilinear_grid_spacing[Y_AXIS])
#define Z_VALUES_ARR  z_values

#define MAX_NOZZLE_HEIGHT_PROBED  6
extern float nozzle_height_probed;


uint8_t auto_probing(bool reply_screen, bool fast_leveling);
void compensate_offset();
void compensate_offset(float offset);