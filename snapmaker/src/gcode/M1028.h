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
