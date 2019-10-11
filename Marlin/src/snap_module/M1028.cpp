#include "M1028.h"
#include "../../Configuration.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"

#define MAX_CALI_SPEED_XY 100
#define MAX_CALI_SPEED_Z  60

#define CALI_SPEED_XY   50
#define CALI_SPEED_Z    30

#define Z_POS_BEFORE_CALI 20
#define Z_POS_CALI_OFFSET 15
#define Z_POS_AFTER_CALI  100

#define LASER_POWER_CALI  15

enum SettingOpt : uint8_t {
  OPT_SHOW_PARAM = 0,
  OPT_SET_HOME_SPEED,
  OPT_SET_HOME_BUMP_SPEED_DIVISOR,
  OPT_SET_MAX_CALI_SPEED,
  OPT_SET_CALI_SPEED,
  OPT_SET_Z_POS,
  OPT_SET_LASER_PWR_CALI,

  OPT_INVALID
};

float sm_homing_feedrate[XYZ] = {HOMING_FEEDRATE_XY/60, HOMING_FEEDRATE_XY/60, HOMING_FEEDRATE_Z/60};
uint8_t sm_homing_bump_divisor[XYZ] = HOMING_BUMP_DIVISOR;

// speed in calibration
float max_speed_in_calibration[XYZ] = {MAX_CALI_SPEED_XY, MAX_CALI_SPEED_XY, MAX_CALI_SPEED_Z};
float speed_in_calibration[XYZ] = {CALI_SPEED_XY, CALI_SPEED_XY, CALI_SPEED_Z};

// z position before calibrate 3DP
float z_position_before_calibration = Z_POS_BEFORE_CALI;

// z position when adjust the Z offset after probe points in calibrate 3DP
float z_position_in_cali_offset = Z_POS_CALI_OFFSET;

// z position after exit calibrating laser or 3DP when all axes homed
float z_position_after_calibration = Z_POS_AFTER_CALI;

// laser power when calibrating laser
float laser_pwr_in_cali = LASER_POWER_CALI;

/**
 * configure the customize parameters of snapmaker
 *  S:
 *    0: show current parameters
 *    1: home feedrate, X<X speed> Y<Y speed> Z<Z speed>
 *    2: home bump speed divisor
 *    3: max feedrate in calibration, X<X speed> Y<Y speed> Z<Z speed>
 *    4: feedrate in calibration, X<X speed> Y<Y speed> Z<Z speed>
 *    5: Z postions in calibration, details please see comments above
 */

void GcodeSuite::M1028() {
  uint8_t s;

  s = (uint8_t)parser.byteval('S', (uint8_t)0);

  switch (s) {
  case OPT_SHOW_PARAM:
    SERIAL_ECHOLNPAIR("\nhoming speed: X:", sm_homing_feedrate[X_AXIS], ", Y:", sm_homing_feedrate[Y_AXIS], ", Z:", sm_homing_feedrate[Z_AXIS]);
    SERIAL_ECHOLNPAIR("\nhoming bump speed divisor: X:", sm_homing_bump_divisor[X_AXIS], ", Y:", sm_homing_bump_divisor[Y_AXIS], ", Z:", sm_homing_bump_divisor[Z_AXIS]);
    SERIAL_ECHOLNPAIR("\nmax caliration speed: X:", max_speed_in_calibration[X_AXIS], ", Y:", max_speed_in_calibration[Y_AXIS], ", Z:", max_speed_in_calibration[Z_AXIS]);
    SERIAL_ECHOLNPAIR("\ncalibration speed: X:", speed_in_calibration[X_AXIS], ", Y:", speed_in_calibration[Y_AXIS], ", Z:", speed_in_calibration[Z_AXIS]);
    SERIAL_ECHOLNPAIR("\nz pos: before cali: ", z_position_before_calibration, ", cali offset: ", z_position_in_cali_offset, ", after cali: ", z_position_after_calibration);
    SERIAL_ECHOLNPAIR("\nlaser power in calibration: ", laser_pwr_in_cali);
    break;

  case OPT_SET_HOME_SPEED:
    sm_homing_feedrate[X_AXIS] = (float)parser.floatval('X', (float)HOMING_FEEDRATE_XY);
    sm_homing_feedrate[Y_AXIS] = (float)parser.floatval('Y', (float)HOMING_FEEDRATE_XY);
    sm_homing_feedrate[Z_AXIS] = (float)parser.floatval('Z', (float)HOMING_FEEDRATE_Z);
    break;

  case OPT_SET_HOME_BUMP_SPEED_DIVISOR:
    sm_homing_bump_divisor[X_AXIS] = (uint8_t)parser.byteval('I', (uint8_t)9);
    sm_homing_bump_divisor[Y_AXIS] = (uint8_t)parser.byteval('J', (uint8_t)9);
    sm_homing_bump_divisor[Z_AXIS] = (uint8_t)parser.byteval('K', (uint8_t)4);
    break;

  case OPT_SET_MAX_CALI_SPEED:
    max_speed_in_calibration[X_AXIS] = (float)parser.floatval('X', (float)MAX_CALI_SPEED_XY);
    max_speed_in_calibration[Y_AXIS] = (float)parser.floatval('Y', (float)MAX_CALI_SPEED_XY);
    max_speed_in_calibration[Z_AXIS] = (float)parser.floatval('Z', (float)MAX_CALI_SPEED_Z);
    break;

  case OPT_SET_CALI_SPEED:
    speed_in_calibration[X_AXIS] = (float)parser.floatval('X', (float)CALI_SPEED_XY);
    speed_in_calibration[Y_AXIS] = (float)parser.floatval('Y', (float)CALI_SPEED_XY);
    speed_in_calibration[Z_AXIS] = (float)parser.floatval('Z', (float)CALI_SPEED_Z);
    break;

  case OPT_SET_Z_POS:
    z_position_before_calibration = (float)parser.floatval('I', (float)Z_POS_BEFORE_CALI);
    z_position_in_cali_offset = (float)parser.floatval('J', (float)Z_POS_CALI_OFFSET);
    z_position_after_calibration = (float)parser.floatval('K', (float)Z_POS_AFTER_CALI);
    break;

  case OPT_SET_LASER_PWR_CALI:
    laser_pwr_in_cali = (float)parser.floatval('P', (float)LASER_POWER_CALI);
    break;

  default:
    SERIAL_ECHOLNPAIR("Error: invalid option: ", s);
    break;
  }
}
