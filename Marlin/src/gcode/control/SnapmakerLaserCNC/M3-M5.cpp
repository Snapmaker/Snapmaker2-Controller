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

#include "../../inc/MarlinConfig.h"

#if ENABLED(SPAPMAKER_LASER_CNC)

#include "../../gcode.h"
#include "../../../module/stepper.h"
#include "../../../module/laserexecuter.h"
#include "../../../module/executermanager.h"

/**
 * M3: Spindle Clockwise
 * M4: Spindle Counter-clockwise
 *
 *  S0 turns off spindle.
 *
 *  If no speed PWM output is defined then M3/M4 just turns it on.
 *
 *  At least 12.8KHz (50Hz * 256) is needed for spindle PWM.
 *  Hardware PWM is required. ISRs are too slow.
 *
 * NOTE: WGM for timers 3, 4, and 5 must be either Mode 1 or Mode 5.
 *       No other settings give a PWM signal that goes from 0 to 5 volts.
 *
 *       The system automatically sets WGM to Mode 1, so no special
 *       initialization is needed.
 *
 *       WGM bits for timer 2 are automatically set by the system to
 *       Mode 1. This produces an acceptable 0 to 5 volt signal.
 *       No special initialization is needed.
 *
 * NOTE: A minimum PWM frequency of 50 Hz is needed. All prescaler
 *       factors for timers 2, 3, 4, and 5 are acceptable.
 *
 *  SPINDLE_LASER_ENABLE_PIN needs an external pullup or it may power on
 *  the spindle/laser during power-up or when connecting to the host
 *  (usually goes through a reset which sets all I/O pins to tri-state)
 *
 *  PWM duty cycle goes from 0 (off) to 255 (always on).
 */
 
void GcodeSuite::M3_M4(const bool is_M4) {

  planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

  /**
   * Our final value for ocr_val is an unsigned 8 bit value between 0 and 255 which usually means uint8_t.
   * Went to uint16_t because some of the uint8_t calculations would sometimes give 1000 0000 rather than 1111 1111.
   * Then needed to AND the uint16_t result with 0x00FF to make sure we only wrote the byte of interest.
   */
 
  if(MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
    if(parser.seen('P')) {
      ExecuterHead.Laser.SetLaserPower(parser.value_float());
    }
    else {
      ExecuterHead.Laser.On();
    }
  }
  else if(MACHINE_TYPE_CNC == ExecuterHead.MachineType) {
    if(parser.seen('P')) ExecuterHead.CNC.SetPower(parser.value_ushort());
    else ExecuterHead.CNC.On();
  }

  
}

/**
 * M5 turn off spindle
 */
void GcodeSuite::M5() {
  planner.synchronize();
  //set_spindle_laser_enabled(false);
  if(MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
    ExecuterHead.Laser.Off();
  }
  else if(MACHINE_TYPE_CNC == ExecuterHead.MachineType) {
    ExecuterHead.CNC.Off();
  }
}

#endif // SPAPMAKER_LASER_CNC