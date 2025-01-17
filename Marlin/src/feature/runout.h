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

#if HAS_FILAMENT_SENSOR
/**
 * feature/runout.h - Runout sensor support
 */

#include "../module/printcounter.h"
#include "../module/stepper.h"
#include "../gcode/queue.h"
#include "../module/endstops.h"
#include "../inc/MarlinConfig.h"

#include "../snapmaker/src/module/toolhead_3dp.h"
#include "../snapmaker/src/service/system.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extensible_ui/ui_api.h"
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "pause.h"
#endif


//#define FILAMENT_RUNOUT_SENSOR_DEBUG

class FilamentMonitorBase {
  public:
    static bool enabled, filament_ran_out;
    static millis_t ranout_timer;

    #if ENABLED(HOST_ACTION_COMMANDS)
      static bool host_handling;
    #else
      constexpr static bool host_handling = false;
    #endif
  private:
    static uint8_t statefromcan;
};

template<class RESPONSE_T, class SENSOR_T>
class TFilamentMonitor : public FilamentMonitorBase {
  private:
    typedef RESPONSE_T response_t;
    typedef SENSOR_T   sensor_t;
    static  response_t response;
    static  sensor_t   sensor;

  public:
    static inline void setup() {
      sensor.setup();
      reset();
    }

    static inline void reset() {
      filament_ran_out = false;
      response.reset();
    }

    // Call this method when filament is present,
    // so the response can reset its counter.
    static inline void filament_present(const uint8_t extruder) {
      response.filament_present(extruder);
    }

    // Handle a block completion. RunoutResponseDelayed uses this to
    // add up the length of filament moved while the filament is out.
    static inline void block_completed(const block_t* const b) {
      if (enabled) {
        response.block_completed(b);
        sensor.block_completed(b);
      }
    }

    // Give the response a chance to update its counter.
    static inline void run() {
      uint32_t fault = systemservice.GetFaultFlag();
      if (enabled && !filament_ran_out && systemservice.GetCurrentStatus() == SYSTAT_WORK) {
        #if FILAMENT_RUNOUT_DISTANCE_MM > 0
          cli(); // Prevent RunoutResponseDelayed::block_completed from accumulating here
        #endif
        response.run();
        sensor.run();
        const bool ran_out = response.has_run_out();
        #if FILAMENT_RUNOUT_DISTANCE_MM > 0
          sei();
        #endif
        if (ran_out) {
          filament_ran_out = true;
          event_filament_runout();
        }
      }
      else if (enabled && (fault&FAULT_FLAG_FILAMENT)) {
        if (is_filament_runout())
          ranout_timer = millis() + 1000;
        else {
          // delay 1s to clear
          if (millis() - ranout_timer > 0) {
            filament_ran_out = false;
            systemservice.ClearExceptionByFaultFlag(FAULT_FLAG_FILAMENT);
          }
        }
      }
    }

    static uint8_t runout_states () { return sensor.poll_runout_states(); }

    // query current state of sensor
    // return true -> no filament
    // return false -> filament is exist
    static bool is_filament_runout() {
      if (enabled) {
        printer1->GetFilamentState();
        return sensor.poll_runout_states();
      }
      else
        return enabled;
    }
};

/*************************** FILAMENT PRESENCE SENSORS ***************************/

class FilamentSensorBase {
  protected:
    static void filament_present(const uint8_t extruder);

  public:
    static inline void setup() {
      #if ENABLED(FIL_RUNOUT_PULLUP)
        #define INIT_RUNOUT_PIN(P) SET_INPUT_PULLUP(P)
      #elif ENABLED(FIL_RUNOUT_PULLDOWN)
        #define INIT_RUNOUT_PIN(P) SET_INPUT_PULLDOWN(P)
      #else
        #define INIT_RUNOUT_PIN(P) SET_INPUT(P)
      #endif

      #if (MOTHERBOARD != BOARD_SNAPMAKER_2_0)
        INIT_RUNOUT_PIN(FIL_RUNOUT_PIN);
      #endif

      #if NUM_RUNOUT_SENSORS > 1
        #if DISABLED(CAN_FILAMENT2_RUNOUT)
          INIT_RUNOUT_PIN(FIL_RUNOUT2_PIN);
        #endif
        #if NUM_RUNOUT_SENSORS > 2
          #if DISABLED(CAN_FILAMENT3_RUNOUT)
            INIT_RUNOUT_PIN(FIL_RUNOUT3_PIN);
          #endif
          #if NUM_RUNOUT_SENSORS > 3
            #if DISABLED(CAN_FILAMENT4_RUNOUT)
              INIT_RUNOUT_PIN(FIL_RUNOUT4_PIN);
            #endif
            #if NUM_RUNOUT_SENSORS > 4
              #if DISABLED(CAN_FILAMENT5_RUNOUT)
                INIT_RUNOUT_PIN(FIL_RUNOUT5_PIN);
              #endif
              #if NUM_RUNOUT_SENSORS > 5
                #if DISABLED(CAN_FILAMENT6_RUNOUT)
                  INIT_RUNOUT_PIN(FIL_RUNOUT6_PIN);
                #endif
              #endif
            #endif
          #endif
        #endif
      #endif
    }

    #if (MOTHERBOARD != BOARD_SNAPMAKER_2_0)
    // Return a bitmask of runout pin states
    static inline uint8_t poll_runout_pins() {
      return (
        #if (MOTHERBOARD != BOARD_SNAPMAKER_2_0)
          (READ(FIL_RUNOUT_PIN ) ? _BV(0) : 0)
        #else
          (printer1->filament_state()? _BV(0):0)
        #endif
        #if NUM_RUNOUT_SENSORS > 1
          #if DISABLED(CAN_FILAMENT2_RUNOUT)
            | (READ(FIL_RUNOUT2_PIN) ? _BV(1) : 0)
          #else
            | (TEST(CanModules.Endstop, FILAMENT2)? _BV(1):0)
          #endif
          #if NUM_RUNOUT_SENSORS > 2
            #if DISABLED(CAN_FILAMENT3_RUNOUT)
              | (READ(FIL_RUNOUT3_PIN) ? _BV(2) : 0)
            #else
              | (TEST(CanModules.Endstop, FILAMENT3)? _BV(2):0)
            #endif
            #if NUM_RUNOUT_SENSORS > 3
              #if DISABLED(CAN_FILAMENT4_RUNOUT)
                | (READ(FIL_RUNOUT4_PIN) ? _BV(3) : 0)
              #else
                | (TEST(CanModules.Endstop, FILAMENT4)? _BV(3):0)
              #endif
              #if NUM_RUNOUT_SENSORS > 4
                #if DISABLED(CAN_FILAMENT5_RUNOUT)
                  | (READ(FIL_RUNOUT5_PIN) ? _BV(4) : 0)
                #else
                  | (TEST(CanModules.Endstop, FILAMENT5)? _BV(4):0)
                #endif
                #if NUM_RUNOUT_SENSORS > 5
                  #if DISABLED(CAN_FILAMENT5_RUNOUT)
                    | (READ(FIL_RUNOUT6_PIN) ? _BV(5) : 0)
                  #else
                    | (TEST(CanModules.Endstop, FILAMENT6)? _BV(5):0)
                  #endif
                #endif
              #endif
            #endif
          #endif
        #endif
      );
    }
    #endif

    // Return a bitmask of runout flag states (1 bits always indicates runout)
    static inline uint8_t poll_runout_states() {
      #if (MOTHERBOARD == BOARD_SNAPMAKER_2_0)
        return printer1->filament_state() ^ uint8_t(
          #if DISABLED(FIL_RUNOUT_INVERTING)
            _BV(NUM_RUNOUT_SENSORS) - 1
          #else
            0
          #endif
        );
      #else
        return poll_runout_pins() ^ uint8_t(
          #if DISABLED(FIL_RUNOUT_INVERTING)
            _BV(NUM_RUNOUT_SENSORS) - 1
          #else
            0
          #endif
        );
      #endif
    }
};

#if ENABLED(FILAMENT_MOTION_SENSOR)

  /**
   * This sensor uses a magnetic encoder disc and a Hall effect
   * sensor (or a slotted disc and optical sensor). The state
   * will toggle between 0 and 1 on filament movement. It can detect
   * filament runout and stripouts or jams.
   */
  class FilamentSensorEncoder : public FilamentSensorBase {
    private:
      static uint8_t motion_detected;

      static inline void poll_motion_sensor() {
        static uint8_t old_state;
        const uint8_t new_state = poll_runout_pins(),
                      change    = old_state ^ new_state;
        old_state = new_state;

        #ifdef FILAMENT_RUNOUT_SENSOR_DEBUG
          if (change) {
            SERIAL_ECHOPGM("Motion detected:");
            for (uint8_t e = 0; e < NUM_RUNOUT_SENSORS; e++)
              if (TEST(change, e)) { SERIAL_CHAR(' '); SERIAL_CHAR('0' + e); }
            SERIAL_EOL();
          }
        #endif

        motion_detected |= change;
      }

    public:
      static inline void block_completed(const block_t* const b) {
        // If the sensor wheel has moved since the last call to
        // this method reset the runout counter for the extruder.
        if (TEST(motion_detected, b->extruder))
          filament_present(b->extruder);

        // Clear motion triggers for next block
        motion_detected = 0;
      }

      static inline void run() { poll_motion_sensor(); }
  };

#else

  /**
   * This is a simple endstop switch in the path of the filament.
   * It can detect filament runout, but not stripouts or jams.
   */
  class FilamentSensorSwitch : public FilamentSensorBase {
    private:
      static inline bool poll_runout_state(const uint8_t extruder) {
        const uint8_t runout_states = poll_runout_states();

        #if NUM_RUNOUT_SENSORS == 1
          UNUSED(extruder);
        #endif

        if (true
          #if NUM_RUNOUT_SENSORS > 1
            #if ENABLED(DUAL_X_CARRIAGE)
              && (dual_x_carriage_mode == DXC_DUPLICATION_MODE || dual_x_carriage_mode == DXC_MIRRORED_MODE)
            #elif ENABLED(MULTI_NOZZLE_DUPLICATION)
              && extruder_duplication_enabled
            #endif
          #endif
        ) return runout_states;               // Any extruder

        #if NUM_RUNOUT_SENSORS > 1
          return TEST(runout_states, extruder); // Specific extruder
        #endif
      }

    public:
      static inline void block_completed(const block_t* const b) { UNUSED(b); }

      static inline void run() {
        const bool out = poll_runout_state(active_extruder);
        if (!out) filament_present(active_extruder);
        //#ifdef FILAMENT_RUNOUT_SENSOR_DEBUG
          static bool was_out = false;
          if (out != was_out) {
            was_out = out;
            SERIAL_ECHOPGM("Filament ");
            serialprintPGM(out ? PSTR("OUT\n") : PSTR("IN\n"));
          }
        //#endif
      }
  };


#endif // !FILAMENT_MOTION_SENSOR

/********************************* RESPONSE TYPE *********************************/

#if FILAMENT_RUNOUT_DISTANCE_MM > 0

  // RunoutResponseDelayed triggers a runout event only if the length
  // of filament specified by FILAMENT_RUNOUT_DISTANCE_MM has been fed
  // during a runout condition.
  class RunoutResponseDelayed {
    private:
      static volatile float runout_mm_countdown[EXTRUDERS];

    public:
      static float runout_distance_mm;

      static inline void reset() {
        LOOP_L_N(i, EXTRUDERS) filament_present(i);
      }

      static inline void run() {
        #ifdef FILAMENT_RUNOUT_SENSOR_DEBUG
          static millis_t t = 0;
          const millis_t ms = millis();
          if (ELAPSED(ms, t)) {
            t = millis() + 1000UL;
            LOOP_L_N(i, EXTRUDERS) {
              serialprintPGM(i ? PSTR(", ") : PSTR("Remaining mm: "));
              SERIAL_ECHO(runout_mm_countdown[i]);
            }
            SERIAL_EOL();
          }
        #endif
      }

      static inline void modify_runout_distance_mm(float distance) {
        #if EXTRUDERS > 1
          runout_mm_countdown[actual_extruder] = runout_distance_mm = distance;
        #else
          runout_mm_countdown[active_extruder] = runout_distance_mm = distance;
        #endif
      }

      static inline bool has_run_out() {
        return runout_mm_countdown[active_extruder] < 0;
      }

      static inline void filament_present(const uint8_t extruder) {
        runout_mm_countdown[extruder] = runout_distance_mm;
      }

      static inline void block_completed(const block_t* const b) {
        if (b->steps[X_AXIS] || b->steps[Y_AXIS] || b->steps[Z_AXIS]
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            || did_pause_print // Allow pause purge move to re-trigger runout state
          #endif
        ) {
          // Only trigger on extrusion with XYZ movement to allow filament change and retract/recover.
          const uint8_t e = b->extruder;
          const int32_t steps = b->steps[E_AXIS];
          runout_mm_countdown[e] -= (TEST(b->direction_bits, E_AXIS) ? -steps : steps) * planner.steps_to_mm[E_AXIS_N(e)];
        }
      }
  };

#else // !FILAMENT_RUNOUT_DISTANCE_MM

  // RunoutResponseDebounced triggers a runout event after a runout
  // condition has been detected runout_threshold times in a row.

  class RunoutResponseDebounced {
    private:
      static constexpr int8_t runout_threshold = 5;
      static int8_t runout_count;
    public:
      static inline void reset()                                  { runout_count = runout_threshold; }
      static inline void run()                                    { runout_count--; }
      static inline bool has_run_out()                            { return runout_count < 0; }
      static inline void block_completed(const block_t* const b)  { UNUSED(b); }
      static inline void filament_present(const uint8_t extruder) { runout_count = runout_threshold; UNUSED(extruder); }
  };

#endif // !FILAMENT_RUNOUT_DISTANCE_MM

/********************************* TEMPLATE SPECIALIZATION *********************************/

typedef TFilamentMonitor<
  #if FILAMENT_RUNOUT_DISTANCE_MM > 0
    #if ENABLED(FILAMENT_MOTION_SENSOR)
      RunoutResponseDelayed, FilamentSensorEncoder
    #else
      RunoutResponseDelayed, FilamentSensorSwitch
    #endif
  #else
    RunoutResponseDebounced, FilamentSensorSwitch
  #endif
> FilamentMonitor;

extern FilamentMonitor runout;

#endif
