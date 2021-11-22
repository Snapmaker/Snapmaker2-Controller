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
#ifndef SNAPMAKER_BED_LEVEL_H_
#define SNAPMAKER_BED_LEVEL_H_

#include "../hmi/event_handler.h"

#if EXTRUDERS > 1
  #define Z_LIFT_LENGTH   1

  #define MAIN_SCALE_LINE_INTERVAL    3      // mm
  #define SUB_SCALE_LINE_INTERVAL     2.94   // mm
  #define SCALE_MEASUREMENT_ACCURACY  (MAIN_SCALE_LINE_INTERVAL - SUB_SCALE_LINE_INTERVAL)
  #define MAIN_SCALE_LINES            31
  #define SUB_SCALE_LINES             21
  #define MAIN_SCALE_0_LINE_NUMBER    15
  #define SUB_SCALE_0_LINE_NUMBER     10
  #define SCALE_LONGER_LINE_SEQUENCE  5
  #define MAIN_SUB_SAFE_DISTANCE      0.3 // mm
  #define LEFT_NOZZLE_MIDDLE_LINE_SHIFT 0.4  // mm
  #define RIGHT_NOZZLE_MIDDLE_LINE_SHIFT 0.4 // mm
  #define FIRST_SCALE_LINE_TO_BORDER  4   // mm

  #define SCALE_0_LINE_LENGTH         25  // mm
  #define SCALE_LINE_LENGTH_LONGER    20  // mm
  #define SCALE_LINE_LENGHT_NORMAL    15  // mm

  #define E_RETRACTION_LENGTH         2   // 0.2 // mm
  #define E_RETRACTION_PAUSE_LENGTH   7   // mm
  #define PRE_EXTRUSION_LENGTH        9   // mm
  #define PRE_EXTRUSION_SPEED         20  // mm/s
  #define RETRACTION_SPEED            40  // mm/s

  #define X_CALIBRATION_A350_START_POINT_XYZ {90.0, 200.0, 0.2}  // mm
  #define X_CALIBRATION_A250_START_POINT_XYZ {40.0, 175.0, 0.2}
  #define X_CALIBRATION_A150_START_POINT_XYZ {-13.0, 60.0, 0.2}

  #define Y_CALIBRATION_A350_START_POINT_XYZ {110.0, 50.0, 0.2}
  #define Y_CALIBRATION_A250_START_POINT_XYZ {100.0, 70.0, 0.2}
  #define Y_CALIBRATION_A150_START_POINT_XYZ {10.0, 50.0, 0.2}

  #define X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH  60  // mm
  #define X_CALIBRATION_UP_DOWN_LINE_LENGTH     98  // mm
  #define Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH  98  // mm
  #define Y_CALIBRATION_UP_DOWN_LINE_LENGTH     60  // mm

  #define E_MOVES_FACTOR  0.05 // 0.03326
#endif

typedef struct {
  int16_t extruder0_temp;
  int16_t extruder1_temp;
  int16_t bed_temp;
  float e_factor;
  float layer_height;
}xy_calibration_param_t;

enum LevelMode: uint8_t {
  LEVEL_MODE_AUTO = 0,
  LEVEL_MODE_MANUAL,
  LEVEL_MODE_AUTO_NO_ADJUST,

  LEVEL_MODE_INVALD
};


#define MANUAL_LEVEL_INDEX_INVALID  0x99
#define MESH_POINT_SIZE             25

#define LIVE_Z_OFFSET_MAX (2.0f)
#define LIVE_Z_OFFSET_MIN (-0.5f)

class BedLevelService {
  public:
    ErrCode CalibrateExtruderTriggerStroke(float x, float y);
    ErrCode ConfirmExtruderTriggerStroke(uint8_t extruder_index);
    ErrCode DoXCalibration(xy_calibration_param_t &cal_param);
    ErrCode ApplyXCalibration(float lines);
    ErrCode DoYCalibration(xy_calibration_param_t &cal_param);
    ErrCode ApplyYCalibration(float lines);

    ErrCode DoAutoLeveling(SSTP_Event_t &event);
    ErrCode DoManualLeveling(SSTP_Event_t &event);
    ErrCode SetManualLevelingPoint(SSTP_Event_t &event);
    ErrCode AdjustZOffsetInLeveling(SSTP_Event_t &event);
    ErrCode SaveAndExitLeveling(SSTP_Event_t &event);
    ErrCode ExitLeveling(SSTP_Event_t &event);
    ErrCode IsLeveled(SSTP_Event_t &event);
    ErrCode SyncPointIndex(uint8_t index);

    ErrCode UpdateLiveZOffset(float offset, uint8_t e=0);
    void ApplyLiveZOffset(uint8_t e=0);
    void UnapplyLiveZOffset(uint8_t e=0);
    void SaveLiveZOffset();

    bool live_z_offset_updated() { return live_z_offset_updated_; }
    void live_z_offset(float offset, uint8_t e=0) { if (offset <=LIVE_Z_OFFSET_MAX && offset >=LIVE_Z_OFFSET_MIN) live_z_offset_[e] = offset; }
    float live_z_offset(uint8_t e=0) { return live_z_offset_[e]; }

  private:
    LevelMode level_mode_ = LEVEL_MODE_INVALD;
    uint8_t manual_level_index_ = MANUAL_LEVEL_INDEX_INVALID;
    float live_z_offset_[2] = {0};
    bool  live_z_offset_updated_ = false;
    float MeshPointZ[MESH_POINT_SIZE];
    float extruder0_manual_level_z_;
    float extruder1_manual_level_z_;
};

extern BedLevelService levelservice;


#define LEVEL_SERVICE_EEPROM_PARAM  float live_z_offset[EXTRUDERS]

#define LEVEL_SERVICE_EEPROM_READ() do { \
                                        float live_z_offset[EXTRUDERS]; \
                                        for (uint32_t i=0; i<EXTRUDERS; i++) { \
                                          _FIELD_TEST(live_z_offset[i]); \
                                          EEPROM_READ(live_z_offset[i]); \
                                          levelservice.live_z_offset(live_z_offset[i], i); \
                                         } \
                                    } while (0)

#define LEVEL_SERVICE_EEPROM_WRITE() do { \
                                      float live_z_offset[EXTRUDERS]; \
                                      for (uint8_t i=0; i<EXTRUDERS; i++) { \
                                        live_z_offset[i] = levelservice.live_z_offset(i); \
                                        _FIELD_TEST(live_z_offset[i]); \
                                        EEPROM_WRITE(live_z_offset[i]); \
                                      } \
                                    } while (0)

#define LEVEL_SERVICE_EEPROM_RESET()  do { \
                                        for (uint8_t i=0; i<EXTRUDERS; i++) { \
                                          levelservice.live_z_offset(0, i); \
                                        } \
                                      } while (0)

#endif  //#ifndef SNAPMAKER_BED_LEVEL_H_
