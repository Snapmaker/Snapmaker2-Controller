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

enum LevelMode: uint8_t {
  LEVEL_MODE_AUTO = 0,
  LEVEL_MODE_MANUAL,

  LEVEL_MODE_INVALD
};


#define MANUAL_LEVEL_INDEX_INVALID  0x99

#define LIVE_Z_OFFSET_MAX (2.0f)
#define LIVE_Z_OFFSET_MIN (-0.5f)

class BedLevelService {
  public:
    ErrCode DoAutoLeveling(SSTP_Event_t &event);
    ErrCode DoManualLeveling(SSTP_Event_t &event);
    ErrCode SetManualLevelingPoint(SSTP_Event_t &event);
    ErrCode AdjustZOffsetInLeveling(SSTP_Event_t &event);
    ErrCode SaveAndExitLeveling(SSTP_Event_t &event);
    ErrCode ExitLeveling(SSTP_Event_t &event);
    ErrCode IsLeveled(SSTP_Event_t &event);
    ErrCode SyncPointIndex(uint8_t index);

    ErrCode UpdateLiveZOffset(float offset, uint8_t e = 0);
    void ApplyLiveZOffset(uint8_t e = 0);
    void UnapplyLiveZOffset(uint8_t e = 0);
    void SaveLiveZOffset();

    bool live_z_offset_updated() { return live_z_offset_updated_; }
    void live_z_offset(float offset, uint8_t e=0) { if (offset <=LIVE_Z_OFFSET_MAX && offset >=LIVE_Z_OFFSET_MIN) live_z_offset_[e] = offset; }
    float live_z_offset(uint8_t e=0) { return live_z_offset_[e]; }

    ErrCode ProbeSensorCalibrationLeftExtruderAutoProbe();
    ErrCode ProbeSensorCalibrationRightExtruderAutoProbe();
    ErrCode ProbeSensorCalibrationLeftExtruderManualProbe();
    ErrCode ProbeSensorCalibrationRightExtruderManualProbe();
    ErrCode ProbeSensorCalibraitonLeftExtruderPositionConfirm();
    ErrCode ProbeSensorCalibraitonRightExtruderPositionConfirm();
    ErrCode ProbeSensorCalibraitonAbort();
    ErrCode DoDualExtruderAutoLeveling(SSTP_Event_t &event);
    ErrCode DualExtruderAutoLevelingProbePoint(SSTP_Event_t &event);
    ErrCode FinishDualExtruderAutoLeveling(SSTP_Event_t &event);
    ErrCode DoDualExtruderManualLeveling(SSTP_Event_t &event);
    ErrCode DualExtruderManualLevelingProbePoint(SSTP_Event_t &event);
    ErrCode FinishDualExtruderManualLeveling(SSTP_Event_t &event);
    ErrCode DualExtruderAutoBedDetect(SSTP_Event_t &event);
    ErrCode DualExtruderLeftExtruderAutoBedDetect();
    ErrCode DualExtruderRightExtruderAutoBedDetect();
    ErrCode DualExtruderManualBedDetect(SSTP_Event_t &event);
    ErrCode DualExtruderLeftExtruderManualBedDetect();
    ErrCode DualExtruderRightExtruderManualBedDetect();
    ErrCode FinishDualExtruderManualBedDetect();

  private:
    LevelMode level_mode_ = LEVEL_MODE_INVALD;
    uint8_t probe_point_;
    uint8_t manual_level_index_ = MANUAL_LEVEL_INDEX_INVALID;
    float live_z_offset_[EXTRUDERS] = {0};
    float live_z_offset_temp_[EXTRUDERS] = {0};
    bool  live_z_offset_updated_ = false;
    float left_extruder_auto_probe_position_;
    float right_extruder_auto_probe_position_;
    float left_extruder_manual_probe_position_;
    float right_extruder_manual_probe_position_;
    union {
      float MeshPointZ[GRID_MAX_POINTS];
      float z_values_tmp[GRID_MAX_NUM][GRID_MAX_NUM];
    };
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
