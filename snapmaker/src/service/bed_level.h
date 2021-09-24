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
#define MESH_POINT_SIZE             25

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

    ErrCode UpdateLiveZOffset(float offset);
    void ApplyLiveZOffset();
    void UnapplyLiveZOffset();
    void SaveLiveZOffset();

    bool live_z_offset_updated() { return live_z_offset_updated_; }
    void live_z_offset(float offset) { if (offset <=LIVE_Z_OFFSET_MAX && offset >=LIVE_Z_OFFSET_MIN) live_z_offset_ = offset; }
    float live_z_offset() { return live_z_offset_; }

  private:
    LevelMode level_mode_ = LEVEL_MODE_INVALD;

    uint8_t manual_level_index_ = MANUAL_LEVEL_INDEX_INVALID;

    float live_z_offset_ = 0;
    bool  live_z_offset_updated_ = false;

    float MeshPointZ[MESH_POINT_SIZE];
};

extern BedLevelService levelservice;


#define LEVEL_SERVICE_EEPROM_PARAM  float live_z_offset

#define LEVEL_SERVICE_EEPROM_READ() do { \
                                        float live_z_offset; \
                                        _FIELD_TEST(live_z_offset); \
                                        EEPROM_READ(live_z_offset); \
                                        levelservice.live_z_offset(live_z_offset); \
                                    } while (0)

#define LEVEL_SERVICE_EEPROM_WRITE() do { \
                                      float live_z_offset; \
                                      live_z_offset = levelservice.live_z_offset(); \
                                      _FIELD_TEST(live_z_offset); \
                                      EEPROM_WRITE(live_z_offset); \
                                    } while (0)

#define LEVEL_SERVICE_EEPROM_RESET()  do { \
                                        levelservice.live_z_offset(0); \
                                      } while (0)

#endif  //#ifndef SNAPMAKER_BED_LEVEL_H_
