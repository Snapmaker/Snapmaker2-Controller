#ifndef LEVEL_HANDLER_H_
#define LEVEL_HANDLER_H_

#include "event_handler.h"

enum LevelMode: uint8_t {
  LEVEL_MODE_AUTO = 0,
  LEVEL_MODE_MANUAL,

  LEVEL_MODE_INVALD
};


#define MANUAL_LEVEL_INDEX_INVALID  0x99
#define MESH_POINT_SIZE             25

#define LIVE_Z_OFFSET_MAX (2.0f)
#define LIVE_Z_OFFSET_MIN (-0.5f)

class LevelService {
  public:
    ErrCode DoAutoLeveling(Event_t &event);
    ErrCode DoManualLeveling(Event_t &event);
    ErrCode SetManualLevelingPoint(Event_t &event);
    ErrCode AdjustZOffsetInLeveling(Event_t &event);
    ErrCode SaveAndExitLeveling(Event_t &event);
    ErrCode ExitLeveling(Event_t &event);
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

extern LevelService levelservice;


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

#endif  //#ifndef LEVEL_HANDLER_H_
