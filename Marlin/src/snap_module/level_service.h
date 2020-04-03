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

class LevelService {
  public:
    ErrCode DoAutoLeveling(Event_t &event);
    ErrCode DoManualLeveling(Event_t &event);
    ErrCode SetManualLevelingPoint(Event_t &event);
    ErrCode AdjustZOffsetInLeveling(Event_t &event);
    ErrCode SaveAndExitLeveling(Event_t &event);
    ErrCode ExitLeveling(Event_t &event);
    ErrCode ResetLeveling(Event_t &event);

    ErrCode SyncPointIndex(uint8_t index);

  private:
    LevelMode level_mode_ = LEVEL_MODE_INVALD;

    uint8_t manual_level_index_ = MANUAL_LEVEL_INDEX_INVALID;

    float MeshPointZ[MESH_POINT_SIZE];
};

extern LevelService levelservice;

#endif  //#ifndef LEVEL_HANDLER_H_
