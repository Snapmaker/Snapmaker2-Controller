#ifndef QUICKSTOP_SERVICE_H_
#define QUICKSTOP_SERVICE_H_

#include "error.h"
#include "../module/planner.h"
#include "../core/macros.h"


enum QuickStopState : uint8_t {
  QS_STA_IDLE,
  QS_STA_TRIGGERED,
  QS_STA_CLEAN_MOVES,
  QS_STA_SAVED_ENV,
  QS_STA_STOPPED,
  QS_STA_PARKING,

  QS_STA_INVALID
};


enum QuickStopSource : uint8_t {
  QS_SOURCE_IDLE,
  QS_SOURCE_PAUSE,
  QS_SOURCE_STOP,
  QS_SOURCE_POWER_LOSS,

  QS_SOURCE_INVALID
};


class QuickStopService {
  public:
    void Init();

    bool CheckInISR(block_t *blk) AT_SNAP_SECTION;

    void Trigger(QuickStopSource new_source, bool from_isr=false);

    void Process();

    bool inline isTriggered() { return source_ != QS_SOURCE_IDLE; }
    bool inline isIdle() { return source_ == QS_SOURCE_IDLE; }

  private:
    void Park();


  private:
    QuickStopState state_ = QS_STA_IDLE;
    QuickStopSource source_ = QS_SOURCE_IDLE;
    QuickStopSource pre_source_ = QS_SOURCE_IDLE;
    bool wrote_flash_ = false;
};

extern QuickStopService quickstop;

#endif
