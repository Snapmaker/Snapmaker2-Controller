#ifndef QUICKSTOP_SERVICE_H_
#define QUICKSTOP_SERVICE_H_

#include "error.h"
#include "../module/planner.h"
#include "../core/macros.h"


enum QuickStopState : uint8_t {
  QS_STA_IDLE,
  QS_STA_TRIGGERED,
  QS_STA_SAVED_ENV,
  QS_STA_WROTE_FLASH,
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


enum QuickStopCallbackType : uint8_t {
  QS_CB_TYPE_PRE,
  QS_CB_TYPE_POST,

  QS_CB_TYPE_INVALID
};
typedef ErrCode (*QSCallback_t)(QuickStopState sta);

class QuickStopService {
  public:
    void Init();

    bool CheckInISR(block_t *blk);

    void Trigger(QuickStopSource new_source, bool from_isr=false);

    void Process();

    ErrCode RegisterCB(QuickStopSource s, QSCallback_t cb, QuickStopCallbackType t);


  private:
    void Park();


  private:
    QuickStopState state_ = QS_STA_IDLE;
    QuickStopSource source_ = QS_SOURCE_IDLE;
    bool wrote_flash_ = false;

    QSCallback_t cb_pre_[QS_SOURCE_INVALID];
    QSCallback_t cb_post_[QS_SOURCE_INVALID];
};

extern QuickStopService quickstop;

#endif
