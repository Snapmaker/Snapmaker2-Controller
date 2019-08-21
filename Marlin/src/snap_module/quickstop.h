#ifndef QUICKSTOP_H_
#define QUICKSTOP_H_

#include "error.h"
#include "../module/planner.h"
#include "../core/macros.h"

// all events which is able to trigger quick stop
// event triggered in non-ISR:
// 1. door of chamber is opened
// 2. stop button is pressed
// 3. filament run out
// 4. Screen or PC pause
// event triggered in ISR:
// 1. power loss
enum QuickStopEvent : uint8_t {
  QS_EVENT_NONE = 0,
  QS_EVENT_ISR_POWER_LOSS,  // power loss
  QS_EVENT_DOOR_OPEN,       // door open of chamber
  QS_EVENT_BUTTON,          // quick stop button on add-on
  QS_EVENT_RUNOUT,          // filament run out
  QS_EVENT_PAUSE,           // screen or PC pause working
  QS_EVENT_STOP,            // screen or PC stop working
  QS_EVENT_LOST_EXECUTOR,   // executor lost when working
  QS_EVENT_INVALID
};

enum QuickStopSync : uint8_t {
  QS_SYNC_NONE = 0,
  QS_SYNC_TRIGGER,
  QS_SYNC_ISR_END,
  QS_SYNC_INVALID
};

class QuickStop {
  public:
    void Reset();
    bool CheckISR(block_t *blk);
    ErrCode Trigger(QuickStopEvent e);
    void Process();

    bool IsStopped() { return stopped_; };
    bool IsTriggered() { return !!event_; }

    void Debug(QuickStopEvent e) { debug_ = e; }

  private:

    volatile QuickStopEvent event_;
    volatile QuickStopSync sync_flag_;

    volatile bool disable_stepper_;
    volatile bool stopped_;

    QuickStopEvent debug_;

    void CleanMoves();
    void TowardStop();
};

extern QuickStop quickstop;

#endif
