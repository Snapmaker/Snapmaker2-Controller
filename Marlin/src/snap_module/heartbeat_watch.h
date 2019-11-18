#ifndef HEARTBEAT_WATCH_H_
#define HEARTBEAT_WATCH_H_

#include "../../Marlin.h"

/* Common implement to check heartbeat
 *  refer to heater watch in thermalmanager
 * */
#define HB_STA_ALIVE          0
#define HB_STA_JUST_ALIVE     1
#define HB_STA_JUST_DEAD      2
#define HB_STA_ALREADY_DEAD   3

#define  HB_DEAD_MAX_DEFAULT  3

class HeartbeatWatch {
  public:

  void Init(uint8_t debounce_dead, uint16_t period) {
    dead_cnt_max_ = debounce_dead;
    period_ = period;
  }

  void Start() { next_ms_ = millis() + period_; }
  void Stop() { next_ms_ = 0; }

  inline void IamAlive() { alive_ = true; }

  inline uint8_t CheckAlive() {
    uint8_t status = HB_STA_ALIVE;
    if (!next_ms_)
      return status;

    if (Elapsed(millis())) {
      if (!alive_) {
        // to do a debounce
        if (++dead_cnt_ == dead_cnt_max_)
          status = HB_STA_JUST_DEAD;
        else if (dead_cnt_ > dead_cnt_max_) {
          dead_cnt_ = dead_cnt_max_;
          status = HB_STA_ALREADY_DEAD;
        }
      }
      else {
        if (dead_cnt_ >= dead_cnt_max_) {
          if (++dead_cnt_ <= 2*dead_cnt_max_)
            status = HB_STA_ALREADY_DEAD;
          else
            dead_cnt_ = 0;
        }
        else {
          dead_cnt_ = 0;
        }
      }

      alive_ = false;
      next_ms_ = millis() + period_;
    }

    return status;
  }

  private:
    inline bool Elapsed(const millis_t &ms) { return next_ms_ && ELAPSED(ms, next_ms_); }

  private:
    millis_t  next_ms_ = 0;
    uint16_t  period_ = 0;
    uint8_t   dead_cnt_ = 0;
    uint8_t   dead_cnt_max_ = HB_DEAD_MAX_DEFAULT;
    bool      alive_ = true;
};

#endif
