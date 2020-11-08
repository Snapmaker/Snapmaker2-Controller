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
#ifndef HEARTBEAT_WATCH_H_
#define HEARTBEAT_WATCH_H_

#include "src/Marlin.h"

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
