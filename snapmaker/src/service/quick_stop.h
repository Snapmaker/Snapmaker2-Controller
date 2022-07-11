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
#ifndef SNAPMAKER_QUICK_STOP_H_
#define SNAPMAKER_QUICK_STOP_H_

#include "../common/error.h"
#include "../common/config.h"

#include "src/module/planner.h"
#include "src/core/macros.h"


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
  QS_SOURCE_STOP_BUTTON,
  QS_SOURCE_SECURITY,

  QS_SOURCE_INVALID
};


class QuickStopService {
  public:
    void Init();

    bool CheckInISR(block_t *blk);

    void Trigger(QuickStopSource new_source, bool from_isr=false);
    void HandleProtection();
    void EmergencyStop();
    void Process();

    bool inline isTriggered() { return source_ != QS_SOURCE_IDLE; }
    bool inline isIdle() { return source_ == QS_SOURCE_IDLE; }
    bool isPowerLoss() {return source_ == QS_SOURCE_POWER_LOSS;}

  private:
    void Park();
    void TurnOffPower();

  private:
    QuickStopState state_ = QS_STA_IDLE;
    QuickStopSource source_ = QS_SOURCE_IDLE;
    QuickStopSource pre_source_ = QS_SOURCE_IDLE;
    bool wrote_flash_ = false;
    bool  homing_is_interrupted_ = false;  // Homing process triggers stop
};

extern QuickStopService quickstop;

#endif  // #ifndef SNAPMAKER_QUICK_STOP_H_
