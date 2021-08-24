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
#ifndef SNAPMAKER_TOOLHEAD_3DP_H_
#define SNAPMAKER_TOOLHEAD_3DP_H_

#include "module_base.h"
#include "can_host.h"

#include "../common/config.h"

#define TOOLHEAD_3DP_FAN_MAX    (2)

#define EXTRUDERS 1

class ToolHead3DP: public ModuleBase {
  public:
    ToolHead3DP(ModuleDeviceID id): ModuleBase(id) {
      for (int i = 0; i < EXTRUDERS; i++) {
        cur_temp_[i]  = 0;
      }

      for (int i = 0; i < TOOLHEAD_3DP_FAN_MAX; i++) {
        fan_speed_[i] = 0;
      }
      mac_index_      = MODULE_MAC_INDEX_INVALID;
      probe_state_    = 0;

      timer_in_process_ = 0;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    ErrCode SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time=0);
    ErrCode SetPID(uint8_t index, float value, uint8_t extrude_index=0);
    float * GetPID(uint8_t extrude_index=0);
    void UpdatePID(uint8_t index, float val) {if (index < 3) pid_[index]=val;};
    ErrCode SetHeater(uint16_t target_temp, uint8_t extrude_index=0);
    void GetFilamentState();
    void Process();

    bool IsOnline(uint8_t head_index=0) { return mac_index_ != MODULE_MAC_INDEX_INVALID; };

    uint32_t mac(uint8_t head_index=0) {
      return canhost.mac(mac_index_);
    }

    uint8_t fan_speed(uint8_t fan_index) {
      if (fan_index >= TOOLHEAD_3DP_FAN_MAX)
        return 0;

      return fan_speed_[fan_index];
    }

    void probe_state(uint8_t state, uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return;

      if (state)
        probe_state_ |= (1<<extrude_index);
      else
        probe_state_ &= ~(1<<extrude_index);
    }
    uint8_t probe_state() {
      return probe_state_;
    }

    void filament_state(uint8_t state, uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return;

      if (state)
        filament_state_ |= (1<<extrude_index);
      else
        filament_state_ &= ~(1<<extrude_index);
    }
    uint8_t filament_state() {
      return filament_state_;
    }

    void SetTemp(int16_t temp, uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return;

      cur_temp_[extrude_index] = temp;
    }
    int16_t GetTemp(uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return 0;

      return cur_temp_[extrude_index];
    }

  private:
    void IOInit(void);

  private:
    uint8_t mac_index_;

    uint16_t timer_in_process_;

    int16_t cur_temp_[EXTRUDERS];
    uint8_t  fan_speed_[TOOLHEAD_3DP_FAN_MAX];

    // 1 bit indicates one sensor
    uint8_t probe_state_;
    uint8_t filament_state_;
    float pid_[3];
};

extern ToolHead3DP *printer1;

#endif  // #ifndef TOOLHEAD_3DP_H_
