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
#include "../common/debug.h"

#define TOOLHEAD_3DP_FAN_MAX      (2)
#define TOOLHEAD_3DP_EXTRUDER_MAX (2)

#define TOOLHEAD_3DP_EXTRUDER0    (1)
#define TOOLHEAD_3DP_EXTRUDER1    (0)

class ToolHead3DP: public ModuleBase {
  public:
    ToolHead3DP(ModuleDeviceID id): ModuleBase(id) {
      for (int i = 0; i < TOOLHEAD_3DP_EXTRUDER_MAX; i++) {
        cur_temp_[i]  = 0;
        target_temp_[i] = 0;
      }

      for (int i = 0; i < TOOLHEAD_3DP_FAN_MAX; i++) {
        fan_speed_[i] = 0;
      }
      mac_index_      = MODULE_MAC_INDEX_INVALID;
      cur_extruder_   = TOOLHEAD_3DP_EXTRUDER0;
      probe_state_    = 0;

      timer_in_process_ = 0;

      msg_id_swtich_extruder_ = MODULE_MESSAGE_ID_INVALID;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    ErrCode SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time=0);
    ErrCode SetPID(uint8_t index, float value, uint8_t extrude_index=0);
    ErrCode SetHeater(uint16_t target_temp, uint8_t extrude_index=0);
    ErrCode SwitchExtruder(uint8_t extrude_index);

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

    void SetProbeState(uint8_t state[]);
    bool GetProbeState(uint8_t extruder=0);
    uint8_t probe_state() { return probe_state_; }

    void SetFilamentState(uint8_t state[]);
    bool GetFilamentState(uint8_t extruder=0);
    uint8_t filament_state() { return filament_state_; }

    void SetTemp(uint8_t temp[]) {
      cur_temp_[0] = temp[0]<<8|temp[1];

      if (device_id_ == MODULE_DEVICE_ID_3DP_DUAL) {
        cur_temp_[1] = temp[4]<<8|temp[5];
      }
    }
    uint16_t GetTemp(uint8_t extrude_index=0) {
      if (extrude_index >= TOOLHEAD_3DP_EXTRUDER_MAX)
        return 0;

      return cur_temp_[extrude_index];
    }

  private:
    void IOInit(void);

  private:
    uint8_t mac_index_;
    uint8_t cur_extruder_;

    uint16_t timer_in_process_;

    uint16_t cur_temp_[TOOLHEAD_3DP_EXTRUDER_MAX];
    uint16_t target_temp_[TOOLHEAD_3DP_EXTRUDER_MAX];
    uint8_t  fan_speed_[TOOLHEAD_3DP_FAN_MAX];

    // 1 bit indicates one sensor
    uint8_t probe_state_;
    uint8_t filament_state_;

    message_id_t msg_id_swtich_extruder_;
};

extern ToolHead3DP *printer1;

#endif  // #ifndef TOOLHEAD_3DP_H_
