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
#ifndef SNAPMAKER_TOOLHEAD_CNC_H_
#define SNAPMAKER_TOOLHEAD_CNC_H_

#include "module_base.h"
#include "can_host.h"

class ToolHeadCNC: public ModuleBase {
  public:
		ToolHeadCNC(): ModuleBase(MODULE_DEVICE_ID_CNC) {
      mac_index_ = MODULE_MAC_INDEX_INVALID;
      power_     = 0;
      rpm_       = 0;

      msg_id_set_speed_ = MODULE_MESSAGE_ID_INVALID;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    ErrCode SetOutput(uint8_t power);

    ErrCode TurnOn();
    ErrCode TurnOff();

    bool IsOnline(uint8_t sub_index = 0) { return mac_index_ != MODULE_MAC_INDEX_INVALID; };

    uint32_t mac(uint8_t sub_index = 0) {
      if (sub_index > 0)
        return MODULE_MAC_ID_INVALID;

      return canhost.mac(mac_index_);
    }

    uint16_t rpm() { return rpm_; }
    void rpm(uint16_t rpm) { rpm_ = rpm; }

    uint8_t power() { return power_; }
    void power(uint8_t power) { power_ = power; }

  private:
    uint8_t  mac_index_;
    uint8_t  power_;
    uint16_t rpm_;

    message_id_t msg_id_set_speed_;
};


extern ToolHeadCNC cnc;

#endif  // #ifndef TOOLHEAD_LASER_H_