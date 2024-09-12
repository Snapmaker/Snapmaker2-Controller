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
#ifndef SNAPMAKER_ENCLOSURE_H_
#define SNAPMAKER_ENCLOSURE_H_

#include "module_base.h"
#include "can_host.h"


#define ENCLOSURE_LIGHT_POWER_DEFAULT (255)
#define ENCLOSURE_FAN_SPEED_DEFAULT   (255)

#define ENCLOSURE_DOOR_STATE_OPEN     (1)
#define ENCLOSURE_DOOR_STATE_CLOSED   (0)

#define ENCLOSURE_DOOR_CHECK_DEFAULT  true

enum EnclosureEventState {
  ENCLOSURE_EVENT_STATE_IDLE,
  ENCLOSURE_EVENT_STATE_OPENED,
  ENCLOSURE_EVENT_STATE_HANDLED_OPEN,
  ENCLOSURE_EVENT_STATE_CLOSED,

  ENCLOSURE_EVENT_STATE_INVALID
};


class Enclosure: public ModuleBase {
  public:
    Enclosure(): ModuleBase(MODULE_DEVICE_ID_ENCLOSURE) {
      door_state_  = ENCLOSURE_DOOR_STATE_CLOSED;
      mac_index_   = MODULE_MAC_INDEX_INVALID;
      enabled_     = ENCLOSURE_DOOR_CHECK_DEFAULT;
      fan_speed_   = 0;
      brightness_  = 0;
      event_state_ = ENCLOSURE_EVENT_STATE_IDLE;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);
    ErrCode PostInit();
    ErrCode SetLightBar(uint8_t brightness);
    ErrCode SetFanSpeed(uint8_t speed);

    bool IsOnline(uint8_t sub_index = 0) { return (mac_index_ != MODULE_MAC_INDEX_INVALID); }
    bool DoorOpened() { return door_state_ == ENCLOSURE_DOOR_STATE_OPEN && enabled_; }

    void Disable();
    void Enable();
    void ReportStatus();
    void Process();

    void PollDoorState();

    // callback for HMI events
    ErrCode ReportStatus(SSTP_Event_t &event);
    ErrCode SetFan(SSTP_Event_t &event);
    ErrCode SetLightBar(SSTP_Event_t &event);
    ErrCode SetDetection(SSTP_Event_t &event);

    uint32_t mac(uint8_t sub_index = 0) { return canhost.mac(mac_index_); }

    uint8_t door_state() { return door_state_; };
    void    door_state(uint8_t state) { door_state_ = state; }

  private:
    void HandleDoorOpened();
    void HandleDoorClosed();


  private:
    uint8_t mac_index_;

    uint8_t fan_speed_;
    uint8_t brightness_;
    uint8_t door_state_;

    uint8_t event_state_;
  public:
    bool    enabled_;
};


extern Enclosure enclosure;

#endif
