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

#ifndef SNAPMAKER_PURIFIER_H_
#define SNAPMAKER_PURIFIER_H_
#include "module_base.h"


typedef enum : uint8_t{
  PURIFIER_FAN_GEARS_0,  // off
  PURIFIER_FAN_GEARS_1,
  PURIFIER_FAN_GEARS_2,
  PURIFIER_FAN_GEARS_3,
}PURIFIER_FAN_GEARS_E;

typedef enum : uint8_t{
  PURIFIER_INFO_LIFETIME,
  PURIFIER_INFO_ERR,
  PURIFIER_INFO_FAN_STA,
  PURIFIER_INFO_FAN_ELEC,
  PURIFIER_REPORT_POWER,
  PURIFIER_REPORT_STATUS,
  PURIFIER_INFO_ALL,
}PURIFIER_INFO_E;

enum {
  PURIFIER_SET_FAN_STA,
  PURIFIER_SET_FAN_GEARS,
  PURIFIER_SET_FAN_POWER,
  PURIFIER_SET_LIGHT,
};

typedef enum : uint8_t {
  LIFETIME_LOW,
  LIFETIME_MEDIUM,
  LIFETIME_NORMAL,
}PURIFIER_LIFETIME_E;

typedef enum : uint8_t  {
  ERR_ADDON_POWER,
  ERR_EXTEND_POWER,
  ERR_FAN_SPEED_TOO_LOW,
  ERR_NO_FILTER,
  ERR_ELEC_TOO_HIGH,
  ERR_EMERGENCY_STOP,
}PURIFIER_ERR_E;

enum {
  PURIFIER_OFFLINE,
  PURIFIER_ONLINE,
};

typedef struct {
  uint16_t fan_speed;
  uint16_t fan_elec;
  uint16_t addon_power;
  uint16_t extend_power;
  PURIFIER_LIFETIME_E lifetime;
  uint8_t  fan_cur_out;  // Actual Output
  uint8_t  fan_gears;
  uint8_t  is_work;
  uint8_t  sys_status_encode;
  uint8_t  err;
} PurifierInfo_t;

class Purifier : public ModuleBase{
 public:
  Purifier(): ModuleBase(MODULE_DEVICE_ID_PURIFIER) {
    online_ = PURIFIER_OFFLINE;
  }
  ErrCode Init(MAC_t &mac, uint8_t mac_index);
  PurifierInfo_t GetInfo(PURIFIER_INFO_E info, uint16_t timeout_ms=200);
  void UpdateInfo(uint8_t data[8]);
  void UpdateLifetime(uint8_t lifetime) {info_.lifetime = (PURIFIER_LIFETIME_E)lifetime;}
  void UpdateFanStatus(uint8_t is_work, uint16_t speed, uint8_t fan_out, uint8_t gears) {
    info_.is_work = is_work;
    info_.fan_speed = speed; 
    info_.fan_cur_out = fan_out;
    info_.fan_gears = gears;
  }
  void UpdateElectricity(uint16_t fan_elec) {info_.fan_elec = fan_elec;}
  void UpdatePower(uint16_t addon, uint16_t extend) {info_.addon_power = addon; info_.extend_power = extend;}
  void UpdateErr(uint8_t err) {info_.err = err;}
  void SetFanStatus(uint8_t is_open, uint16_t delay_close_s=0, uint8_t is_forced=false);
  ErrCode ReportStatus();
  ErrCode ReportLifetimeStatus();
  void SetFanGears(uint8_t gears);
  void SetFanPower(uint8_t power);
  void SetLightColor(uint8_t rgb[3]);
  void DisplayInfo();
  void DisplayErrInfo(uint8_t err);
  void DisplaySysStatus();
  bool IsOnline() {return online_ == PURIFIER_ONLINE;}
  void ErrCheckLoop();
  void SendCmd(uint16_t funcid, uint8_t *data, uint8_t len);
  void Process();

 private:
  PurifierInfo_t info_= {0};
  uint8_t online_;
  volatile uint8_t update_info_flag_ = 0;
  uint8_t last_err_ = 0;
  uint8_t last_lifetime_ = 0;
  uint8_t mac_index_;
  uint32_t delay_close_ = 0;
};

extern Purifier purifier;

#endif
