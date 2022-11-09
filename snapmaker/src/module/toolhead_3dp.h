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

#define TOOLHEAD_3DP_FAN_MAX    (3)

#define EXTRUDERS 2

typedef enum {
  PROBE_SENSOR_PROXIMITY_SWITCH,
  PROBE_SENSOR_LEFT_OPTOCOUPLER,
  PROBE_SENSOR_RIGHT_OPTOCOUPLER,
  PROBE_SENSOR_LEFT_CONDUCTIVE,
  PROBE_SENSOR_RIGHT_CONDUCTIVE,
  PROBE_SENSOR_INVALID,
}probe_sensor_t;

typedef enum {
  GO_HOME,
  MOVE_SYNC,
  MOVE_ASYNC,
}move_type_e;

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

    virtual ErrCode SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time=0);
    ErrCode SetPID(uint8_t index, float value, uint8_t extrude_index=0);
    float * GetPID(uint8_t extrude_index=0);
    void UpdatePID(uint8_t index, float val) {if (index < 3) pid_[index]=val;};
    virtual ErrCode SetHeater(uint16_t target_temp, uint8_t extrude_index=0);
    void GetFilamentState();
    virtual void Process();

    bool IsOnline(uint8_t head_index=0) { return mac_index_ != MODULE_MAC_INDEX_INVALID; };

    uint32_t mac(uint8_t head_index=0) {
      return canhost.mac(mac_index_);
    }

    uint8_t fan_speed(uint8_t fan_index) {
      if (fan_index >= TOOLHEAD_3DP_FAN_MAX)
        return 0;

      return fan_speed_[fan_index];
    }

    void report_probe_state(uint8_t state, uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return;

      if (state)
        probe_state_ |= (1<<extrude_index);
      else
        probe_state_ &= ~(1<<extrude_index);
    }
    virtual bool probe_state() {
      return (bool)probe_state_;
    }
    virtual bool probe_state(probe_sensor_t sensor) {
      return (bool)probe_state_;
    }

    void ResetFilamentState(uint8_t state, uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return;

      if (state)
        filament_state_ |= (1<<extrude_index);
      else
        filament_state_ &= ~(1<<extrude_index);
    }

    void report_filament_state(uint8_t state, uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return;

      if (state)
        filament_state_ |= (1<<extrude_index);
      else
        filament_state_ &= ~(1<<extrude_index);
    }
    virtual bool filament_state() {
      return (bool)filament_state_;
    }
    virtual bool filament_state(uint8_t e) {
      return (bool)filament_state_;
    }

    void SetTemp(int16_t temp, uint8_t extrude_index=0);
    int16_t GetTemp(uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return 0;

      return cur_temp_[extrude_index];
    }

    void UpdateEAxisStepsPerUnit(ModuleToolHeadType type);

    virtual void UpdateHotendMaxTemp(int16_t temp, uint8_t e = 0);

    // for dualextruder
    virtual ErrCode ToolChange(uint8_t new_extruder, bool use_compensation = true) { return E_SUCCESS; }
    virtual void SelectProbeSensor(probe_sensor_t sensor) { return; }
    virtual void SetZCompensation(float comp, uint32_t e = 0) { return; }
    void GetZCompensation(float &left_z_compensation, float &right_z_compensation) { return; }
    virtual ErrCode ModuleCtrlProximitySwitchPower(uint8_t state) { return E_SUCCESS; }
    virtual ErrCode ModuleCtrlProbeStateSync() { return E_SUCCESS; }
    virtual ErrCode ModuleCtrlSetPid(float p, float i, float d) { return E_SUCCESS; }
    virtual ErrCode ModuleCtrlToolChange(uint8_t new_extruder) { return E_SUCCESS; }
    virtual ErrCode ModuleCtrlSetExtruderChecking(bool on_off) { return E_SUCCESS; }
    virtual ErrCode ModuleCtrlSaveHotendOffset(float offset, uint8_t axis) { return E_SUCCESS; }
    virtual ErrCode ModuleCtrlRightExtruderMove(move_type_e type, float destination = 0) { return E_SUCCESS; }
    virtual ErrCode ModuleCtrlSetRightExtruderPosition(float raise_for_home_pos, float z_max_pos) { return E_SUCCESS; }
    virtual ErrCode HmiGetHotendType(SSTP_Event_t &event) { return E_SUCCESS; }
    virtual ErrCode HmiGetFilamentState(SSTP_Event_t &event) { return E_SUCCESS; }
    virtual ErrCode HmiGetHotendTemp(SSTP_Event_t &event) { return E_SUCCESS; }
    virtual ErrCode HmiRequestToolChange(SSTP_Event_t &event) { return E_SUCCESS; }
    virtual ErrCode HmiSetFanSpeed(SSTP_Event_t &event) { return E_SUCCESS; }
    virtual ErrCode HmiSetHotendOffset(SSTP_Event_t &event) { return E_SUCCESS; }
    virtual ErrCode HmiGetHotendOffset(SSTP_Event_t &event) { return E_SUCCESS; }
    virtual ErrCode HmiRequestGetActiveExtruder(SSTP_Event_t &event) { return E_SUCCESS; }

    virtual void ShowInfo() {}

  protected:
    void IOInit(void);

  protected:
    int16_t cur_temp_[EXTRUDERS];
    uint8_t  fan_speed_[TOOLHEAD_3DP_FAN_MAX];
    // 1 bit indicates one sensor
    uint8_t probe_state_ = 0;
    uint8_t filament_state_;
    float pid_[3];
    uint16_t timer_in_process_;

  private:
    uint8_t mac_index_;
};

extern ToolHead3DP printer_single;
extern ToolHead3DP *printer1;

#endif  // #ifndef TOOLHEAD_3DP_H_
