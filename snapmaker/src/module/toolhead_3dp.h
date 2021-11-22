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
#include "../../../Marlin/src/module/motion.h"

#define TOOLHEAD_3DP_FAN_MAX    (3)

#define TOOLHEAD_3DP_EXTRUDER0    0
#define TOOLHEAD_3DP_EXTRUDER1    1

typedef enum {
  SINGLE_EXTRUDER_MODULE_FAN       = 0,
  SINGLE_EXTRUDER_NOZZLE_FAN       = 1,
  DUAL_EXTRUDER_LEFT_MODULE_FAN    = 0,
  DUAL_EXTRUDER_RIGHT_MODULE_FAN   = 1,
  DUAL_EXTRUDER_NOZZLE_FAN         = 2,
}fan_e;

typedef enum {
  PROBE_SENSOR_PROXIMITY_SWITCH,
  PROBE_SENSOR_LEFT_OPTOCOUPLER,
  PROBE_SENSOR_RIGHT_OPTOCOUPLER,
  PROBE_SENSOR_LEFT_CONDUCTIVE,
  PROBE_SENSOR_RIGHT_CONDUCTIVE,
  PROBE_SENSOR_INVALID,
}probe_sensor_t;

typedef enum {
  NOZZLE_TYPE_0,
  NOZZLE_TYPE_1,
  NOZZLE_TYPE_2,
  NOZZLE_TYPE_3,
  NOZZLE_TYPE_4,
  NOZZLE_TYPE_5,
  NOZZLE_TYPE_6,
  NOZZLE_TYPE_7,
  NOZZLE_TYPE_8,
  NOZZLE_TYPE_9,
  NOZZLE_TYPE_10,

  NOZZLE_TYPE_IDLE,
  NOZZLE_TYPE_INVALID = 0xff,
}nozzle_type_t;

typedef enum {
    EXTRUDER_STATUS_CHECK,
    EXTRUDER_STATUS_IDLE,
    EXTRUDER_STATUS_ERROR,
}extruder_status_e;

class ToolHead3DP: public ModuleBase {
  public:
    ToolHead3DP(ModuleDeviceID id): ModuleBase(id) {
      for (int i = 0; i < EXTRUDERS; i++) {
        cur_temp_[i]  = 0;
      }

      for (int i = 0; i < TOOLHEAD_3DP_FAN_MAX; i++) {
        fan_speed_[i] = 0;
      }
      mac_index_           = MODULE_MAC_INDEX_INVALID;
      probe_state_         = 0;
      active_probe_sensor_ = PROBE_SENSOR_PROXIMITY_SWITCH;
      extruder_status_     = 0;
      timer_in_process_    = 0;
      nozzle_type_[0]      = NOZZLE_TYPE_IDLE;
      nozzle_type_[1]      = NOZZLE_TYPE_IDLE;
      need_to_tell_hmi_nozzle_type_ = false;
      need_to_tell_hmi_extruder_info_ = false;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    ErrCode SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time=0);
    ErrCode SetPID(uint8_t index, float value, uint8_t extrude_index=0);
    float * GetPID(uint8_t extrude_index=0);
    void UpdatePID(uint8_t index, float val) {if (index < 3) pid_[index]=val;};
    ErrCode SetHeater(uint16_t target_temp, uint8_t extrude_index=0);
    void GetFilamentState();
    void SetNozzleType(uint8_t *data);
    void SetExtruderInfo(uint8_t *data);
    nozzle_type_t GetNozzleType(uint8_t e);
    void NozzleStatusCheck();
    ErrCode SwitchExtruder(uint8_t extrude_index);
    void SwitchExtruderWithoutMove(uint8_t extruder_index);
    void SetProbeSensor(probe_sensor_t sensor);
    ErrCode SendNozzleTypeToHmi();
    ErrCode SendExtruderInfoToHmi();
    ErrCode SendFilamentStateToHmi();
    ErrCode SendNozzleTempToHmi();
    void SetExtruderCheck(extruder_status_e status);
    ErrCode ExtruderStatusCtrl(SSTP_Event_t &event);
    ErrCode ExtruderStrokeCalibration(SSTP_Event_t &event);
    ErrCode ExtruderStrokeConfirm(SSTP_Event_t &event);
    ErrCode SetHotendOffset(SSTP_Event_t &event);
    ErrCode SetFanSpeed(SSTP_Event_t &event);
    ErrCode LeftStopperAssemble(SSTP_Event_t &event);
    ErrCode RightStopperAssemble(SSTP_Event_t &event);
    ErrCode DoXYCalibration(SSTP_Event_t &event);
    ErrCode XYCalibrationConfirm(SSTP_Event_t &event);
    void UpdateFilamentStatus();
    void UpdateProbeSensorStatus();
    void UpdateNozzleType();
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

    void probe_state(uint8_t state[]) {
      if (device_id_ == MODULE_DEVICE_ID_DUAL_EXTRUDER) {
        if (state[0]) {
          probe_state_ |= 0x01;
        } else {
          probe_state_ &= ~0x01;
        }

        if (state[1]) {
          probe_state_ |= 0x02;
        } else {
          probe_state_ &= ~0x02;
        }

        if (state[2]) {
          probe_state_ |= 0x04;
        } else {
          probe_state_ &= ~0x04;
        }
      } else if (device_id_ == MODULE_DEVICE_ID_3DP_SINGLE) {
        if (state[0])
          probe_state_ |= 0x01;
        else
          probe_state_ &= ~0x01;
      }
    }

    bool probe_state() {
      return (bool)(probe_state_ & (1<<active_probe_sensor_));
    }

    bool probe_state(probe_sensor_t sensor) {
      return (bool)(probe_state_ & (1<<((uint8_t)sensor)));
    }

    void filament_state(uint8_t state[]) {
      if (device_id_ == MODULE_DEVICE_ID_DUAL_EXTRUDER) {
        if (!state[0])
          filament_state_ |= 0x01;
        else
          filament_state_ &= ~0x01;

        if (!state[1])
          filament_state_ |= 0x02;
        else
          filament_state_ &= ~0x02;
      } else if (device_id_ == MODULE_DEVICE_ID_3DP_SINGLE) {
        if (state[0])
          filament_state_ |= 0x01;
        else
          filament_state_ &= ~0x01;
      }
    }

    bool filament_state() {
      return (bool)(filament_state_ & (1<<active_extruder));
    }

    bool filament_state(int extruder) {
      return (bool)(filament_state_ & (1<<extruder));
    }

    void SetTemp(uint8_t *data) {
      cur_temp_[TOOLHEAD_3DP_EXTRUDER0] = data[0] << 8 | data[1];

      if (device_id_ == MODULE_DEVICE_ID_DUAL_EXTRUDER) {
        cur_temp_[TOOLHEAD_3DP_EXTRUDER1] = data[4] << 8 | data[5];
      }
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
    int16_t target_temp_[EXTRUDERS];
    uint8_t fan_speed_[TOOLHEAD_3DP_FAN_MAX];

    // 1 bit indicates one sensor
    uint8_t probe_state_;
    uint8_t filament_state_;
    float pid_[3];

    probe_sensor_t active_probe_sensor_;
    uint8_t extruder_status_;
    nozzle_type_t nozzle_type_[EXTRUDERS];
    bool need_to_tell_hmi_nozzle_type_;
    bool need_to_tell_hmi_extruder_info_;

  public:
    bool is_filament_status_initialized_;
    bool is_probe_sensor_initialized_;
    bool is_nozzle_type_initialized_;
};

extern ToolHead3DP *printer1;

#endif  // #ifndef TOOLHEAD_3DP_H_
