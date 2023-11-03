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
#ifndef SNAPMAKER_TOOLHEAD_LASER_H_
#define SNAPMAKER_TOOLHEAD_LASER_H_

#include "module_base.h"
#include "can_host.h"

#include "../hmi/uart_host.h"

#include "src/module/planner.h"

#define TOOLHEAD_LASER_POWER_SAFE_LIMIT             (0.5)
#define TOOLHEAD_LASER_20W_40W_POWER_SAFE_LIMIT     (0.1)
#define TOOLHEAD_LASER_POWER_NORMAL_LIMIT           (100)
#define TOOLHEAD_LASER_CAMERA_FOCUS_MAX             (65000)  // mm*1000
#define INVALID_OFFSET                              (-10000)


#define FIRE_DETECT_TRIGGER_LIMIT_ADC_VALUE     (4095)
#define FIRE_DETECT_TRIGGER_DISABLE_ADC_VALUE   (0xFFFF)

enum ToolheadLaserFanState {
  TOOLHEAD_LASER_FAN_STATE_OPEN,
  TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED,
  TOOLHEAD_LASER_FAN_STATE_CLOSED,

  TOOLHEAD_LASER_FAN_STATE_INVALID
};


enum ToolHeadLaserState {
  TOOLHEAD_LASER_STATE_OFFLINE,

  TOOLHEAD_LASER_STATE_OFF,
  TOOLHEAD_LASER_STATE_ON,
};


enum LaserCameraCommand {
  M_REPORT_VERSIONS = 0x1,
  S_REPORT_VERSIONS,
  M_CAMERA_GET_AWB = 0x3,
  S_CAMERA_GET_AWB_ACK,
  M_CAMERA_SET_AWB = 0x5,
  S_CAMERA_SET_AWB_ACK,
  M_CAMERA_SET_ACE = 0x7,
  S_CAMERA_SET_ACE_ACK,
  M_CAMERA_SET_IMG_SIZE = 0x9,
  S_CAMERA_SET_IMG_SIZE_ACK,
  M_CAMERA_SET_QUALITY = 0xb,
  S_CAMERA_SET_QUALITY_ACK,
  M_CAMERA_GET_IMG = 0xd,
  S_CAMERA_IMG_ACK,
  M_UPDATE_MOUDLE = 0xf,
  S_UPDATRE_ACK,
  M_SET_BT_NAME = 0x11,
  S_SET_BT_NAME_ACK,
  M_REPORT_BT_NAME = 0x13,
  S_REPORT_BT_NAME_ACK,
  M_REPORT_BT_MAC = 0x15,
  S_REPORT_BT_MAC_ACK,
  M_SET_CAMERA_LIGHT = 0x17,
  S_SET_CAMERA_LIGHT_ACK,
  M_REPORT_CAMERA_LIGHT = 0x19,
  S_REPORT_CAMERA_LIGHT_ACK,
  M_REPORT_CAMERA_STATU = 0x1b,
  S_REPORT_CAMERA_STATU_ACK,

  S_CAMERA_INIT_FAIL = 0xfd,
  S_RECV_FAIL = 0xff,
};

enum LASER_STATUS {
  LASER_ENABLE,
  LASER_WAIT_DISABLE,
  LASER_DISABLE,
};


class ToolHeadLaser: public ModuleBase {
  public:
		ToolHeadLaser(ModuleDeviceID id): ModuleBase(id) {
      power_limit_pwm_ = 255;
      power_pwm_   = 0;
      power_val_   = 0;
      mac_index_   = MODULE_MAC_INDEX_INVALID;
      state_ = TOOLHEAD_LASER_STATE_OFFLINE;
      focus_ = TOOLHEAD_LASER_CAMERA_FOCUS_MAX / 1000;
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_CLOSED;
      fan_tick_  = 0;
      msg_id_set_fan_   = MODULE_MESSAGE_ID_INVALID;
      msg_id_get_focus_ = MODULE_MESSAGE_ID_INVALID;
      timer_in_process_ = 0;

      security_status_ = 0;
      laser_temperature_ = 0;
      imu_temperature_   = 0;
      need_to_turnoff_laser_ = false;
      need_to_tell_hmi_ = false;
      laser_status_ = LASER_DISABLE;
      laser_pwm_pin_checked_ = false;
      pwm_pin_pullup_state_ = 0xff;
      pwm_pin_pulldown_state_ = 0xff;
      fire_sensor_trigger_value_ = 0;
      fire_sensor_trigger_ = false;
      crosslight_offset_x = crosslight_offset_y = INVALID_OFFSET;
      half_power_mode_ = false;
      air_pump_switch_ = false;
      weak_light_origin_mode_ = false;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);
    void TurnoffLaserIfNeeded();
    void PrintInfo(void);

    void TurnOn();
    void PwmCtrlDirectly(uint8_t duty);
    void TurnOff();

    void SetFanPower(uint8_t power);  // power 0 - 100

    void SetPower(float power, bool is_map=true);       // change power_val_ and power_pwm_ but not change actual output
    void SetOutput(float power, bool is_map=true);      // change power_val_, power_pwm_ and actual output
    void SetPowerLimit(float limit);  // change power_val_, power_pwm_ and power_limit_, may change actual output if current output is beyond limit
    uint16_t PowerConversionPwm(float power);

    void TryCloseFan();
    bool IsOnline(uint8_t sub_index = 0) { return mac_index_ != MODULE_MAC_INDEX_INVALID; }

    bool SetAirPumpSwitch(bool onoff, bool output_log=true);
    bool GetAirPumpSwitch(void) { return air_pump_switch_; }
    bool GetHalfPowerMode(void) { return half_power_mode_; }
    bool SetWeakLightOriginMode(bool mode);
    bool GetWeakLightOriginMode(void) { return weak_light_origin_mode_; }

    ErrCode SetCrossLightCAN(bool sw);
    ErrCode GetCrossLightCAN(bool &sw);
    ErrCode SetFireSensorSensitivityCAN(uint16 sen);
    ErrCode GetFireSensorSensitivityCAN(uint16 &sen);
    ErrCode SetFireSensorReportTime(uint16 itv);
    ErrCode SetCrossLightOffsetCAN(float x, float y);
    ErrCode GetCrossLightOffsetCAN(float &x, float &y);
    ErrCode CheckCrossLightOffset(float x_offset, float y_offset);

    // callbacks for HMI event
    ErrCode GetFocus(SSTP_Event_t &event);
    ErrCode SetFocus(SSTP_Event_t &event);
    ErrCode DoManualFocusing(SSTP_Event_t &event);
    ErrCode DoAutoFocusing(SSTP_Event_t &event);

    ErrCode SetCameraBtName(SSTP_Event_t &event);
    ErrCode GetCameraBtName(SSTP_Event_t &event);
    ErrCode GetCameraBtMAC(SSTP_Event_t &event);
    ErrCode ReadBluetoothVer();
    void SetCameraLight(uint8_t state);

    ErrCode GetSecurityStatus(SSTP_Event_t &event);
    ErrCode SendSecurityStatus();
    ErrCode SendPauseStatus();
    ErrCode SetAutoFocusLight(SSTP_Event_t &event);
    ErrCode SetOnlineSyncId(SSTP_Event_t &event);
    ErrCode GetOnlineSyncId(SSTP_Event_t &event);
    ErrCode SetProtectTemp(SSTP_Event_t &event);
    ErrCode LaserControl(uint8_t state);
    ErrCode LaserBranchCtrl(bool onoff);
    ErrCode LaserGetHWVersion();
    ErrCode SetCrossLight(SSTP_Event_t &event);
    ErrCode GetCrossLight(SSTP_Event_t &event);
    ErrCode SetFireSensorSensitivity(SSTP_Event_t &event);
    ErrCode GetFireSensorSensitivity(SSTP_Event_t &event);
    ErrCode SetFireSensorReportTime(SSTP_Event_t &event);
    ErrCode SetCrosslightOffset(SSTP_Event_t &event);
    ErrCode GetCrosslightOffset(SSTP_Event_t &event);
    ErrCode SetWeakLightOriginWork(SSTP_Event_t &event);

    void TellSecurityStatus();
    uint8_t LaserGetPwmPinState();
    void LaserConfirmPinState();
    void Process();

    uint32_t mac(uint8_t sub_index = 0) { return canhost.mac(mac_index_); }

    float power() { return power_val_; }

    uint16_t power_pwm() { return power_pwm_; };
    void power_pwm(uint16_t pwm) { power_pwm_ = pwm; }
    uint16_t tim_pwm();
    void tim_pwm(uint16_t pwm);

    uint16_t focus() { return focus_; }
    void focus(uint16_t focus) {
      if(focus > TOOLHEAD_LASER_CAMERA_FOCUS_MAX)
        focus_ = TOOLHEAD_LASER_CAMERA_FOCUS_MAX / 1000;
      else
        focus_ = focus / 1000;
    }

    ToolHeadLaserState state() { return state_; }

  private:
    void    CheckFan(uint16_t pwm);
    ErrCode LoadFocus();
    ErrCode ReadBluetoothInfo(LaserCameraCommand cmd, uint8_t *out, uint16_t &length);
    ErrCode SetBluetoothInfo(LaserCameraCommand cmd, uint8_t *info, uint16_t length);

  private:
    uint8_t *power_table_;
    uint8_t  mac_index_;
    uint16_t timer_in_process_;
    ToolHeadLaserState  state_;
    float power_val_;
    uint16_t power_limit_pwm_;
    uint16_t power_pwm_;
    uint8_t  fan_state_;
    uint16_t fan_tick_;
    uint16_t laser_tick_ = 0;

    // save orignal value from module
    uint16_t focus_;
    message_id_t msg_id_set_fan_;
    message_id_t msg_id_get_focus_;
    UartHost esp32_;

    bool laser_pwm_pin_checked_;
    uint8_t pwm_pin_pullup_state_;
    uint8_t pwm_pin_pulldown_state_;

  public:
    uint8_t security_status_;
    int16_t roll_;
    int16_t pitch_;
    int8_t laser_temperature_;
    int8_t imu_temperature_;
    bool need_to_turnoff_laser_;
    bool need_to_tell_hmi_;
    bool fire_sensor_trigger_;
    bool fire_sensor_sensitivity_update_;
    uint16_t fire_sensor_trigger_value_;
    uint16_t fire_sensor_rawdata_;
    bool crosslight_offset_update_;
    float crosslight_offset_x, crosslight_offset_y;
    LASER_STATUS laser_status_;
    bool cross_light_state_update_;
    bool cross_light_state_;
    bool half_power_mode_;
    bool air_pump_switch_;
    bool weak_light_origin_mode_;

  // Laser Inline Power functions
  public:
    /**
     * Inline power adds extra fields to the planner block
     * to handle laser power and scale to movement speed.
     */

    // Force disengage planner power control
    void InlineDisable();

    // Set the power for subsequent movement blocks
    void SetOutputInline(float power);
    void SetOutputInline(uint16_t power_pwm, bool is_sync_power=true);
    void UpdateInlinePower(uint16_t power_pwm, float sync_power);

    // Optimized TurnOn function for use from the Stepper ISR
    void TurnOn_ISR(uint16_t power_pwm, bool is_sync_power, float power);
};


extern ToolHeadLaser *laser;
extern ToolHeadLaser laser_1_6_w;
extern ToolHeadLaser laser_10w;
extern ToolHeadLaser laser_20w;
extern ToolHeadLaser laser_40w;

#endif  // #ifndef TOOLHEAD_LASER_H_
