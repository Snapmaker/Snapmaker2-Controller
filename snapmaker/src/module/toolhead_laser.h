#ifndef SNAPMAKER_TOOLHEAD_LASER_H_
#define SNAPMAKER_TOOLHEAD_LASER_H_

#include "module_base.h"
#include "can_host.h"

#include "../common/uart_host.h"

#define TOOLHEAD_LASER_POWER_SAFE_LIMIT   (0.5)
#define TOOLHEAD_LASER_POWER_NORMAL_LIMIT (100)


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

  TOOLHEAD_LASER_STATE_INVALID
};

class ToolHeadLaser: public ModuleBase {
  public:
		ToolHeadLaser(): ModuleBase(MODULE_DEVICE_ID_LINEAR) {
      power_limit_ = 100;

      power_pwm_ = 0;
      power_val_ = 0;

      state_ = TOOLHEAD_LASER_STATE_OFFLINE;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    void TurnOn();
    void TurnOff();

    void ChangePower(float power);
    void ChangePowerImmediately(float power);
    void ChangePowerLimit(float limit);

    void TryCloseFan();

    bool IsOnline(uint8_t sub_index = 0);

    uint32_t mac(uint8_t sub_index = 0);

    float power() { return power_val_; }

    uint16_t power_pwm() { return power_pwm_; };
    void power_pwm(uint16_t pwm) { power_pwm_ = pwm; }

    uint16_t focus() { return focus_; }

    ToolHeadLaserState state() { return state_; }
  private:
    ErrCode LoadFocus();
    void CheckFan(uint16_t pwm);


  private:
    uint8_t  mac_index_;

    ToolHeadLaserState  state_;

    float power_val_;
    float power_limit_;

    uint16_t power_pwm_;

    uint8_t  fan_state_;
    uint16_t fan_tick_;

    uint16_t focus_;

    message_id_t msg_id_set_fan_;
    message_id_t msg_id_get_focus_;

    UartHost esp32_;
};


extern ToolHeadLaser laser;

#endif  // #ifndef TOOLHEAD_LASER_H_
