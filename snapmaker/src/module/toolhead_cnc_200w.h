#ifndef SNAPMAKER_TOOLHEAD_CNC_200W_H_
#define SNAPMAKER_TOOLHEAD_CNC_200W_H_

#include "module_base.h"
#include "can_host.h"

#define CNC_POWER_MAX (100)
#define CNC_200W_DEFAULT_MAX_RPM   18000
#define CNC_200W_DEFAULT_MIN_RPM   8000

typedef enum {
  CMD_SET_MOTOR_POWER = 0,
  CMD_SET_MOTOR_RPM,
  CMD_SET_MOTOR_RUN_MODE,
  CMD_SET_MOTOR_RUN_DIR,
  CMD_GET_MOTOR_PID_KP,
  CMD_GET_MOTOR_PID_KI,
  CMD_GET_MOTOR_PID_KD,
  CMD_SET_MOTOR_PID_KP,
  CMD_SET_MOTOR_PID_KI,
  CMD_SET_MOTOR_PID_KD,
  CMD_SET_MOTOR_FAN,
}CncProcType;

enum CNCSpeedControlType {
  CNC_PWM_SET_SPEED = 0,
  CNC_RPM_SET_SPEED,
};

enum CNCOutputStatus {
  CNC_OUTPUT_OFF = 0,
  CNC_OUTPUT_ON = 1,
  CNC_OUTPUT_OFF_ING = 2,
  CNC_OUTPUT_INVALID
};

enum CNCSpeedControlMode {
  CNC_CONSTANT_POWER_MODE = 0,
  CNC_CONSTANT_RPM_MODE,
};

class ToolHeadCNC200W: public ModuleBase {
  public:
		ToolHeadCNC200W(): ModuleBase(MODULE_DEVICE_ID_200W_CNC) {
      is_print_rpm       = false;
      mac_index_         = MODULE_MAC_INDEX_INVALID;
      power_             = 0;
      cur_power_         = 0;
      rpm_               = 0;
      target_rpm_        = 0;
      m_error_           = 0;
      motor_current_     = 0;
      motor_temperature_ = 0;
      pcb_temperature_   = 0;
      motor_voltage_     = 0;
      timer_tick         = 0;
      m_state_           = CNC_OUTPUT_OFF;
      ctr_mode_          = CNC_CONSTANT_RPM_MODE;
      msg_id_pwm_set_speed_ = MODULE_MESSAGE_ID_INVALID;
      msg_id_rpm_set_speed_ = MODULE_MESSAGE_ID_INVALID;
    }

    void Process(void);
    void PrintInfo(void);
    ErrCode Init(MAC_t &mac, uint8_t mac_index);
    ErrCode Cnc200WSpeedSetting(uint16_t value, CNCSpeedControlType type=CNC_PWM_SET_SPEED, bool is_update_power=true);
    ErrCode Cnc200WTargetSpeedConfigure(uint16_t value, CNCSpeedControlType type=CNC_PWM_SET_SPEED);
    bool IsOnline(uint8_t sub_index = 0) { return mac_index_ != MODULE_MAC_INDEX_INVALID; };

    uint32_t mac(uint8_t sub_index = 0) {
      if (sub_index > 0)
        return MODULE_MAC_ID_INVALID;
      return canhost.mac(mac_index_);
    }

    uint16_t rpm() { return rpm_; }
    void rpm(uint16_t rpm) { rpm_ = rpm; }
    void set_is_print_rpm(bool is_print) { is_print_rpm = !!is_print; }

    uint16_t power() { return power_; }
    void power(uint16_t power);

    uint8_t cnc_error() { return m_error_; }
    CNCOutputStatus cnc_state() { return m_state_; }

    friend void CallbackAckSpindleRunInfo(CanStdDataFrame_t &cmd);
    friend void CallbackAckSpindleSensorInfo(CanStdDataFrame_t &cmd);

  private:
    bool      is_print_rpm;
    uint8_t   mac_index_;
    uint8_t   m_error_;
    uint16_t  power_;
    uint16_t  cur_power_;
    uint16_t  rpm_;
    uint16_t  target_rpm_;
    int16_t   motor_current_;
    float     motor_temperature_;
    float     pcb_temperature_;
    float     motor_voltage_;
    uint32_t  timer_tick;
    CNCOutputStatus m_state_;
    CNCSpeedControlMode ctr_mode_;

    message_id_t msg_id_pwm_set_speed_;
    message_id_t msg_id_rpm_set_speed_;
};

extern ToolHeadCNC200W cnc_200w;

#endif