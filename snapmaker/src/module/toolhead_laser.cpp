#include "toolhead_laser.h"

#include "../common/config.h"


#include MARLIN_SRC(pins/pins.h)

#define LASER_CLOSE_FAN_DELAY     (120)


#define TimSetPwm(n)  Tim1SetCCR4(n)

extern void Tim1SetCCR4(uint16_t pwm);
extern void Tim1PwmInit();

static const uint8_t power_table[]= {
  0,
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};


static void CallbackAckLaserFocus(uint8_t *cmd, uint8_t length) {
  laser.f
}


ErrCode ToolHeadLaser::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[16];

  Function_t    function;
  message_id_t  message_id[4];

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS)
    return ret;

  // we have configured Laser in same port
  if (mac_index_ != MODULE_MAC_INDEX_INVALID)
    return E_SAME_STATE;

  cmd.mac    = mac;
  cmd.data   = func_buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_FUNCID_REQ;

  // try to get function ids from module
  if (canhost.SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;

  function.channel   = mac.bits.channel;
  function.mac_index = mac_index;
  function.sub_index = 0;
  function.priority  = MODULE_FUNC_PRIORITY_DEFAULT;

  // register function ids to can host, it will assign message id
  for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    if (function.id == MODULE_FUNC_CURRENT_LASER_FOCUS)
      message_id[i] = canhost.RegisterFunction(function, CallbackAckLaserFocus);
    else
      message_id[i] = canhost.RegisterFunction(function, NULL);
  }

  ret = canhost.BindMessageID(func_buffer, message_id);

  Tim1PwmInit();
  esp32_.Init(&MSerial3, EXECUTOR_SERIAL_IRQ_PRIORITY);

  mac_index_ = mac_index;

  return E_SUCCESS;
}


void ToolHeadLaser::TurnOn() {
  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  state_ = TOOLHEAD_LASER_STATE_ON;
  CheckFan(power_pwm_);
  TimSetPwm(power_pwm_);
}


void ToolHeadLaser::TurnOff() {
  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  state_ = TOOLHEAD_LASER_STATE_OFF;
  CheckFan(0);
  TimSetPwm(0);
}


void ToolHeadLaser::ChangePowerImmediately(float power) {
  ChangePower(power);
  TurnOn();
}


void ToolHeadLaser::ChangePower(float power) {
  int   integer;
  float decimal;

  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  power_val_ = power;

  if (power > power_limit_)
    power = power_limit_;

  integer = (int)power;
  decimal = power - integer;

  power_pwm_ = (uint16_t)(power_table[integer] + (power_table[integer + 1] - power_table[integer]) * decimal);
}


void ToolHeadLaser::ChangePowerLimit(float limit) {
  float cur_power = power_val_;

  if (limit > TOOLHEAD_LASER_POWER_NORMAL_LIMIT)
    limit = TOOLHEAD_LASER_POWER_NORMAL_LIMIT;

  power_limit_ = limit;

  ChangePower(power_val_);
  power_val_ = cur_power;

  // check if we need to change current output
  if (state_ == TOOLHEAD_LASER_STATE_ON)
    TurnOn();
}


void ToolHeadLaser::CheckFan(uint16_t pwm) {
  CanStdMesgCmd_t cmd;
  uint8_t         buffer[2];

  switch (fan_state_) {
  case TOOLHEAD_LASER_FAN_STATE_OPEN:
    if (pwm == 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED;
      fan_tick_  = 0;
    }
    break;

  case TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED:
    if (pwm > 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_OPEN;
      fan_tick_  = 0;
    }
    break;

  case TOOLHEAD_LASER_FAN_STATE_CLOSED:
    if (pwm > 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_OPEN;

      buffer[0]  = 0;
      buffer[1]  = 255;
      cmd.id     = msg_id_set_fan_;
      cmd.data   = buffer;
      cmd.length = 2;
      canhost.SendStdCmd(cmd);
    }
    break;
  }
}


void ToolHeadLaser::TryCloseFan() {
  CanStdMesgCmd_t cmd;
  uint8_t         buffer[2];

  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  if (fan_state_ == TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED) {
    if (++fan_tick_ > LASER_CLOSE_FAN_DELAY) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_CLOSED;
      fan_tick_  = 0;

      buffer[0]  = 0;
      buffer[1]  = 0;
      cmd.id     = msg_id_set_fan_;
      cmd.data   = buffer;
      cmd.length = 2;
      canhost.SendStdCmd(cmd);
    }
  }
}



ErrCode ToolHeadLaser::LoadFocus() {

}
