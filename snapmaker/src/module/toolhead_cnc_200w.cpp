#include "toolhead_cnc_200w.h"

#include "../common/config.h"
#include "../common/debug.h"

// marlin headers
#include "src/core/macros.h"
#include "src/core/boards.h"
#include "Configuration.h"
#include "src/pins/pins.h"
#include "src/HAL/HAL_GD32F1/HAL.h"
#include "../service/system.h"

ToolHeadCNC200W cnc_200w;
uint8_t record_last_error = 0;

void CallbackAckSpindleRunInfo(CanStdDataFrame_t &cmd) {
  uint8_t error_sta_bak = 0;
  cnc_200w.rpm(cmd.data[0]<<8 | cmd.data[1]);
  cnc_200w.m_state_ = (CNCOutputStatus)cmd.data[3];
  cnc_200w.ctr_mode_ = (CNCSpeedControlMode)cmd.data[5];
  cnc_200w.cur_power_ = cmd.data[7];
  error_sta_bak = cnc_200w.m_error_;
  if (~(error_sta_bak & 0x3f) & (cmd.data[2] & 0x3f)) {
    if (systemservice.GetCurrentStage() == SYSTAGE_WORK || systemservice.GetCurrentStage() == SYSTAGE_PAUSE) {
      quickstop.Trigger(QS_SOURCE_SECURITY, true);
    }
    // else {
    //   cnc_200w.power(0);
    // }
    LOG_E("new exception trigger, cnc error: 0x%x\n", cmd.data[2]);
    cnc_200w.PrintInfo();
  }
  cnc_200w.m_error_ = cmd.data[2];

  if (cnc_200w.m_error_)
    record_last_error = cnc_200w.m_error_;
}

void CallbackAckSpindleSensorInfo(CanStdDataFrame_t &cmd) {
  cnc_200w.motor_temperature_ = (float)((int16_t)(cmd.data[0] << 8 | cmd.data[1])) / 10;
  cnc_200w.pcb_temperature_ = (float)((int16_t)(cmd.data[2] << 8 | cmd.data[3])) / 10;
  cnc_200w.motor_current_ = (int16_t)(cmd.data[4] << 8 | cmd.data[5]);
  cnc_200w.motor_voltage_ = (float)((int16_t)(cmd.data[6] << 8 | cmd.data[7])) / 100;
}

ErrCode ToolHeadCNC200W::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;
  CanExtCmd_t cmd;
  uint8_t     func_buffer[2*50 + 2];
  Function_t    function;
  message_id_t  message_id[50];

  LOG_I("\tGet 200W CNC!\n");

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS) {
    LOG_I("\t200w CNC assembly error or dir pin damages!\n");
    return ret;
  }

  // we have configured CNC in same port
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

    if (function.id == MODULE_FUNC_REPORT_SPINDLE_RUN_INFO)
      message_id[i] = canhost.RegisterFunction(function, CallbackAckSpindleRunInfo);
    else if (function.id == MODULE_FUNC_REPORT_SPINDLE_SENSOR_INFO)
      message_id[i] = canhost.RegisterFunction(function, CallbackAckSpindleSensorInfo);
    else
      message_id[i] = canhost.RegisterFunction(function, NULL);

    // buffer the message id to set spindle speed
    if (MODULE_FUNC_SET_SPINDLE_SPEED == function.id)
      msg_id_pwm_set_speed_ = message_id[i];
    else if (MODULE_FUNC_SET_SPINDLE_RPM == function.id)
      msg_id_rpm_set_speed_ = message_id[i];
  }

  ret = canhost.BindMessageID(cmd, message_id);

  mac_index_ = mac_index;

  SetToolhead(MODULE_TOOLHEAD_CNC_200W);
  return E_SUCCESS;
}

ErrCode ToolHeadCNC200W::Cnc200WSpeedSetting(uint16_t value, CNCSpeedControlType type, bool is_update_power) {
  CanStdMesgCmd_t cmd;
  uint8_t data[8];
  uint8_t index = 0;
  ErrCode ret = E_FAILURE;
  message_id_t tmp_msg_id_set_speed;

  if (type == CNC_PWM_SET_SPEED)
    tmp_msg_id_set_speed = msg_id_pwm_set_speed_;
  else
    tmp_msg_id_set_speed = msg_id_rpm_set_speed_;

  if (tmp_msg_id_set_speed == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("Cnc200WSpeedSetting fail: MODULE_MESSAGE_ID_INVALID\n");
    return ret;
  }

  if (type == CNC_PWM_SET_SPEED) {
    value = value > CNC_POWER_MAX ? CNC_POWER_MAX : value;
    if (is_update_power) {
      power_ = value;
      // Manual conversion to corresponding target speed
      target_rpm_ = (float_t)value / CNC_POWER_MAX * CNC_200W_DEFAULT_MAX_RPM;
    }
    data[index++] = (uint8_t)value;
  }
  else {
    value = value > CNC_200W_DEFAULT_MAX_RPM ? CNC_200W_DEFAULT_MAX_RPM : value;
    if (value && value < CNC_200W_DEFAULT_MIN_RPM)
      value = CNC_200W_DEFAULT_MIN_RPM;

    if (is_update_power) {
      power_ = (float)value / CNC_200W_DEFAULT_MAX_RPM * CNC_POWER_MAX;
      target_rpm_ = value;
    }
    data[index++] = (value >> 8) & 0xff;
    data[index++] = value & 0xff;
  }

  cmd.data   = data;
  cmd.length = index;
  cmd.id     = tmp_msg_id_set_speed;
  ret = canhost.SendStdCmd(cmd);

  if (ret == E_SUCCESS) {
    if (value > 0)
      m_state_ = CNC_OUTPUT_ON;
    else if (m_state_ == CNC_OUTPUT_ON)
      m_state_ = CNC_OUTPUT_OFF_ING;
    else
      m_state_ = CNC_OUTPUT_OFF;
  }
  else {
    LOG_E("Cnc200WSpeedSetting send msg fail, ret: %d\n", ret);
  }

  return ret;
}

ErrCode ToolHeadCNC200W::Cnc200WTargetSpeedConfigure(uint16_t value, CNCSpeedControlType type) {
  ErrCode ret = E_FAILURE;
  if (m_state_ != CNC_OUTPUT_ON) {
    if (type == CNC_PWM_SET_SPEED) {
      value = value > CNC_POWER_MAX ? CNC_POWER_MAX : value;
      power_ = value;
      target_rpm_ = (float_t)value / CNC_POWER_MAX * CNC_200W_DEFAULT_MAX_RPM;
    }
    else {
      value = value > CNC_200W_DEFAULT_MAX_RPM ? CNC_200W_DEFAULT_MAX_RPM : value;
      if (value && value < CNC_200W_DEFAULT_MIN_RPM)
        value = CNC_200W_DEFAULT_MIN_RPM;
      power_ = (float)value / CNC_200W_DEFAULT_MAX_RPM * CNC_POWER_MAX;
      target_rpm_ = value;
    }
  }
  else {
    ret = Cnc200WSpeedSetting(value, type);
  }

  return ret;
}

void ToolHeadCNC200W::power(uint16_t power) {
  LIMIT(power, 0, CNC_POWER_MAX);
  power_ = power;
  target_rpm_ = (float)power_ / CNC_POWER_MAX * CNC_200W_DEFAULT_MAX_RPM;
}

void ToolHeadCNC200W::PrintInfo(void) {
  LOG_I("rpm: %d, error: 0x%x\n", rpm_, m_error_);
  LOG_I("M_I: %d, M_V: %.2f\n",motor_current_, motor_voltage_);
  LOG_I("M_TEMP: %.2f, PCB_TEMP: %.2f\n",motor_temperature_, pcb_temperature_);
  LOG_I("ctr mode: %s\n",ctr_mode_ ? "CNC_CONSTANT_RPM_MODE" : "CNC_CONSTANT_POWER_MODE");
  LOG_I("run status: %s\n",m_state_ == 0 ? "STOP" :  m_state_ == 1 ? "RUN" : "STOPING");
  LOG_I("cur_power: %d target_power: %d\n",  cur_power_, power_);
  LOG_I("cur_rpm: %d target_rpm: %d\n",  rpm_, target_rpm_);
  LOG_I("last error: 0x%x\n", record_last_error);
}

void ToolHeadCNC200W::Process(void) {
  if (is_print_rpm) {
    if (++timer_tick >= 100) {
      timer_tick = 0;
      SERIAL_ECHOLNPAIR("RPM: ", rpm());
    }
  }
}