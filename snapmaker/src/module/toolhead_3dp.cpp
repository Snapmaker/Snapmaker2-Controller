#include "toolhead_3dp.h"

#include "../common/config.h"

// marlin headers
#include "src/core/macros.h"
#include "src/core/boards.h"
#include "Configuration.h"
#include "src/pins/pins.h"

ToolHead3DP printer;


static void CallbackAckProbeState(uint8_t *cmd, uint8_t length) {
  printer.probe_state(cmd[0]);
}


static void CallbackAckNozzleTemp(uint8_t *cmd, uint8_t length) {
  // temperature from module, was
  printer.SetTemp(cmd[0]<<8 | cmd[1], 0);
}


static void CallbackAckFilamentState(uint8_t *cmd, uint8_t length) {
  // temperature from module, was
  printer.filament_state(cmd[0], 0);
}


ErrCode ToolHead3DP::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[16];

  Function_t    function;
  message_id_t  message_id[4];

  CanStdCmdCallback_t cb;

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS)
    return ret;

  // we have configured 3DP in same port
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
    switch (function.id) {
    case MODULE_FUNC_PROBE_STATE:
      cb = CallbackAckProbeState;
      break;

    case MODULE_FUNC_RUNOUT_SENSOR_STATE:
      cb = CallbackAckFilamentState;
      break;

    case MODULE_FUNC_GET_NOZZLE_TEMP:
      cb = CallbackAckNozzleTemp;
      break;

    default:
      cb = NULL;
      break;
    }
    message_id[i] = canhost.RegisterFunction(function, cb);
  }

  ret = canhost.BindMessageID(func_buffer, message_id);

  mac_index_ = mac_index;

  return E_SUCCESS;
}


ErrCode ToolHead3DP::SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time) {
  CanStdFuncCmd_t cmd;

  uint8_t buffer[2];

  buffer[0] = delay_time;
  buffer[1] = speed;

  if (fan_index)
    cmd.id = MODULE_FUNC_SET_FAN2;
  else
    cmd.id = MODULE_FUNC_SET_FAN1;

  cmd.data   = buffer;
  cmd.length = 2;

  fan_speed_[fan_index] = speed;

  return canhost.SendStdCmd(cmd, 0);
}


ErrCode ToolHead3DP::SetPID(uint8_t index, float value, uint8_t extrude_index) {
  CanStdFuncCmd_t cmd;

  uint8_t  buffer[5];
  uint32_t scale_val = (uint32_t)(value * 1000);

  buffer[0] = index;

  buffer[1] = (uint8_t)(scale_val>>24);
  buffer[2] = (uint8_t)(scale_val>>16);
  buffer[3] = (uint8_t)(scale_val>>8);
  buffer[4] = (uint8_t)(scale_val);

  cmd.id     = MODULE_FUNC_SET_3DP_PID;
  cmd.data   = buffer;
  cmd.length = 5;

  return canhost.SendStdCmd(cmd, 0);
}


ErrCode ToolHead3DP::SetHeater(uint16_t target_temp, uint8_t extrude_index) {
  CanStdFuncCmd_t cmd;

  uint8_t buffer[2];

  buffer[0] = (uint8_t)(target_temp>>8);
  buffer[1] = (uint8_t)target_temp;

  if (target_temp > 60) {
    SetFan(1, 255);
  }

  // if we turn off heating
  if (target_temp == 0) {
    // check if need to delay to turn off fan
    if (cur_temp_[extrude_index] > 150) {
      SetFan(1, 0, 120);
    }
    else if (cur_temp_[extrude_index] > 60) {
      SetFan(1, 0, 60);
    }
    else {
      SetFan(1, 0);
    }
  }

  cmd.id     = MODULE_FUNC_SET_NOZZLE_TEMP;
  cmd.data   = buffer;
  cmd.length = 2;

  return canhost.SendStdCmd(cmd, 0);
}