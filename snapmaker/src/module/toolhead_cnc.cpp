#include "toolhead_cnc.h"

#include "../common/config.h"

#include MARLIN_SRC(pins/pins.h)

ToolHeadCNC cnc;


static void CallbackAckSpindleSpeed(uint8_t *cmd, uint8_t length) {
  cnc.rpm(cmd[0]<<8 | cmd[1]);
}


ErrCode ToolHeadCNC::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[16];

  Function_t    function;
  message_id_t  message_id[4];

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS)
    return ret;

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
    if (function.id == MODULE_FUNC_GET_SPINDLE_SPEED)
      message_id[i] = canhost.RegisterFunction(function, CallbackAckSpindleSpeed);
    else
      message_id[i] = canhost.RegisterFunction(function, NULL);
  }

  ret = canhost.BindMessageID(func_buffer, message_id);

  mac_index_ = mac_index;

  return E_SUCCESS;
}


ErrCode ToolHeadCNC::SetOutput(uint8_t power) {
  if (power > 100)
    power_ = 100;
  else
    power_ = power;

  return TurnOn();
}


ErrCode ToolHeadCNC::TurnOn() {
  CanStdMesgCmd_t cmd;

  uint8_t buffer[2];

  buffer[0] = power_;

  cmd.data   = buffer;
  cmd.length = 1;
  cmd.id     = msg_id_set_speed_;

  return canhost.SendStdCmd(cmd);
}


ErrCode ToolHeadCNC::TurnOff() {
  CanStdMesgCmd_t cmd;

  uint8_t buffer[2];

  buffer[0] = 0;

  cmd.data   = buffer;
  cmd.length = 1;
  cmd.id     = msg_id_set_speed_;

  return canhost.SendStdCmd(cmd);
}

