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
#include "toolhead_cnc.h"

#include "../common/config.h"
#include "../common/debug.h"

// marlin headers
#include "src/core/macros.h"
#include "src/core/boards.h"
#include "Configuration.h"
#include "src/pins/pins.h"
#include "src/module/stepper.h"

// marlin headers

ToolHeadCNC cnc;


static void CallbackAckSpindleSpeed(CanStdDataFrame_t &cmd) {
  cnc.rpm(cmd.data[0]<<8 | cmd.data[1]);
}


ErrCode ToolHeadCNC::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[16];

  Function_t    function;
  message_id_t  message_id[4];

  if (axis_to_port[E_AXIS] != PORT_8PIN_1) {
    LOG_E("toolhead CNC failed: Please use the <M1029 E1> set E port\n");
    return E_HARDWARE;
  }

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS)
    return ret;

  LOG_I("\tGet toolhead CNC!\n");

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

    // buffer the message id to set spindle speed
    if (function.id == MODULE_FUNC_SET_SPINDLE_SPEED)
      msg_id_set_speed_ = message_id[i];
  }

  ret = canhost.BindMessageID(cmd, message_id);

  mac_index_ = mac_index;

  SetToolhead(MODULE_TOOLHEAD_CNC);

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

