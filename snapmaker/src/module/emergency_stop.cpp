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
#include "emergency_stop.h"
#include "../common/config.h"
#include "../common/protocol_sstp.h"
#include "../common/debug.h"
#include "../service/system.h"

EmergencyStop emergency_stop;


static void CallbackEmergencyStopState(CanStdDataFrame_t &cmd) {
  emergency_stop.SetStatus(cmd.data[0]);
}


ErrCode EmergencyStop::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[16];

  Function_t    function;
  message_id_t  message_id[4];

  cmd.mac    = mac;
  cmd.data   = func_buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_FUNCID_REQ;

  // try to get function ids from module
  if (canhost.SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;

  LOG_I("\tGot EmergencyStop Module!\n");

  function.channel   = mac.bits.channel;
  function.mac_index = mac_index;
  function.sub_index = 0;
  function.priority  = MODULE_FUNC_PRIORITY_DEFAULT;

  // register function ids to can host, it will assign message id
  for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    if (function.id == MODULE_FUNC_REPORT_EMERGENCY_STOP)
      message_id[i] = canhost.RegisterFunction(function, CallbackEmergencyStopState);
    else
      message_id[i] = canhost.RegisterFunction(function, NULL);
  }

  ret = canhost.BindMessageID(cmd, message_id);
  if (ret == E_SUCCESS) {
    event_state_  = EMERGENCY_STOP_ONLINE;
    PollState();
  }

  mac_index_ = mac_index;
  return ret;
}

void EmergencyStop::PollState() {
  CanStdFuncCmd_t cmd;

  cmd.id = MODULE_FUNC_REPORT_EMERGENCY_STOP;
  cmd.length = 0;

  canhost.SendStdCmd(cmd, 0);
}

void EmergencyStop::Process() {
  if (!IsOnline())
    return;

  if (event_state_ != EMERGENCY_STOP_TRIGGER) {
    if (state_ == EMERGENCY_STOP_TRIGGER) {
      event_state_ = EMERGENCY_STOP_TRIGGER;
      systemservice.StopTrigger(TRIGGER_SOURCE_STOP_BUTTON, SYSCTL_OPC_STOP);
      ModuleBase::LockMarlinUart(LOCK_SOURCE_EMERGENCY_STOP);
      ReportStatus();
      LOG_I("emergency stop!\n");
    }
  }
}

ErrCode EmergencyStop::ReportStatus() {
  uint8_t state = event_state_;
  SSTP_Event_t event;
  // For active sending
  event.id = EID_ADDON_ACK;
  event.op_code = ADDON_OPC_GET_ADDON_STOP;

  event.length = 1;
  event.data = &state;
  LOG_I("SC req emergency stop sta:%d\n", state);
  return hmi.Send(event);
}
