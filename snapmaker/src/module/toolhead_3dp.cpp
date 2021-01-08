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
#include "toolhead_3dp.h"

#include "../common/config.h"
#include "common/debug.h"

// marlin headers
#include "src/core/macros.h"
#include "src/core/boards.h"
#include "Configuration.h"
#include "src/pins/pins.h"
#include "src/inc/MarlinConfig.h"
#include HAL_PATH(src/HAL, HAL.h)

ToolHead3DP printer_single(MODULE_DEVICE_ID_3DP_SINGLE);
ToolHead3DP printer_dual(MODULE_DEVICE_ID_3DP_DUAL);

ToolHead3DP *printer1 = &printer_single;

static void CallbackAckProbeState(CanStdDataFrame_t &cmd) {
  printer1->SetProbeState(cmd.data);
}


static void CallbackAckNozzleTemp(CanStdDataFrame_t &cmd) {
  // temperature from module, was
  printer1->SetTemp(cmd.data);
}


static void CallbackAckFilamentState(CanStdDataFrame_t &cmd) {
  // temperature from module, was
  printer1->SetFilamentState(cmd.data);
}


void ToolHead3DP::IOInit(void) {
  SET_OUTPUT(E0_STEP_PIN);

  SET_OUTPUT(E0_DIR_PIN);

  SET_OUTPUT(E0_ENABLE_PIN);
  if (!E_ENABLE_ON) WRITE(E0_ENABLE_PIN, HIGH);
}


ErrCode ToolHead3DP::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[32];

  Function_t    function;
  message_id_t  message_id[10];

  CanStdMesgCmd_t mesg_cmd = {MODULE_MESSAGE_ID_INVALID, 0, NULL};
  uint8_t msg_id_index_probe = 0xff;
  uint8_t msg_id_index_runout = 0xff;

  CanStdCmdCallback_t cb = NULL;

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS)
    return ret;

  LOG_I("\tGot toolhead 3DP%d!\n", device_id_);

  // we have configured 3DP in same port
  if (mac_index_ != MODULE_MAC_INDEX_INVALID)
    return E_SAME_STATE;

  cmd.mac    = mac;
  cmd.data   = func_buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_GET_FUNCID_REQ;

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
      msg_id_index_probe = i;
      break;

    case MODULE_FUNC_RUNOUT_SENSOR_STATE:
      cb = CallbackAckFilamentState;
      msg_id_index_runout = i;
      break;

    case MODULE_FUNC_GET_NOZZLE_TEMP:
      cb = CallbackAckNozzleTemp;
      break;

    default:
      cb = NULL;
      break;
    }

    message_id[i] = canhost.RegisterFunction(function, cb);

    if (message_id[i] == MODULE_MESSAGE_ID_INVALID) {
      LOG_E("\tfailed to register function!\n");
      break;
    }

    if (function.id == MODULE_FUNC_SWITCH_EXTRUDER)
      msg_id_swtich_extruder_ = message_id[i];
  }

  ret = canhost.BindMessageID(cmd, message_id);
  if (ret != E_SUCCESS) {
    LOG_E("\tfailed to bind message id!\n");
    goto out;
  }

  mac_index_ = mac_index;
  printer1 = this;
  IOInit();

  SetToolhead(MODULE_TOOLHEAD_3DP);

  // read the state of sensors
  vTaskDelay(portTICK_PERIOD_MS * 100);

  if (msg_id_index_probe != 0xff) {
    mesg_cmd.id = message_id[msg_id_index_probe];
    canhost.SendStdCmd(mesg_cmd);
  }

  vTaskDelay(portTICK_PERIOD_MS * 10);
  if (msg_id_index_runout != 0xff) {
    mesg_cmd.id = message_id[msg_id_index_runout];
    canhost.SendStdCmd(mesg_cmd);
  }

  vTaskDelay(portTICK_PERIOD_MS * 100);

#if 0
  // enable extruder 0
  mesg_cmd.id     = msg_id_swtich_extruder_;
  mesg_cmd.length = 1;
  mesg_cmd.data   = &cur_extruder_;
  canhost.SendStdCmd(mesg_cmd);
#endif

  LOG_I("\tprobe: 0x%x, filament: 0x%x\n", probe_state_, filament_state_);

out:
  return ret;
}


void ToolHead3DP::SetProbeState(uint8_t state[]) {
  if (device_id_ == MODULE_DEVICE_ID_3DP_DUAL) {
    if (!state[0])
      probe_state_ |= 0x01;
    else
      probe_state_ &= ~0x01;

    if (!state[1])
      probe_state_ |= 0x02;
    else
      probe_state_ &= ~0x02;
  }
  else {
    if (state[0])
      probe_state_ |= 0x01;
    else
      probe_state_ &= ~0x01;
  }
}


bool ToolHead3DP::GetProbeState(uint8_t extruder) {
  return (bool)(probe_state_ & (1<<extruder));
}


void ToolHead3DP::SetFilamentState(uint8_t state[]) {
  if (device_id_ == MODULE_DEVICE_ID_3DP_DUAL) {
    if (!state[1])
      filament_state_ |= 0x02;
    else
      filament_state_ &= ~0x02;

    if (!state[0])
      filament_state_ |= 0x01;
    else
      filament_state_ &= ~0x01;
  }
  else {
    if (state[0])
      filament_state_ |= 0x01;
    else
      filament_state_ &= ~0x01;
  }
}

bool ToolHead3DP::GetFilamentState(uint8_t extruder) {
  return (bool)(filament_state_ & (0x01<<extruder));
}


ErrCode ToolHead3DP::SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time) {
  CanStdFuncCmd_t cmd;

  uint8_t buffer[4];

  buffer[0] = delay_time;
  buffer[1] = speed;
  buffer[2] = delay_time;
  buffer[3] = speed;

  if (fan_index)
    cmd.id = MODULE_FUNC_SET_FAN2;
  else
    cmd.id = MODULE_FUNC_SET_FAN1;

  cmd.data   = buffer;
  cmd.length = 4;

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

  uint8_t buffer[2 * TOOLHEAD_3DP_EXTRUDER_MAX];

  if (extrude_index > TOOLHEAD_3DP_EXTRUDER_MAX)
    return E_PARAM;

  target_temp_[extrude_index] = target_temp;
  LOG_I("Set T%d, T0=%d, T1=%d\n", extrude_index, target_temp_[0], target_temp_[1]);

  for (int i = 0; i < TOOLHEAD_3DP_EXTRUDER_MAX; i++) {
    buffer[2*i + 0] = (uint8_t)(target_temp_[i]>>8);
    buffer[2*i + 1] = (uint8_t)target_temp_[i];
  }

  uint8_t fan_index = 1;
  uint8_t fan_speed = 0;
  uint8_t fan_delay = 0;

  if (target_temp >= 60) {
    fan_speed = 255;
  }
  else if (target_temp == 0) {
    // check if need to delay to turn off fan
    if (cur_temp_[extrude_index] >= 150) {
      fan_speed = 0;
      fan_delay = 120;
    }
    else if (cur_temp_[extrude_index] >= 60) {
      fan_speed = 0;
      fan_delay = 60;
    }
    else {
      fan_speed = 0;
      fan_delay = 0;
    }
  }

  if (printer1->device_id() == MODULE_DEVICE_ID_3DP_SINGLE) {
    fan_index = 1;
  }
  else if (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) {
    if (extrude_index == TOOLHEAD_3DP_EXTRUDER0) {
      fan_index = 0;
    }
    else if (extrude_index == TOOLHEAD_3DP_EXTRUDER1) {
      fan_index = 1;
    }
  }

  SetFan(fan_index, fan_speed, fan_delay);

  cmd.id     = MODULE_FUNC_SET_NOZZLE_TEMP;
  cmd.data   = buffer;
  cmd.length = 2 * TOOLHEAD_3DP_EXTRUDER_MAX;

  return canhost.SendStdCmd(cmd, 0);
}


ErrCode ToolHead3DP::SwitchExtruder(uint8_t extrude_index, uint8_t motor_switching/*=0*/, uint16_t motor_runtime/*=0*/) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[4];

  switch (extrude_index) {
  case 0:
    extrude_index = TOOLHEAD_3DP_EXTRUDER0;
    break;

  case 1:
    extrude_index = TOOLHEAD_3DP_EXTRUDER1;
    break;

  default:
    return E_PARAM;
  }

  switch (motor_switching) {
  case 0:
    motor_switching = 0;
    break;

  case 1:
    motor_switching = 1;
    break;

  default:
    return E_PARAM;
  }

  can_buffer[0] = extrude_index;
  can_buffer[1] = motor_switching;
  can_buffer[2] = (motor_runtime >> 8) & 0xff;
  can_buffer[3] = motor_runtime & 0xff;
  cmd.id        = MODULE_FUNC_SWITCH_EXTRUDER;
  cmd.data      = can_buffer;
  cmd.length    = 4;

  return canhost.SendStdCmdSync(cmd, 10000);
}

void ToolHead3DP::Process() {
  if (++timer_in_process_ < 100) return;

  timer_in_process_ = 0;
}