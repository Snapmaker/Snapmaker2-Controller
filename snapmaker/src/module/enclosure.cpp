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
#include "enclosure.h"

#include "../common/config.h"
#include "../common/debug.h"
#include "../module/toolhead_laser.h"
#include "../service/system.h"

Enclosure enclosure;


static void CallbackAckDoorState(CanStdDataFrame_t &cmd) {
  enclosure.door_state(cmd.data[0]);
}


ErrCode Enclosure::Init(MAC_t &mac, uint8_t mac_index) {
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

  LOG_I("\tGot Enclosure!\n");

  function.channel   = mac.bits.channel;
  function.mac_index = mac_index;
  function.sub_index = 0;
  function.priority  = MODULE_FUNC_PRIORITY_DEFAULT;

  // register function ids to can host, it will assign message id
  for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    if (function.id == MODULE_FUNC_ENCLOSURE_DOOR_STATE)
      message_id[i] = canhost.RegisterFunction(function, CallbackAckDoorState);
    else
      message_id[i] = canhost.RegisterFunction(function, NULL);
  }

  ret = canhost.BindMessageID(cmd, message_id);

  mac_index_ = mac_index;

  return ret;
}

ErrCode Enclosure::PostInit() {
  if (IsOnline() && ModuleBase::toolhead() == MODULE_TOOLHEAD_3DP) {
    Disable();
  }
  return E_SUCCESS; 
}

ErrCode Enclosure::SetLightBar(uint8_t brightness) {
  CanStdFuncCmd_t cmd;
  brightness = (brightness > 100) ? 100 : brightness;
  uint8_t out = brightness * 255 / 100;

  uint8_t         buffer[4];

  buffer[0] = 1;
  buffer[1] = out;
  buffer[2] = out;
  buffer[3] = out;

  LOG_I("Eclosure: set LIGHT power %u\n", brightness);

  cmd.id     = MODULE_FUNC_SET_ENCLOSURE_LIGHT;
  cmd.data   = buffer;
  cmd.length = 4;

  brightness_ = brightness;

  return canhost.SendStdCmd(cmd, 0);
}


ErrCode Enclosure::SetFanSpeed(uint8_t speed) {
  CanStdFuncCmd_t cmd;
  uint8_t         buffer[2];
  speed = (speed > 100) ? 100 : speed;
  uint8_t out = speed * 255 / 100;

  buffer[0] = 0;
  buffer[1] = out;

  LOG_I("Eclosure: set FAN speed %u\n", speed);

  cmd.id     = MODULE_FUNC_SET_ENCLOSURE_FAN;
  cmd.data   = buffer;
  cmd.length = 4;

  fan_speed_ = speed;

  return canhost.SendStdCmd(cmd, 0);
}


void Enclosure::ReportStatus() {
  if (IsOnline()) {
    SERIAL_ECHOLN("Enclosure online: On");
    SERIAL_ECHO("Enclosure: ");
    SERIAL_ECHOLN((enabled_)? "On" : "Off");
    SERIAL_ECHO("Enclosure door: ");
    SERIAL_ECHOLN((door_state_ == ENCLOSURE_DOOR_STATE_OPEN)? "Open" : "Closed");
    SERIAL_ECHO("Enclosure light power: ");
    SERIAL_PRINTLN(brightness_, DEC);
    SERIAL_ECHO("Enclosure fan power: ");
    SERIAL_PRINTLN(fan_speed_, DEC);
  }
  else {
    SERIAL_ECHOLN("Enclosure online: Off");
  }
}


void Enclosure::PollDoorState() {
  CanStdFuncCmd_t cmd;

  cmd.id = MODULE_FUNC_ENCLOSURE_DOOR_STATE;
  cmd.length = 0;

  canhost.SendStdCmd(cmd, 0);
}


void Enclosure::Disable() {
  LOG_I("disable door checking!\n");
  enabled_ = false;
  if (event_state_ == ENCLOSURE_EVENT_STATE_HANDLED_OPEN &&
      event_state_ == ENCLOSURE_EVENT_STATE_OPENED) {
    HandleDoorClosed();
    event_state_ = ENCLOSURE_EVENT_STATE_IDLE;
  }
}


void Enclosure::Enable() {
  LOG_I("enable door checking!\n");
  enabled_ = true;
  if (door_state_ == ENCLOSURE_DOOR_STATE_OPEN &&
      event_state_ == ENCLOSURE_EVENT_STATE_IDLE) {
    HandleDoorOpened();
  }
}


void Enclosure::HandleDoorOpened() {
  LOG_I("door opened!\n");
  systemservice.PauseTrigger(TRIGGER_SOURCE_DOOR_OPEN);
  if (laser.IsOnline())
    laser.SetPowerLimit(TOOLHEAD_LASER_POWER_SAFE_LIMIT);

  event_state_ = ENCLOSURE_EVENT_STATE_OPENED;
}


void Enclosure::HandleDoorClosed() {
  LOG_I("door closed!\n");
  systemservice.ClearSystemFaultBit(FAULT_FLAG_DOOR_OPENED);

  if (laser.IsOnline())
    laser.SetPowerLimit(TOOLHEAD_LASER_POWER_NORMAL_LIMIT);
}

void Enclosure::Process() {
  if (!IsOnline() || !enabled_)
    return;


  switch (event_state_) {
  case ENCLOSURE_EVENT_STATE_IDLE:
    // door is just opened
    if (door_state_ == ENCLOSURE_DOOR_STATE_OPEN) {
      HandleDoorOpened();
    }
    break;

  case ENCLOSURE_EVENT_STATE_OPENED:
    event_state_ = ENCLOSURE_EVENT_STATE_HANDLED_OPEN;
    break;

  case ENCLOSURE_EVENT_STATE_HANDLED_OPEN:
    // query if door is closed
    if(door_state_ == ENCLOSURE_DOOR_STATE_CLOSED) {
      HandleDoorClosed();
      event_state_ = ENCLOSURE_EVENT_STATE_CLOSED;
    }
    break;

  case ENCLOSURE_EVENT_STATE_CLOSED:
    event_state_ = ENCLOSURE_EVENT_STATE_IDLE;
    break;

  default:
    LOG_E("invalid enclosure door state!\n");
    break;
  }
}


ErrCode Enclosure::ReportStatus(SSTP_Event_t &event) {
  uint8_t buff[4];

  LOG_I("SC req enclosure sta\n");

  if (IsOnline()) {
    buff[0] = E_SUCCESS;
  }
  else {
    buff[0] = E_FAILURE;
  }

  buff[1] = brightness_;
  buff[2] = fan_speed_;
  buff[3] = enabled_;

  event.length = 4;
  event.data = buff;

  return hmi.Send(event);
}

ErrCode Enclosure::SetLightBar(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  if (event.length < 1) {
    LOG_E("must specify light power!\n");
    err = E_FAILURE;
    goto OUT;
  }

  SetLightBar(event.data[0]);

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

ErrCode Enclosure::SetFan(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  if (event.length < 1) {
    LOG_E("must specify Fan speed!\n");
    err = E_FAILURE;
    goto OUT;
  }

  SetFanSpeed(event.data[0]);

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

ErrCode Enclosure::SetDetection(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  if (event.length < 1) {
    LOG_E("must tell me what to do for enclosure detection!\n");
    err = E_FAILURE;
    goto OUT;
  }

  if (!IsOnline()) {
    LOG_E("Enclosure is offline!\n");
    err = E_HARDWARE;
    goto OUT;
  }

  if (event.data[0])
    Enable();
  else
    Disable();

  err = E_SUCCESS;

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}
