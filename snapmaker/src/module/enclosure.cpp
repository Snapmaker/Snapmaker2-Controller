#include "enclosure.h"

#include "../common/config.h"
#include "../common/debug.h"
#include "../module/toolhead_laser.h"
#include "../service/system.h"

Enclosure enclosure;


static void CallbackAckDoorState(uint8_t *cmd, uint8_t length) {
  enclosure.door_state(cmd[0]);
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

  ret = canhost.BindMessageID(func_buffer, message_id);

  mac_index_ = mac_index;

  return ret;
}


ErrCode Enclosure::SetLightBar(uint8_t brightness) {
  CanStdFuncCmd_t cmd;
  uint8_t         buffer[4];

  buffer[0] = 1;
  buffer[1] = brightness;
  buffer[2] = brightness;
  buffer[3] = brightness;

  cmd.id     = MODULE_FUNC_SET_ENCLOSURE_LIGHT;
  cmd.data   = buffer;
  cmd.length = 4;

  brightness_ = brightness;

  return canhost.SendStdCmd(cmd, 0);
}


ErrCode Enclosure::SetFanSpeed(uint8_t speed) {
  CanStdFuncCmd_t cmd;
  uint8_t         buffer[2];

  buffer[0] = 0;
  buffer[1] = speed;

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
  if (event_state_ == ENCLOSURE_EVENT_STATE_HANDLED_OPEN &&
      event_state_ == ENCLOSURE_EVENT_STATE_OPENED) {
    HandleDoorClosed();
    event_state_ = ENCLOSURE_EVENT_STATE_IDLE;
  }
}


void Enclosure::Enable() {
  if (door_state_ == ENCLOSURE_DOOR_STATE_OPEN &&
      event_state_ == ENCLOSURE_EVENT_STATE_IDLE) {
    HandleDoorOpened();
  }
}


void Enclosure::HandleDoorOpened() {
  LOG_I("door opened!\n");
  systemservice.PauseTrigger(TRIGGER_SOURCE_DOOR_OPEN);
  if (laser.IsOnline())
    laser.ChangePowerLimit(TOOLHEAD_LASER_POWER_SAFE_LIMIT);

  event_state_ = ENCLOSURE_EVENT_STATE_OPENED;
}


void Enclosure::HandleDoorClosed() {
  systemservice.ClearSystemFaultBit(FAULT_FLAG_DOOR_OPENED);

  if (laser.IsOnline())
    laser.ChangePowerLimit(TOOLHEAD_LASER_POWER_NORMAL_LIMIT);
}

void Enclosure::Process() {
  if (!enabled_)
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


