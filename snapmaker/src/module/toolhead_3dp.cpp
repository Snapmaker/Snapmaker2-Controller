#include "toolhead_3dp.h"

#include "../common/config.h"
#include "common/debug.h"

// marlin headers
#include "src/core/macros.h"
#include "src/core/boards.h"
#include "Configuration.h"
#include "src/pins/pins.h"

ToolHead3DP printer;


static void CallbackAckProbeState(CanStdDataFrame_t &cmd) {
  printer.probe_state(cmd.data[0]);
}


static void CallbackAckNozzleTemp(CanStdDataFrame_t &cmd) {
  // temperature from module, was
  printer.SetTemp(cmd.data[0]<<8 | cmd.data[1], 0);
}


static void CallbackAckFilamentState(CanStdDataFrame_t &cmd) {
  // temperature from module, was
  printer.filament_state(cmd.data[0], 0);
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

  LOG_I("\tGot 3D printer!\n");

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

  // register function ids to can host, it will assign message id
  for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    function.priority  = MODULE_FUNC_PRIORITY_DEFAULT;
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
  }

  ret = canhost.BindMessageID(cmd, message_id);
  if (ret != E_SUCCESS) {
    LOG_E("\tfailed to bind message id!\n");
    goto out;
  }

  mac_index_ = mac_index;

  // read the state of sensors
  vTaskDelay(portTICK_PERIOD_MS * 5);

  if (msg_id_index_probe != 0xff) {
    mesg_cmd.id = message_id[msg_id_index_probe];
    canhost.SendStdCmd(mesg_cmd);
  }

  vTaskDelay(portTICK_PERIOD_MS * 5);
  if (msg_id_index_runout != 0xff) {
    mesg_cmd.id = message_id[msg_id_index_runout];
    canhost.SendStdCmd(mesg_cmd);
  }

  vTaskDelay(portTICK_PERIOD_MS * 5);

  LOG_I("\tprobe: %u, filament: %u\n", probe_state_, filament_state_[0]);

  toolhead_ = MODULE_TOOLHEAD_3DP;

out:
  return ret;
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


void ToolHead3DP::Process() {
  if (++timer_in_process_ < 1000) return;

  timer_in_process_ = 0;


}