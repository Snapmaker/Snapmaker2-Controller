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
#include "../../../Marlin/src/module/temperature.h"
#include "../../Marlin/src/module/tool_change.h"
#include "../../../Marlin/src/module/probe.h"
#include "linear.h"

int32_t HEATER_0_MINTEMP = 5;
int32_t HEATER_1_MINTEMP = 5;
int32_t HEATER_2_MINTEMP = 5;
int32_t HEATER_3_MINTEMP = 5;
int32_t HEATER_4_MINTEMP = 5;
int32_t HEATER_5_MINTEMP = 5;
int32_t BED_MINTEMP      = 5;
int32_t CHAMBER_MINTEMP  = 5;

int32_t HEATER_0_MAXTEMP = 290;
int32_t HEATER_1_MAXTEMP = 315;
int32_t HEATER_2_MAXTEMP = 275;
int32_t HEATER_3_MAXTEMP = 275;
int32_t HEATER_4_MAXTEMP = 275;
int32_t HEATER_5_MAXTEMP = 275;
int32_t BED_MAXTEMP      = 130;
int32_t CHAMBER_MAXTEMP  = 100;

ToolHead3DP printer_single(MODULE_DEVICE_ID_3DP_SINGLE);
ToolHead3DP printer_dual(MODULE_DEVICE_ID_DUAL_EXTRUDER);

ToolHead3DP *printer1 = &printer_single;

static void CallbackAckProbeState(CanStdDataFrame_t &cmd) {
  printer1->is_probe_sensor_initialized_ = true;
  printer1->probe_state(cmd.data);
}

static void CallbackAckNozzleTemp(CanStdDataFrame_t &cmd) {
  printer1->SetTemp(cmd.data);
}

static void CallbackAckReportPidTemp(CanStdDataFrame_t &cmd) {
  // PID from module, was
  float val = (float)((cmd.data[1] << 24) | (cmd.data[2] << 16) | (cmd.data[3] << 8) | cmd.data[4]) / 1000;
  printer1->UpdatePID(cmd.data[0], val);
}

static void CallbackAckFilamentState(CanStdDataFrame_t &cmd) {
  // temperature from module, was
  printer1->is_filament_status_initialized_ = true;
  printer1->filament_state(cmd.data);
}

static void CallbackAckNozzleType(CanStdDataFrame_t &cmd) {
  printer1->SetNozzleType(cmd.data);
}

static void CallbackAckExtruderInfo(CanStdDataFrame_t &cmd) {
  printer1->SetExtruderInfo(cmd.data);
}

void ToolHead3DP::GetFilamentState() {
  CanStdFuncCmd_t fun_cmd = {MODULE_FUNC_RUNOUT_SENSOR_STATE, 0, NULL};
  canhost.SendStdCmd(fun_cmd);
  vTaskDelay(pdMS_TO_TICKS(20));
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
  uint8_t     func_buffer[2*15+2];

  Function_t    function;
  message_id_t  message_id[15];

  CanStdMesgCmd_t mesg_cmd = {MODULE_MESSAGE_ID_INVALID, 0, NULL};
  uint8_t msg_id_index_probe = 0xff;
  uint8_t msg_id_index_runout = 0xff;

  CanStdCmdCallback_t cb = NULL;

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS)
    return ret;

  if (device_id_ == MODULE_DEVICE_ID_3DP_SINGLE) {
    LOG_I("\tGot toolhead single extruder 3DP!\n");
  } else if (device_id_ == MODULE_DEVICE_ID_DUAL_EXTRUDER) {
    LOG_I("\tGot toolhead dual extruder 3DP!\n");
  }

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

    case MODULE_FUNC_REPORT_3DP_PID:
      cb = CallbackAckReportPidTemp;
      break;

    case MODULE_FUNC_REPORT_NOZZLE_TYPE:
      cb = CallbackAckNozzleType;
      break;

    case MODULE_REPORT_EXTRUDER_INFO:
      cb = CallbackAckExtruderInfo;
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
  vTaskDelay(pdMS_TO_TICKS(5));

  if (msg_id_index_probe != 0xff) {
    mesg_cmd.id = message_id[msg_id_index_probe];
    canhost.SendStdCmd(mesg_cmd);
  }

  vTaskDelay(pdMS_TO_TICKS(5));
  if (msg_id_index_runout != 0xff) {
    mesg_cmd.id = message_id[msg_id_index_runout];
    canhost.SendStdCmd(mesg_cmd);
  }

  vTaskDelay(pdMS_TO_TICKS(5));

  LOG_I("\tprobe: 0x%x, filament: 0x%x\n", probe_state_, filament_state_);

  IOInit();
  if (device_id_ == MODULE_DEVICE_ID_3DP_SINGLE) {
    is_filament_status_initialized_ = true;
    is_probe_sensor_initialized_    = true;
    is_nozzle_type_initialized_     = true;
    HEATER_0_MAXTEMP = 290;
    xprobe_offset_from_extruder = X_PROBE_OFFSET_FROM_EXTRUDER;
    yprobe_offset_from_extruder = Y_PROBE_OFFSET_FROM_EXTRUDER;
    z_position_before_calibration = 20;
    SetToolhead(MODULE_TOOLHEAD_3DP);
  } else if (device_id_ == MODULE_DEVICE_ID_DUAL_EXTRUDER) {
    is_filament_status_initialized_ = false;
    is_probe_sensor_initialized_    = false;
    is_nozzle_type_initialized_     = false;
    HEATER_0_MAXTEMP = 315;
    HEATER_1_MAXTEMP = 315;
    xprobe_offset_from_extruder = DUAL_EXTRUDER_X_PROBE_OFFSET_FROM_EXTRUDER;
    yprobe_offset_from_extruder = DUAL_ETTRUDER_Y_PROBE_OFFSET_FROM_EXTRUDER;
    z_position_before_calibration = 50;
    SetToolhead(MODULE_TOOLHEAD_DUAL_EXTRUDER);
  }
  printer1 = this;

out:
  return ret;
}


ErrCode ToolHead3DP::SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time) {
  CanStdFuncCmd_t cmd;

  uint8_t buffer[2];

  buffer[0] = delay_time;
  buffer[1] = speed;

  switch ((uint8_t)fan_index) {
    case 0:
      cmd.id = MODULE_FUNC_SET_FAN1;
      break;
    case 1:
      cmd.id = MODULE_FUNC_SET_FAN2;
      break;
    case 2:
      cmd.id = MODULE_FUNC_SET_FAN3;
      break;
    default:
      return E_FAILURE;
  }

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

float * ToolHead3DP::GetPID(uint8_t extrude_index) {
  CanStdFuncCmd_t cmd = {MODULE_FUNC_REPORT_3DP_PID, 0, NULL};
  canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(200));
  return pid_;
}


ErrCode ToolHead3DP::SetHeater(uint16_t target_temp, uint8_t extrude_index) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[2*EXTRUDERS];

  if (extrude_index >= EXTRUDERS) {
    return E_PARAM;
  }

  target_temp_[extrude_index] = target_temp;
  LOG_I("Set T%d, T0=%d, T1=%d\n", extrude_index, target_temp_[0], target_temp_[1]);

  for (int i = 0; i < EXTRUDERS; i++) {
    buffer[2*i + 0] = (uint8_t)(target_temp_[i]>>8);
    buffer[2*i + 1] = (uint8_t)target_temp_[i];
  }

  fan_e module_fan_index = SINGLE_EXTRUDER_MODULE_FAN;
  fan_e nozzle_fan_index = SINGLE_EXTRUDER_NOZZLE_FAN;
  uint8_t fan_speed = 0;
  uint8_t fan_delay = 0;

  if (target_temp >= 60) {
    fan_speed = 255;
  } else if (target_temp == 0) {
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
    module_fan_index = SINGLE_EXTRUDER_MODULE_FAN;
    nozzle_fan_index = SINGLE_EXTRUDER_NOZZLE_FAN;
  }
  else if (printer1->device_id() == MODULE_DEVICE_ID_DUAL_EXTRUDER) {
    if (extrude_index == TOOLHEAD_3DP_EXTRUDER0) {
      module_fan_index = DUAL_EXTRUDER_LEFT_MODULE_FAN;
    }
    else if (extrude_index == TOOLHEAD_3DP_EXTRUDER1) {
      module_fan_index = DUAL_EXTRUDER_RIGHT_MODULE_FAN;
    }
    nozzle_fan_index = DUAL_EXTRUDER_NOZZLE_FAN;
  }

  SetFan((uint8_t)module_fan_index, fan_speed, fan_delay);
  SetFan((uint8_t)nozzle_fan_index, fan_speed, fan_delay);

  cmd.id     = MODULE_FUNC_SET_NOZZLE_TEMP;
  cmd.data   = buffer;
  cmd.length = 2 * EXTRUDERS;

  return canhost.SendStdCmd(cmd, 0);
}

void ToolHead3DP::SetNozzleType(uint8_t *data) {
  if (nozzle_type_[0] != (nozzle_type_t)data[0]) {
    nozzle_type_[0] = (nozzle_type_t)data[0];
    need_to_tell_hmi_nozzle_type_ = true;
  }

  if (nozzle_type_[1] != (nozzle_type_t)data[1]) {
    nozzle_type_[1] = (nozzle_type_t)data[1];
    need_to_tell_hmi_nozzle_type_ = true;
  }
}

void ToolHead3DP::SetExtruderInfo(uint8_t *data) {
  extruder_status_ = data[0];
  active_extruder  = data[1];
  need_to_tell_hmi_extruder_info_ = true;
}

nozzle_type_t ToolHead3DP::GetNozzleType(uint8_t e) {
  if (e >= EXTRUDERS) {
    return NOZZLE_TYPE_INVALID;
  }

  return nozzle_type_[e];
}

void ToolHead3DP::NozzleStatusCheck() {
  for (uint32_t i=0; i < EXTRUDERS; i++) {
    if (nozzle_type_[i] == NOZZLE_TYPE_INVALID) {
      thermalManager.setTargetHotend(0, i);
    }
  }
}

ErrCode ToolHead3DP::SwitchExtruder(uint8_t extrude_index) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[1];

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

  can_buffer[0] = extrude_index;
  cmd.id        = MODULE_FUNC_SWITCH_EXTRUDER;
  cmd.data      = can_buffer;
  cmd.length    = 1;

  return canhost.SendStdCmd(cmd);
}

void ToolHead3DP::SwitchExtruderWithoutMove(uint8_t extruder_index) {
  if (extruder_index >= EXTRUDERS) return;

  active_extruder = extruder_index;
  target_extruder = active_extruder;
  SwitchExtruder(extruder_index);
}

void ToolHead3DP::SetProbeSensor(probe_sensor_t sensor) {
  if (sensor >= PROBE_SENSOR_INVALID) return;
  active_probe_sensor_ = sensor;
}

ErrCode ToolHead3DP::SendNozzleTypeToHmi() {
  uint8_t buff[2];
  SSTP_Event_t event_tmp = {EID_SYS_CTRL_ACK, SYSCTL_OPC_GET_NOZZLE_TYPE};
  buff[0] = (uint8_t)nozzle_type_[0];
  buff[1] = (uint8_t)nozzle_type_[1];
  event_tmp.length = 2;
  event_tmp.data = buff;
  return hmi.Send(event_tmp);
}

ErrCode ToolHead3DP::SendExtruderInfoToHmi() {
  SSTP_Event_t event = {EID_SETTING_ACK, SETTINGS_OPC_EXTRUDER_STATUS_CTRL};
  uint8_t buf[3];
  buf[0] = extruder_status_;
  buf[1] = active_extruder;
  buf[2] = target_extruder;
  event.data = buf;
  event.length = 3;
  return hmi.Send(event);
}

ErrCode ToolHead3DP::SendFilamentStateToHmi() {
  uint8_t buff[2];
  SSTP_Event_t event_tmp = {EID_SYS_CTRL_ACK, SYSCTL_OPC_GET_FILAMENT_STATE};
  buff[0] = (uint8_t)(!filament_state(TOOLHEAD_3DP_EXTRUDER0));
  buff[1] = (uint8_t)(!filament_state(TOOLHEAD_3DP_EXTRUDER1));
  event_tmp.length = 2;
  event_tmp.data = buff;
  return hmi.Send(event_tmp);
}

ErrCode ToolHead3DP::SendNozzleTempToHmi() {
  uint8_t buff[8];
  int16_t temp;
  SSTP_Event_t event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_GET_NOZZLE_TEMP};
  temp = (int16_t)thermalManager.degHotend(TOOLHEAD_3DP_EXTRUDER0);
  buff[0] = ((uint8_t *)&temp)[1];
  buff[1] = ((uint8_t *)&temp)[0];
  temp = (int16_t)thermalManager.degTargetHotend(TOOLHEAD_3DP_EXTRUDER0);
  buff[2] = ((uint8_t *)&temp)[1];
  buff[3] = ((uint8_t *)&temp)[0];
  temp = (int16_t)thermalManager.degHotend(TOOLHEAD_3DP_EXTRUDER1);
  buff[4] = ((uint8_t *)&temp)[1];
  buff[5] = ((uint8_t *)&temp)[0];
  temp = (int16_t)thermalManager.degTargetHotend(TOOLHEAD_3DP_EXTRUDER1);
  buff[6] = ((uint8_t *)&temp)[1];
  buff[7] = ((uint8_t *)&temp)[0];
  event.length = 8;
  event.data = buff;
  return hmi.Send(event);
}

void ToolHead3DP::SetExtruderCheck(extruder_status_e status) {
  CanStdFuncCmd_t cmd;
  cmd.id        = MODULE_SET_EXTRUDER_CHECK;
  cmd.data      = (uint8_t *)&status;
  cmd.length    = 1;
  canhost.SendStdCmd(cmd);
}

ErrCode ToolHead3DP::ExtruderStatusCtrl(SSTP_Event_t &event) {
  if (event.length != 2) {
    return E_PARAM;
  }

  if (event.data[0] == 1 && event.data[1] > TOOLHEAD_3DP_EXTRUDER1) {
    return E_PARAM;
  }

  if (event.data[0] == 0 || (event.data[0] == 1 && event.data[1] == active_extruder)) {
    return SendExtruderInfoToHmi();
  } else {
    SwitchExtruder(event.data[1]);
    tool_change(event.data[1]);
  }

  return E_SUCCESS;
}

ErrCode ToolHead3DP::ExtruderStrokeCalibration(SSTP_Event_t &event) {
  if (event.length != 2) {
    return E_PARAM;
  }

  return levelservice.CalibrateExtruderTriggerStroke(event.data[0], event.data[1]);
}

ErrCode ToolHead3DP::ExtruderStrokeConfirm(SSTP_Event_t &event) {
  ErrCode ret;
  uint8_t buf[9];
  uint32_t index = 0;

  if (event.length != 1 || event.data[0] > 1) {
    return E_PARAM;
  }

  ret = levelservice.ConfirmExtruderTriggerStroke(event.data[0]);
  if (ret == E_SUCCESS) {
    buf[index++] = 0;
  } else {
    buf[index++] = 1;
  }

  uint32_t tmp1, tmp2;
  tmp1 = (uint32_t)(switch_stroke_extruder0*1000);
  tmp2 = (uint32_t)(switch_stroke_extruder1*1000);
  buf[index++] = tmp1 >> 24;
  buf[index++] = tmp1 >> 16;
  buf[index++] = tmp1 >> 8;
  buf[index++] = tmp1 & 0xff;
  buf[index++] = tmp2 >> 24;
  buf[index++] = tmp2 >> 16;
  buf[index++] = tmp2 >> 8;
  buf[index++] = tmp2 & 0xff;

  event.data = buf;
  event.id = EID_SETTING_ACK;
  event.op_code = SETTINGS_OPC_STROKE_CONFIRM;
  event.length = index;
  return hmi.Send(event);
}

ErrCode ToolHead3DP::SetHotendOffset(SSTP_Event_t &event) {
  if (event.length != 5 || event.data[0] > 3) {
    return E_PARAM;
  }

  float tmp_offset;
  uint8_t buf[13] = {0};
  uint8_t index = 0;
  float nozzle_offset[XYZ][HOTENDS] = DEFAULT_HOTEND_OFFSETS;
  tmp_offset = (event.data[1]<<24 | event.data[2]<<16 | event.data[3]<<8 | event.data[4])/1000;
  if (event.data[0] == 0) {
    buf[index++] = 0;
  } else {
    if (tmp_offset > nozzle_offset[event.data[0]][TOOLHEAD_3DP_EXTRUDER1] + HOTEND_OFFSET_MAX_DEVIATION ||
        tmp_offset < nozzle_offset[event.data[0]][TOOLHEAD_3DP_EXTRUDER1] - HOTEND_OFFSET_MAX_DEVIATION) {
      buf[index++] = 1;
    } else {
      hotend_offset[event.data[0] - 1][TOOLHEAD_3DP_EXTRUDER1] = tmp_offset;
    }
  }

  int32_t tmp;
  tmp = hotend_offset[X_AXIS][TOOLHEAD_3DP_EXTRUDER1] * 1000;
  buf[index++] = tmp >> 24;
  buf[index++] = tmp >> 16;
  buf[index++] = tmp >> 8;
  buf[index++] = tmp;
  tmp = hotend_offset[Y_AXIS][TOOLHEAD_3DP_EXTRUDER1] * 1000;
  buf[index++] = tmp >> 24;
  buf[index++] = tmp >> 16;
  buf[index++] = tmp >> 8;
  buf[index++] = tmp;
  tmp = hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] * 1000;
  buf[index++] = tmp >> 24;
  buf[index++] = tmp >> 16;
  buf[index++] = tmp >> 8;
  buf[index++] = tmp;

  event.length = index;
  event.id = EID_SETTING_ACK;
  event.op_code = SETTINGS_OPC_STROKE_CONFIRM;
  return hmi.Send(event);
}

ErrCode ToolHead3DP::SetFanSpeed(SSTP_Event_t &event) {
  if (event.length != 2 || event.data[0] > 2) {
    return E_PARAM;
  }

  SetFan(event.data[0], event.data[1]);

  event.data = fan_speed_;
  event.length = 3;
  event.id = EID_SETTING_ACK;
  event.op_code = SETTINGS_OPC_STROKE_CONFIRM;
  return hmi.Send(event);
}

ErrCode ToolHead3DP::LeftStopperAssemble(SSTP_Event_t &event) {
  if (event.length != 1 || event.data[0] > 1) {
    return E_PARAM;
  }

  float position;
  if (event.data[0] == 0) {
    position = 0;
    if ((linear_p->machine_size() == MACHINE_SIZE_A250) || (linear_p->machine_size() == MACHINE_SIZE_A350)) {
      current_position[X_AXIS] += STOPPER_ASSEMBLE_LEFT_SAFTY_SAPCING;
      sync_plan_position();
    } else if (linear_p->machine_size() == MACHINE_SIZE_A150) {
      current_position[X_AXIS] -= STOPPER_ASSEMBLE_LEFT_SAFTY_SAPCING;
      sync_plan_position();
    }
  } else if (event.data[0] == 1) {
    position = DUAL_EXTRUDER_X_MAX_POS/2;
    current_position[X_AXIS] -= STOPPER_ASSEMBLE_LEFT_SAFTY_SAPCING;
    sync_plan_position();
  }

  do_blocking_move_to_xy(position, current_position[Y_AXIS], 100);
  planner.synchronize();

  event.data = NULL;
  event.length = 0;
  event.id = EID_MOTION_ACK;
  event.op_code = MOTION_OPC_LEFT_STOPPER_ASSEMBLE;
  return hmi.Send(event);
}

ErrCode ToolHead3DP::RightStopperAssemble(SSTP_Event_t &event) {
  if (event.length != 1 || event.data[0] > 1) {
    return E_PARAM;
  }

  float position;
  if (event.data[0] == 0) {
    position = X_MAX_POS;
    if (linear_p->machine_size() == MACHINE_SIZE_A150) {
      current_position[X_AXIS] -= STOPPER_ASSEMBLE_LEFT_SAFTY_SAPCING;
      sync_plan_position();
    }
  } else if (event.data[0] == 1) {
    position = DUAL_EXTRUDER_X_MAX_POS/2;
  }

  do_blocking_move_to_xy(position, current_position[Y_AXIS], 100);
  planner.synchronize();

  event.data = NULL;
  event.length = 0;
  event.id = EID_MOTION_ACK;
  event.op_code = MOTION_OPC_LEFT_STOPPER_ASSEMBLE;
  return hmi.Send(event);
}

ErrCode ToolHead3DP::DoXYCalibration(SSTP_Event_t &event) {
  uint8_t action;
  xy_calibration_param_t cal_param;

  action = event.data[0];
  if (action == 0) {
    levelservice.DoXCalibration(cal_param);
  } else if (action == 1) {
    levelservice.DoYCalibration(cal_param);
  }

  event.data = &action;
  event.length = 1;
  event.id = EID_SETTING_ACK;
  event.op_code = SETTINGS_OPC_XY_CALIBRATION;
  return hmi.Send(event);
}

ErrCode ToolHead3DP::XYCalibrationConfirm(SSTP_Event_t &event) {
  float lines = (event.data[1] << 24 | event.data[2] << 16 | event.data[3] << 8 | event.data[4]) / 1000;

  if (event.data[0] == 0) {
    levelservice.ApplyXCalibration(lines);
  } else if (event.data[0] == 1) {
    levelservice.ApplyYCalibration(lines);
  }

  return E_SUCCESS;
}

void ToolHead3DP::UpdateFilamentStatus() {
  CanStdFuncCmd_t cmd;
  cmd.id        = MODULE_FUNC_RUNOUT_SENSOR_STATE;
  cmd.data      = NULL;
  cmd.length    = 0;
  canhost.SendStdCmd(cmd);
}

void ToolHead3DP::UpdateProbeSensorStatus() {
  CanStdFuncCmd_t cmd;
  cmd.id        = MODULE_FUNC_PROBE_STATE;
  cmd.data      = NULL;
  cmd.length    = 0;
  canhost.SendStdCmd(cmd);
}

void ToolHead3DP::UpdateNozzleType() {
  CanStdFuncCmd_t cmd;
  cmd.id        = MODULE_FUNC_REPORT_NOZZLE_TYPE;
  cmd.data      = NULL;
  cmd.length    = 0;
  canhost.SendStdCmd(cmd);
}

void ToolHead3DP::Process() {
  if (need_to_tell_hmi_nozzle_type_) {
    need_to_tell_hmi_nozzle_type_ = false;
    LOG_I("nozzle type changed, nozzle0: %d, nozzle1: %d\n", nozzle_type_[0], nozzle_type_[1]);
    NozzleStatusCheck();
    SendNozzleTypeToHmi();
  }

  if (need_to_tell_hmi_extruder_info_) {
    need_to_tell_hmi_extruder_info_ = false;
    LOG_I("extruder info, active_extruder: %d, target_extruder: %d\n", active_extruder, target_extruder);
    SendExtruderInfoToHmi();
  }

  if (++timer_in_process_ < 100) return;
  timer_in_process_ = 0;

  if (is_filament_status_initialized_ == false) {
    UpdateFilamentStatus();
  }

  if (is_probe_sensor_initialized_ == false) {
    UpdateProbeSensorStatus();
  }

  if (is_nozzle_type_initialized_ == false) {
    UpdateNozzleType();
  }
}