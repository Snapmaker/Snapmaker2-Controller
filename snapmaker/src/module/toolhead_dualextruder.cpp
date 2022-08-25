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

#include "toolhead_dualextruder.h"
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
#include "../../../Marlin/src/feature/bedlevel/bedlevel.h"
#include "../../../Marlin/src/module/tool_change.h"


#define CAN_FRAME_SIZE                         8
#define DEFAULT_hotend_offsetX                 26
#define DEFAULT_hotend_offsetY                 0
#define DEFAULT_hotend_offsetZ                 -1.5
#define BIAS_hotend_offsetX                    1.2
#define BIAS_hotend_offsetY                    1.2
#define BIAS_hotend_offsetZ                    1.2


ToolHeadDualExtruder printer_dualextruder(MODULE_DEVICE_ID_DUAL_EXTRUDER);

static void CallbackAckProbeState(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportProbeState(cmd.data);
}

static void CallbackAckNozzleTemp(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportTemperature(cmd.data);
}

static void CallbackAckReportPidTemp(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportPID(cmd.data);
}

static void CallbackAckFilamentState(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportFilamentState(cmd.data);
}

static void CallbackAckNozzleType(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportHotendType(cmd.data);
}

static void CallbackAckExtruderInfo(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportExtruderInfo(cmd.data);
}

static void CallbackAckReportHotendOffset(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportHotendOffset(cmd.data);
}

static void CallbackAckReportProbeSensorCompensation(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportProbeSensorCompensation(cmd.data);
}

static void CallbackAckReportRightExtruderPos(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportRightExtruderPos(cmd.data);
}

ErrCode ToolHeadDualExtruder::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;
  CanExtCmd_t cmd;
  uint8_t     func_buffer[2*50 + 2];
  Function_t    function;
  message_id_t  message_id[50];
  CanStdCmdCallback_t cb = NULL;

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS)
    return ret;

  LOG_I("\tGot toolhead dualextruder!\n");

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
      break;

    case MODULE_FUNC_RUNOUT_SENSOR_STATE:
      cb = CallbackAckFilamentState;
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

    case MODULE_FUNC_REPORT_HOTEND_OFFSET:
      cb = CallbackAckReportHotendOffset;
      break;

    case MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION:
      cb = CallbackAckReportProbeSensorCompensation;
      break;

    case MODULE_FUNC_REPORT_RIGHT_EXTRUDER_POS:
      cb = CallbackAckReportRightExtruderPos;
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

  IOInit();
  mac_index_ = mac_index;
  UpdateEAxisStepsPerUnit(MODULE_TOOLHEAD_DUALEXTRUDER);
  SetToolhead(MODULE_TOOLHEAD_DUALEXTRUDER);
  printer1 = this;

  // sync the state of sensors
  ModuleCtrlProbeStateSync();
  ModuleCtrlPidSync();
  ModuleCtrlHotendTypeSync();
  ModuleCtrlFilamentStateSync();
  ModuleCtrlHotendOffsetSync();
  ModuleCtrlZProbeSensorCompensationSync();
  ModuleCtrlRightExtruderPosSync();

  LOG_I("dualextruder ready!\n");

out:
  return ret;
}

void ToolHeadDualExtruder::ReportProbeState(uint8_t state[]) {
  if (state[0]) {
    probe_state_ |= 0x01;
  } else {
    probe_state_ &= ~0x01;
  }

  if (state[1]) {
    probe_state_ |= 0x02;
  } else {
    probe_state_ &= ~0x02;
  }

  if (state[2]) {
    probe_state_ |= 0x04;
  } else {
    probe_state_ &= ~0x04;
  }
}

void ToolHeadDualExtruder::ReportTemperature(uint8_t *data) {
  cur_temp_[0] = data[0] << 8 | data[1];
  cur_temp_[1] = data[4] << 8 | data[5];
}

void ToolHeadDualExtruder::ReportPID(uint8_t *data) {
  uint8_t param = data[0];
  float val = (float)((data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]) / 1000;
  switch (param) {
    case 0:
      #ifdef USE_FDM_INTERRUPT_LOG
        LOG_I("P: %f\n", val);
      #endif
      pid_[0] = val;
      PID_PARAM(Kp, 0) = val;
      break;
    case 1:
      #ifdef USE_FDM_INTERRUPT_LOG
      LOG_I("I: %f\n", val);
      #endif
      pid_[1] = val;
      PID_PARAM(Ki, 0) = val;
      break;
    case 2:
      #ifdef USE_FDM_INTERRUPT_LOG
        LOG_I("D: %f\n", val);
      #endif
      pid_[2] = val;
      PID_PARAM(Kd, 0) = val;
      break;
    default:
      break;
  }
}

void ToolHeadDualExtruder::ReportFilamentState(uint8_t state[]) {
  if (!state[0])
    filament_state_ |= 0x01;
  else
    filament_state_ &= ~0x01;

  if (!state[1])
    filament_state_ |= 0x02;
  else
    filament_state_ &= ~0x02;
}

void ToolHeadDualExtruder::ReportHotendType(uint8_t *data) {
  if (hotend_type_initialized_ == false) {
    hotend_type_initialized_ = true;
    for (uint32_t i = 0; i < EXTRUDERS; i++) {
      if (data[i] < HOTEND_INFO_MAX) {
        hotend_type_[i] = hotend_info[data[i]].model;
        hotend_diameter_[i] = hotend_info[data[i]].diameter;
      }

      #ifdef USE_FDM_INTERRUPT_LOG
        LOG_I("nozzle_index: %d, type: %d\n", i, hotend_type[i]);
      #endif
    }
  } else {
    // report error
    // todo
  }
}

void ToolHeadDualExtruder::ReportExtruderInfo(uint8_t *data) {
  uint8_t extruder_state = data[0];

  if (extruder_state) {
    // 报错
    // TODO
  } else {
    // 清除错误
    // TODO
  }
}

void ToolHeadDualExtruder::ReportHotendOffset(uint8_t *data) {
  uint8_t axis = data[0];
  float offset;
  ((uint8_t *)&offset)[0] = data[1];
  ((uint8_t *)&offset)[1] = data[2];
  ((uint8_t *)&offset)[2] = data[3];
  ((uint8_t *)&offset)[3] = data[4];

  #ifdef USE_FDM_INTERRUPT_LOG
    LOG_I("axis: %d, offset: %f\n", axis, offset);
  #endif
  if (isnan(offset)) {
    switch (axis) {
      case X_AXIS:
        hotend_offset[X_AXIS][1] = DEFAULT_hotend_offsetX;
        break;
      case Y_AXIS:
        hotend_offset[Y_AXIS][1] = DEFAULT_hotend_offsetY;
        break;
      case Z_AXIS:
        hotend_offset[Z_AXIS][1] = DEFAULT_hotend_offsetZ;
        break;
    }
  } else {
    hotend_offset[axis][1] = offset;
    switch(axis) {
      case X_AXIS:
        if ((hotend_offset[axis][1] < DEFAULT_hotend_offsetX - BIAS_hotend_offsetX) || (hotend_offset[axis][1] > DEFAULT_hotend_offsetX + BIAS_hotend_offsetX)) {
          hotend_offset[axis][1] = DEFAULT_hotend_offsetX;
          #ifdef USE_FDM_INTERRUPT_LOG
            LOG_E("report x hotend offset error: %f\n", offset);
          #endif
        }
        break;
      case Y_AXIS:
        if ((hotend_offset[axis][1] < DEFAULT_hotend_offsetY - BIAS_hotend_offsetY) || (hotend_offset[axis][1] > DEFAULT_hotend_offsetY + BIAS_hotend_offsetY)) {
          hotend_offset[axis][1] = DEFAULT_hotend_offsetY;
          #ifdef USE_FDM_INTERRUPT_LOG
            LOG_E("report y hotend offset error: %f\n", offset);
          #endif
        }
        break;
      case Z_AXIS:
        if ((hotend_offset[axis][1] < DEFAULT_hotend_offsetZ - BIAS_hotend_offsetZ) || (hotend_offset[axis][1] > DEFAULT_hotend_offsetZ + BIAS_hotend_offsetZ)) {
          hotend_offset[axis][1] = DEFAULT_hotend_offsetZ;
          #ifdef USE_FDM_INTERRUPT_LOG
            LOG_E("report z hotend offset error: %f\n", offset);
          #endif
        }
        break;
    }
  }
}

void ToolHeadDualExtruder::ReportProbeSensorCompensation(uint8_t *data) {
  uint8_t e = data[0];
  float compensation = (float)((data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]) / 1000;

  // bedlevel_svc.z_compensation_[e] = compensation;
  // todo

  #ifdef USE_FDM_INTERRUPT_LOG
    LOG_I("extruder: %d, compensation: %f\n", e, compensation);
  #endif
}

void ToolHeadDualExtruder::ReportRightExtruderPos(uint8_t *data) {
  float raise_for_home_pos = (float)(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) / 1000;
  float z_max_pos = (float)(data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]) / 1000;

  LOG_I("raise_for_home_pos: %f, z_max_pos: %f\n", raise_for_home_pos, z_max_pos);
}

ErrCode ToolHeadDualExtruder::ModuleCtrlProximitySwitchPower(uint8_t state) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[CAN_FRAME_SIZE];

  uint8_t index = 0;
  can_buffer[index++] = state;

  cmd.id        = MODULE_FUNC_PROXIMITY_SWITCH_POWER_CTRL;
  cmd.data      = can_buffer;
  cmd.length    = index;

  return canhost.SendStdCmdSync(cmd, 2000);
}

ErrCode ToolHeadDualExtruder::SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time /* =0 */) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];

  uint8_t index = 0;
  buffer[index++] = delay_time;
  buffer[index++] = speed;

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
  cmd.length = index;
  fan_speed_[fan_index] = speed;
  return canhost.SendStdCmd(cmd, 0);
}

ErrCode ToolHeadDualExtruder::SetHeater(uint16_t target_temp, uint8_t extrude_index /*=0*/) {
  if (extrude_index >= EXTRUDERS) {
    return E_PARAM;
  }

  if (hotend_type_[extrude_index] == 0xff) {
    LOG_E("hotend %d is invalid\n", extrude_index);
    return E_HARDWARE;
  }

  hotend_temp_[extrude_index].target = target_temp;
  LOG_I("Set T%d=%d\n", extrude_index, hotend_temp_[extrude_index].target);

  fan_e nozzle_fan_index = DUAL_EXTRUDER_NOZZLE_FAN;
  uint8_t fan_speed = 0;
  uint8_t fan_delay = 0;
  if (hotend_temp_[extrude_index].target >= 60) {
    fan_speed = 255;
  } else if (hotend_temp_[extrude_index].target == 0) {
    // check if need to delay to turn off fan
    if (hotend_temp_[extrude_index].current >= 150) {
      fan_speed = 0;
      fan_delay = 120;
    } else if (hotend_temp_[extrude_index].target >= 60) {
      fan_speed = 0;
      fan_delay = 60;
    } else {
      fan_speed = 0;
      fan_delay = 0;
    }
  }
  SetFan((uint8_t)nozzle_fan_index, fan_speed, fan_delay);

  uint8_t buffer[2*EXTRUDERS];
  for (int i = 0; i < EXTRUDERS; i++) {
    buffer[2*i + 0] = (uint8_t)(hotend_temp_[i].target>>8);
    buffer[2*i + 1] = (uint8_t)hotend_temp_[i].target;
  }
  CanStdFuncCmd_t cmd;
  cmd.id     = MODULE_FUNC_SET_NOZZLE_TEMP;
  cmd.data   = buffer;
  cmd.length = 2 * EXTRUDERS;

  return canhost.SendStdCmd(cmd, 0);
}

ErrCode ToolHeadDualExtruder::ModuleCtrlProbeStateSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_PROBE_STATE;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(5));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlPidSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_3DP_PID;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(5));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlHotendTypeSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_NOZZLE_TYPE;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(5));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlFilamentStateSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_RUNOUT_SENSOR_STATE;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(5));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlHotendOffsetSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_HOTEND_OFFSET;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(5));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlZProbeSensorCompensationSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(5));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlRightExtruderPosSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_RIGHT_EXTRUDER_POS;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(5));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSetPid(float p, float i, float d) {
  CanStdFuncCmd_t cmd;
  uint32_t val[3];
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;

  pid_[0] = p;
  pid_[1] = i;
  pid_[2] = d;
  val[0] = (uint32_t)(p*1000);
  val[1] = (uint32_t)(i*1000);
  val[2] = (uint32_t)(d*1000);

  for (int32_t i = 0; i < 3; i++) {
    index = 0;
    buffer[index++] = i;
    buffer[index++] = (uint8_t)(val[i]>>24);
    buffer[index++] = (uint8_t)(val[i]>>16);
    buffer[index++] = (uint8_t)(val[i]>>8);
    buffer[index++] = (uint8_t)(val[i]);
    cmd.id     = MODULE_FUNC_SET_3DP_PID;
    cmd.data   = buffer;
    cmd.length = index;
    ErrCode ret = canhost.SendStdCmd(cmd, 0);

    if (ret != E_SUCCESS) {
      LOG_E("failed to set hotend pid, ret: %u\n", ret);
      return ret;
    }
  }

  return E_SUCCESS;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlRightExtruderMove(uint8_t type, float destination/* = 0*/) {
  ErrCode ret;
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  int32_t scaled_dest;
  scaled_dest = (int32_t)(destination * 1000);

  uint8_t index = 0;
  buffer[index++] = (uint8_t)type;
  buffer[index++] = (scaled_dest >> 24) & 0xff;
  buffer[index++] = (scaled_dest >> 16) & 0xff;
  buffer[index++] = (scaled_dest >> 8) & 0xff;
  buffer[index++] = scaled_dest & 0xff;

  cmd.id     = MODULE_FUNC_MOVE_TO_DEST;
  cmd.data   = buffer;
  cmd.length = index;

  switch (type) {
    case 0:
    case 1:
      ret = canhost.SendStdCmdSync(cmd, 20000);
      // LOG_I("recv, move type: %d, move result: %d\n", recv_buf[0], recv_buf[1]);
      // 报错
      // todo
      break;
    case 2:
      ret = canhost.SendStdCmd(cmd, 0);
      break;
    default:
      ret = E_FAILURE;
      break;
  }

  if (ret != E_SUCCESS) {
    LOG_E("failed to move extruder, ret: %u\n", ret);
  }

  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlToolChange(uint8_t new_extruder) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;

  buffer[index++] = new_extruder;
  cmd.id          = MODULE_FUNC_SWITCH_EXTRUDER;
  cmd.data        = buffer;
  cmd.length      = index;
  return canhost.SendStdCmdSync(cmd, 5000);
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSaveHotendOffset(float offset, uint8_t axis) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;

  buffer[index++] = axis;
  buffer[index++] = ((uint8_t *)&offset)[0];
  buffer[index++] = ((uint8_t *)&offset)[1];
  buffer[index++] = ((uint8_t *)&offset)[2];
  buffer[index++] = ((uint8_t *)&offset)[3];
  cmd.id          = MODULE_FUNC_SET_HOTEND_OFFSET;
  cmd.data        = buffer;
  cmd.length      = index;
  return canhost.SendStdCmd(cmd, 0);
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSaveZCompensation(float *val) {
  ErrCode ret = E_SUCCESS;
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index;
  float scaled_compensation;

  for (uint32_t i = 0; i < EXTRUDERS; i++) {
    index = 0;
    buffer[index++]  = i;
    scaled_compensation = val[i];
    LOG_I("scaled_compensation: %f\n", scaled_compensation);
    buffer[index++]  = ((uint8_t *)&scaled_compensation)[0];
    buffer[index++]  = ((uint8_t *)&scaled_compensation)[1];
    buffer[index++]  = ((uint8_t *)&scaled_compensation)[2];
    buffer[index++]  = ((uint8_t *)&scaled_compensation)[3];
    cmd.id           = MODULE_FUNC_SET_PROBE_SENSOR_COMPENSATION;
    cmd.data         = buffer;
    cmd.length       = index;
    ret = canhost.SendStdCmd(cmd, 0);

    if (ret != E_SUCCESS) {
      LOG_E("failed to set z compensation, ret: %u\n", ret);
      return ret;
    }
  }

  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSetRightExtruderPosition(float raise_for_home_pos, float z_max_pos) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;

  LOG_I("set raise for home pos: %f, z_max_pos: %f\n", raise_for_home_pos, z_max_pos);
  uint32_t raise_for_home_pos_scaled = raise_for_home_pos * 1000;
  uint32_t z_max_pos_scaled = z_max_pos * 1000;
  index = 0;
  buffer[index++] = (raise_for_home_pos_scaled >> 24) & 0xff;
  buffer[index++] = (raise_for_home_pos_scaled >> 16) & 0xff;
  buffer[index++] = (raise_for_home_pos_scaled >> 8) & 0xff;
  buffer[index++] = raise_for_home_pos_scaled & 0xff;
  buffer[index++] = (z_max_pos_scaled >> 24) & 0xff;
  buffer[index++] = (z_max_pos_scaled >> 16) & 0xff;
  buffer[index++] = (z_max_pos_scaled >> 8) & 0xff;
  buffer[index++] = z_max_pos_scaled & 0xff;
  cmd.id          = MODULE_FUNC_SET_RIGHT_EXTRUDER_POS;
  cmd.data        = buffer;
  cmd.length      = index;
  return canhost.SendStdCmd(cmd, 0);
}


bool ToolHeadDualExtruder::probe_state() {
  return (bool)(probe_state_ & (1 << active_probe_sensor_));
}

bool ToolHeadDualExtruder::probe_state(probe_sensor_t sensor) {
  return (bool)(probe_state_ & (1<<((uint8_t)sensor)));
}

bool ToolHeadDualExtruder::filament_state() {
  return (bool)(filament_state_ & (1<<active_extruder));
}

bool ToolHeadDualExtruder::filament_state(uint8_t e) {
  return (bool)(filament_state_ & (1<<e));
}

ErrCode ToolHeadDualExtruder::ToolChange(uint8_t new_extruder, bool use_compensation/* = true */) {
  planner.synchronize();

  const bool leveling_was_active = planner.leveling_active;
  set_bed_leveling_enabled(false);

  if (new_extruder >= EXTRUDERS) {
    return E_PARAM;
  }

  if (!all_axes_homed()) {
    LOG_I("need go home before toolchange\n");
    return E_FAILURE;
  }

  if (new_extruder != active_extruder) {
    float hotend_offset_tmp[XYZ][EXTRUDERS] = {0};
    memset(hotend_offset_tmp, 0, sizeof(hotend_offset_tmp));
    memcpy(hotend_offset_tmp, hotend_offset, sizeof(hotend_offset));

    if (!use_compensation) {
      hotend_offset_tmp[Z_AXIS][1] = 0;
    }

    levelservice.UnapplyLiveZOffset(active_extruder);

    if (current_position[X_AXIS] < X_MIN_POS + hotend_offset_tmp[X_AXIS][1]) {
      do_blocking_move_to_xy(X_MIN_POS + hotend_offset_tmp[X_AXIS][1], current_position[Y_AXIS]);
    } else if (current_position[X_AXIS] > X_MAX_POS - hotend_offset_tmp[X_AXIS][1]) {
      do_blocking_move_to_xy(X_MAX_POS - hotend_offset_tmp[X_AXIS][1], current_position[Y_AXIS]);
    }

    if (current_position[Y_AXIS] < Y_MIN_POS + hotend_offset_tmp[Y_AXIS][1]) {
      do_blocking_move_to_xy(current_position[X_AXIS], Y_MIN_POS + hotend_offset_tmp[Y_AXIS][1]);
    } else if (current_position[Y_AXIS] > Y_MAX_POS - hotend_offset_tmp[Y_AXIS][1]) {
      do_blocking_move_to_xy(current_position[X_AXIS], Y_MAX_POS - hotend_offset_tmp[Y_AXIS][1]);
    }

    set_destination_from_current();
    current_position[Z_AXIS] += toolchange_settings.z_raise;
    NOMORE(current_position[Z_AXIS], soft_endstop[Z_AXIS].max);
    planner.buffer_line(current_position, feedrate_mm_s, active_extruder);
    planner.synchronize();

    if (new_extruder == 0) {
      ModuleCtrlToolChange(new_extruder);
    }

    float xdiff = hotend_offset_tmp[X_AXIS][new_extruder] - hotend_offset_tmp[X_AXIS][active_extruder];
    float ydiff = hotend_offset_tmp[Y_AXIS][new_extruder] - hotend_offset_tmp[Y_AXIS][active_extruder];
    float zdiff = hotend_offset_tmp[Z_AXIS][new_extruder] - hotend_offset_tmp[Z_AXIS][active_extruder];
    int32_t xdiff_scaled = xdiff * planner.settings.axis_steps_per_mm[X_AXIS];
    int32_t ydiff_scaled = ydiff * planner.settings.axis_steps_per_mm[Y_AXIS];
    int32_t zdiff_scaled = zdiff * planner.settings.axis_steps_per_mm[Z_AXIS];
    // Ensure that the divisor is not zero
    // todo
    xdiff = (float)xdiff_scaled / planner.settings.axis_steps_per_mm[X_AXIS];
    ydiff = (float)ydiff_scaled / planner.settings.axis_steps_per_mm[Y_AXIS];
    zdiff = (float)zdiff_scaled / planner.settings.axis_steps_per_mm[Z_AXIS];
    current_position[X_AXIS] += xdiff;
    current_position[Y_AXIS] += ydiff;
    current_position[Z_AXIS] += zdiff;
    sync_plan_position();

    apply_motion_limits(destination);
    do_blocking_move_to(destination);
    levelservice.ApplyLiveZOffset(active_extruder);
    planner.synchronize();
    active_extruder = new_extruder;

    if (new_extruder == 1) {
      ModuleCtrlToolChange(new_extruder);
    }
  }

  set_bed_leveling_enabled(leveling_was_active);
  return E_SUCCESS;
}


