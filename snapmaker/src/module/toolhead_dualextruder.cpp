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

#define USE_FDM_INTERRUPT_LOG

#define CAN_FRAME_SIZE                         8
#define DEFAULT_hotend_offsetX                 26
#define DEFAULT_hotend_offsetY                 0
#define DEFAULT_hotend_offsetZ                 -1.5
#define BIAS_hotend_offsetX                    1.2
#define BIAS_hotend_offsetY                    1.2
#define BIAS_hotend_offsetZ                    1.2

#define MIN_LEVELING_HEIGHT_3DP2E               (35)

ToolHeadDualExtruder printer_dualextruder(MODULE_DEVICE_ID_DUAL_EXTRUDER);

static void CallbackAckProbeState(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportProbeState(cmd.data);
}

static void CallbackAckNozzleTemp(CanStdDataFrame_t &cmd) {
  if (cmd.id.bits.length < 8)
    return;

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

static void CallbackAckReportHWVersion(CanStdDataFrame_t &cmd) {
  printer_dualextruder.ReportHWVersion(cmd.data);
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

    case MODULE_FUNC_GET_HW_VERSION:
      cb = CallbackAckReportHWVersion;
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

  E_ENABLE_ON = 1;
  IOInit();
  mac_index_ = mac_index;
  UpdateEAxisStepsPerUnit(MODULE_TOOLHEAD_DUALEXTRUDER);
  SetToolhead(MODULE_TOOLHEAD_DUALEXTRUDER);
  UpdateHotendMaxTemp(300, 0);
  UpdateHotendMaxTemp(300, 1);
  printer1 = this;

  // sync the state of sensors
  GetHWVersion();

  ModuleCtrlProbeStateSync();
  ModuleCtrlPidSync();
  ModuleCtrlFilamentStateSync();
  ModuleCtrlHotendOffsetSync();
  ModuleCtrlZProbeSensorCompensationSync();
  ModuleCtrlRightExtruderPosSync();

  CheckLevelingData();
  ModuleCtrlHotendTypeSync();

  LOG_I("dualextruder ready!\n");

out:
  return ret;
}

void ToolHeadDualExtruder::CheckLevelingData() {
  for (uint32_t i = 0; i < GRID_MAX_POINTS_X; i++) {
    for (uint32_t j = 0; j < GRID_MAX_POINTS_Y; j++) {
      if (z_values[i][j] < MIN_LEVELING_HEIGHT_3DP2E) {
        LOG_I("found a abnormal level data[%.3f] in point[%u][%u], will reset leveling data\n", z_values[i][j], i, j);
        reset_bed_level();
        bed_level_virt_interpolate();
        return;
      }
    }
  }
}

void ToolHeadDualExtruder::GetHWVersion() {
  CanStdFuncCmd_t cmd;
  uint8_t buff[2] {0xff, 0xff};
  ErrCode ret;

  cmd.id        = MODULE_FUNC_GET_HW_VERSION;
  cmd.data      = buff;
  cmd.length    = 2;

  ret = canhost.SendStdCmd(cmd);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get HW Version: ret[%u]\n", ret);
  }
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


#define ERR_OVERTEMP_BIT_MASK         (0)
#define ERR_INVALID_NOZZLE_BIT_MASK   (1)
void ToolHeadDualExtruder::ReportTemperature(uint8_t *data) {
  cur_temp_[0] = data[0] << 8 | data[1];
  if (data[2] & (1<<ERR_OVERTEMP_BIT_MASK)) {
    systemservice.ThrowException(EHOST_HOTEND0, ETYPE_OVERRUN_MAXTEMP);
  }
  cur_temp_[1] = data[4] << 8 | data[5];
  if (data[6] & (1<<ERR_OVERTEMP_BIT_MASK)) {
    systemservice.ThrowException(EHOST_HOTEND1, ETYPE_OVERRUN_MAXTEMP);
  }
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
  if (hotend_type_[0] == INVALID_HOTEND_TYPE || hotend_type_[1] == INVALID_HOTEND_TYPE) {
    for (uint32_t i = 0; i < EXTRUDERS; i++) {
      if (data[i] < HOTEND_INFO_MAX) {
        hotend_type_[i] = hotend_info[data[i]].model;
        hotend_diameter_[i] = hotend_info[data[i]].diameter;
      }
      else {
        hotend_type_[i] = hotend_info[HOTEND_INFO_MAX].model;
        hotend_diameter_[i] = hotend_info[HOTEND_INFO_MAX].diameter;
      }
    }
  }

  if (hotend_type_[0] == INVALID_HOTEND_TYPE || hotend_type_[1] == INVALID_HOTEND_TYPE) {
    systemservice.ThrowException(EHOST_EXECUTOR, ETYPE_3DP2E_UNKNOWN_NOZZLE);
  }
  else {
    systemservice.ClearException(EHOST_EXECUTOR, ETYPE_3DP2E_UNKNOWN_NOZZLE);
  }

  #ifdef USE_FDM_INTERRUPT_LOG
    LOG_I("nozzle_index: 0, type: %d, diameter: %.2f\n", hotend_type_[0], hotend_diameter_[0]);
    LOG_I("nozzle_index: 1, type: %d, diameter: %.2f\n", hotend_type_[1], hotend_diameter_[1]);
  #endif
}

void ToolHeadDualExtruder::ReportExtruderInfo(uint8_t *data) {
  uint8_t extruder_state = data[0];

  if (extruder_state) {
    systemservice.ThrowException(EHOST_EXECUTOR, ETYPE_3DP2E_EXTRUDER_MISMATCH);
  } else {
    systemservice.ClearException(EHOST_EXECUTOR, ETYPE_3DP2E_EXTRUDER_MISMATCH);
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
  z_compensation_[e] = (float)((data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]) / 1000;

  #ifdef USE_FDM_INTERRUPT_LOG
    LOG_I("origin extruder: %d, compensation: %f\n", e, z_compensation_[e]);
  #endif
  z_compensation_[e] -= 0.05;
}

void ToolHeadDualExtruder::ReportRightExtruderPos(uint8_t *data) {
  float raise_for_home_pos = (float)(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) / 1000;
  float z_max_pos = (float)(data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]) / 1000;

  LOG_I("raise_for_home_pos: %f, z_max_pos: %f\n", raise_for_home_pos, z_max_pos);
}

void ToolHeadDualExtruder::ReportHWVersion(uint8_t *data) {
  if ((data[0] ^ data[1]) != 0xff) {
    LOG_E("errors in hw ver:[0x%x, 0x%x]\n", data[0], data[1]);
    return;
  }
  hw_version_ = data[0];
  LOG_I("HW version of 3DP2E: 0x%x\n", hw_version_);
}

ErrCode ToolHeadDualExtruder::ModuleCtrlProximitySwitchPower(uint8_t state) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[CAN_FRAME_SIZE];
  ErrCode ret = E_SUCCESS;
  uint8_t index = 0;
  can_buffer[index++] = state;

  cmd.id        = MODULE_FUNC_PROXIMITY_SWITCH_POWER_CTRL;
  cmd.data      = can_buffer;
  cmd.length    = index;

  if ((ret = canhost.SendStdCmdSync(cmd, 2000)) != E_SUCCESS) {
    LOG_E("failed to set Proximity power!\n");
  }

  return ret;
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

  if (target_temp > 0 && (action_ban & ACTION_BAN_NO_HEATING_HOTEND)) {
    LOG_E("cannot heating hotend cause exception: 0x%x\n", action_ban);
    return E_EXCEPTION;
  }

  target_temp_[extrude_index] = target_temp;
  LOG_I("Set T%d=%d\n", extrude_index, target_temp_[extrude_index]);

  fan_e nozzle_fan_index = DUAL_EXTRUDER_NOZZLE_FAN;
  uint8_t fan_speed = 0;
  uint8_t fan_delay = 0;

  if (target_temp >= 50) {
    fan_speed = 255;
  }
  else {
    // check if need to delay to turn off fan
    if (cur_temp_[0] >= 150 || cur_temp_[1] >= 150) {
      fan_speed = 0;
      fan_delay = 120;
    } else if (cur_temp_[0] >= 50 || cur_temp_[1] >= 50) {
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
    buffer[2*i + 0] = (uint8_t)(target_temp_[i]>>8);
    buffer[2*i + 1] = (uint8_t)target_temp_[i];
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
  vTaskDelay(pdMS_TO_TICKS(100));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlPidSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_3DP_PID;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlHotendTypeSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_NOZZLE_TYPE;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlFilamentStateSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_RUNOUT_SENSOR_STATE;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlHotendOffsetSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_HOTEND_OFFSET;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlZProbeSensorCompensationSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlRightExtruderPosSync() {
  CanStdFuncCmd_t cmd;

  cmd.id      = MODULE_FUNC_REPORT_RIGHT_EXTRUDER_POS;
  cmd.data    = NULL;
  cmd.length  = 0;
  ErrCode ret = canhost.SendStdCmd(cmd, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSetPid(float p, float i, float d) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;

  int32_t tmp_i32;

  pid_[0] = p;
  pid_[1] = i;
  pid_[2] = d;

  for (int32_t i = 0; i < 3; i++) {
    index = 0;
    buffer[index++] = i;
    tmp_i32 = (int32_t)(pid_[i] * 1000);
    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, index);

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

ErrCode ToolHeadDualExtruder::ModuleCtrlRightExtruderMove(move_type_e type, float destination/* = 0*/) {
  ErrCode ret;
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  int32_t scaled_dest;

  uint8_t index = 0;
  buffer[index++] = (uint8_t)type;

  cmd.id     = MODULE_FUNC_MOVE_TO_DEST;
  cmd.data   = buffer;
  cmd.length = index;

  switch (type) {
    case MOVE_SYNC:
      scaled_dest = (int32_t)(destination * 1000);
      WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, scaled_dest, index);
    case GO_HOME:
      ret = canhost.SendStdCmdSync(cmd, 20000);
      break;
    case MOVE_ASYNC:
      ret = canhost.SendStdCmd(cmd, 0);
      break;
    default:
      ret = E_FAILURE;
      break;
  }

  if (ret != E_SUCCESS) {
    LOG_E("failed to move extruder, ret: %u\n", ret);
  }
  else {
    if (type == GO_HOME) {
      // enable extruder checking
      ModuleCtrlSetExtruderChecking(true);
    }
  }

  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlToolChange(uint8_t new_extruder) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;
  ErrCode ret = E_SUCCESS;

  buffer[index++] = new_extruder;
  cmd.id          = MODULE_FUNC_SWITCH_EXTRUDER;
  cmd.data        = buffer;
  cmd.length      = index;
  if ((ret = canhost.SendStdCmdSync(cmd, 5000)) != E_SUCCESS) {
    LOG_E("failed to switch extruder!\n");
  }

  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSetExtruderChecking(bool on_off) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  ErrCode ret = E_SUCCESS;

  #define EXTRUDER_CHECKING_IDLE    (1)
  #define EXTRUDER_CHECKING_ENABLE  (0)

  buffer[0]     = on_off? EXTRUDER_CHECKING_ENABLE : EXTRUDER_CHECKING_IDLE;
  cmd.id          = MODULE_SET_EXTRUDER_CHECK;
  cmd.data        = buffer;
  cmd.length      = 1;
  if ((ret = canhost.SendStdCmd(cmd)) != E_SUCCESS) {
    LOG_E("failed to set extruder checking!\n");
  }

  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSaveHotendOffset(float offset, uint8_t axis) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;

  buffer[index++] = axis;
  *(float *)(buffer + 1) = offset;

  cmd.id      = MODULE_FUNC_SET_HOTEND_OFFSET;
  cmd.data    = buffer;
  cmd.length  = index + 4;
  return canhost.SendStdCmd(cmd, 0);
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSaveZCompensation(float comp, uint32_t e) {
  ErrCode ret = E_SUCCESS;
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;

  index = 0;
  buffer[index++] = e;

  // little ending
  *(float *)(buffer + 1) = comp;

  LOG_I("save compensation[%u]: %f\n", comp, e);

  cmd.id      = MODULE_FUNC_SET_PROBE_SENSOR_COMPENSATION;
  cmd.data    = buffer;
  cmd.length  = index + 4;

  ret = canhost.SendStdCmd(cmd, 0);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set z compensation, ret: %u\n", ret);
  }

  return ret;
}

ErrCode ToolHeadDualExtruder::ModuleCtrlSetRightExtruderPosition(float raise_for_home_pos, float z_max_pos) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[CAN_FRAME_SIZE];
  uint8_t index = 0;
  int32_t tmp_i32;

  LOG_I("set raise for home pos: %f, z_max_pos: %f\n", raise_for_home_pos, z_max_pos);

  tmp_i32 = raise_for_home_pos * 1000;
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, index);

  tmp_i32 = z_max_pos * 1000;
  WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp_i32, index);

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

void ToolHeadDualExtruder::SelectProbeSensor(probe_sensor_t sensor) {
  if (sensor >= PROBE_SENSOR_INVALID) {
    return;
  }

  active_probe_sensor_ = sensor;
}

void ToolHeadDualExtruder::SetZCompensation(float comp, uint32_t e) {
  z_compensation_[e] = comp;
  ModuleCtrlSaveZCompensation(z_compensation_[e], e);
}

void ToolHeadDualExtruder::GetZCompensation(float &left_z_compensation, float &right_z_compensation) {
  left_z_compensation = z_compensation_[0];
  right_z_compensation = z_compensation_[1];
}

ErrCode ToolHeadDualExtruder::ToolChange(uint8_t new_extruder, bool use_compensation/* = true */) {
  volatile int32_t xdiff_scaled, ydiff_scaled, zdiff_scaled;
  volatile float xdiff, ydiff, zdiff;
  volatile float hotend_offset_tmp[XYZ][HOTENDS] {0};
  volatile float z_raise = 0;

  float pre_position[X_TO_E];

  volatile uint32_t old_extruder;

  planner.synchronize();

  const bool leveling_was_active = planner.leveling_active;

  if (new_extruder >= EXTRUDERS) {
    return E_PARAM;
  }

  if (!all_axes_homed()) {
    LOG_I("need go home before toolchange\n");
    return E_FAILURE;
  }

  if (new_extruder != active_extruder) {
    LOOP_XYZ(i) {
      HOTEND_LOOP() {
        hotend_offset_tmp[i][e] = hotend_offset[i][e];
      }
    }

    if (!use_compensation) {
      hotend_offset_tmp[Z_AXIS][1] = 0;
    }

    planner.synchronize();
    taskENTER_CRITICAL();
    LOOP_X_TO_EN(i) backup_current_position[i] = current_position[i];
    backup_position_valid = true;
    taskEXIT_CRITICAL();

    set_bed_leveling_enabled(false);

    LOG_I("\norigin pos: %.3f, %.3f, %.3f\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

    z_raise = current_position[Z_AXIS] + toolchange_settings.z_raise;

    NOMORE(z_raise, soft_endstop[Z_AXIS].max);

    z_raise = z_raise - current_position[Z_AXIS];

    LOG_I("raise: %.3f, endstop max: %.3f, z offset: %.3f\n", z_raise, soft_endstop[Z_AXIS].max, hotend_offset_tmp[Z_AXIS][1]);

    do_blocking_move_to_z(current_position[Z_AXIS] + z_raise, 30);

    LOG_I("raised pos: %.3f, %.3f, %.3f\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

    // remove live z offset of old extruder after raise Z, cause Z will fall in unapplying live offset
    levelservice.UnapplyLiveZOffset(active_extruder);
    // to avoid power-loss, we record the new extruder  after unapply z offset!
    old_extruder = active_extruder;
    active_extruder = new_extruder;
    actual_extruder = new_extruder;

    //set_destination_from_current();
    COPY(pre_position, current_position);

    if (new_extruder > old_extruder) {
      // left -> right, toolhead will move to left, make sure there is enough space in left for the moving
      if (current_position[X_AXIS] < X_MIN_POS + hotend_offset_tmp[X_AXIS][1]) {
        do_blocking_move_to_xy(X_MIN_POS + hotend_offset_tmp[X_AXIS][1], current_position[Y_AXIS], 50);
      }
    }
    else {
      // right -> left, toolhead will move to right, make sure there is enough space in right for the moving
      if (current_position[X_AXIS] > X_MAX_POS - hotend_offset_tmp[X_AXIS][1]) {
        do_blocking_move_to_xy(X_MAX_POS - hotend_offset_tmp[X_AXIS][1], current_position[Y_AXIS], 50);
      }
    }

    update_software_endstops(X_AXIS, old_extruder, new_extruder);
    update_software_endstops(Y_AXIS, old_extruder, new_extruder);
    update_software_endstops(Z_AXIS, old_extruder, new_extruder);

    if (new_extruder == 0) {
      ModuleCtrlToolChange(new_extruder);
    }

    xdiff = hotend_offset_tmp[X_AXIS][new_extruder] - hotend_offset_tmp[X_AXIS][old_extruder];
    ydiff = hotend_offset_tmp[Y_AXIS][new_extruder] - hotend_offset_tmp[Y_AXIS][old_extruder];
    zdiff = hotend_offset_tmp[Z_AXIS][new_extruder] - hotend_offset_tmp[Z_AXIS][old_extruder];
    xdiff_scaled = xdiff * planner.settings.axis_steps_per_mm[X_AXIS];
    ydiff_scaled = ydiff * planner.settings.axis_steps_per_mm[Y_AXIS];
    zdiff_scaled = zdiff * planner.settings.axis_steps_per_mm[Z_AXIS];
    // Ensure that the divisor is not zero
    // todo
    xdiff = (float)xdiff_scaled / planner.settings.axis_steps_per_mm[X_AXIS];
    ydiff = (float)ydiff_scaled / planner.settings.axis_steps_per_mm[Y_AXIS];
    zdiff = (float)zdiff_scaled / planner.settings.axis_steps_per_mm[Z_AXIS];
    current_position[X_AXIS] += xdiff;
    current_position[Y_AXIS] += ydiff;
    current_position[Z_AXIS] += zdiff;
    LOG_I("offset pos: %.3f, %.3f, %.3f\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
    sync_plan_position();

    apply_motion_limits(pre_position);
    do_blocking_move_to(pre_position);

    if (new_extruder == 1) {
      ModuleCtrlToolChange(new_extruder);
    }

    // here we should apply live z offset of new extruder!
    levelservice.ApplyLiveZOffset(active_extruder);

    do_blocking_move_to_z(current_position[Z_AXIS] - z_raise, 30);

    LOG_I("descent pos: %.3f, %.3f, %.3f\n\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

    // after swtich extruder, just select relative OPTOCOUPLER
    SelectProbeSensor((probe_sensor_t)(PROBE_SENSOR_LEFT_OPTOCOUPLER + new_extruder));
  }

  set_bed_leveling_enabled(leveling_was_active);
  taskENTER_CRITICAL();
  backup_position_valid = false;
  taskEXIT_CRITICAL();

  return E_SUCCESS;
}

// hmi interface
ErrCode ToolHeadDualExtruder::HmiGetHotendType(SSTP_Event_t &event) {
  uint8_t buff[10];
  int32_t tmp_i32;

  int i = 0;

  buff[i++] = hotend_type_[0];
  tmp_i32 = (int32_t)(hotend_diameter_[0] * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);
  buff[i++] = hotend_type_[1];
  tmp_i32 = (int32_t)(hotend_diameter_[1] * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);


  event.data   = buff;
  event.length = (uint16_t)i;
  return hmi.Send(event);
}

ErrCode ToolHeadDualExtruder::HmiGetFilamentState(SSTP_Event_t &event) {
  uint8_t buf[2];
  uint8_t index = 0;

  buf[index++] = (uint8_t)(!filament_state(0));
  buf[index++] = (uint8_t)(!filament_state(1));

  event.data   = buf;
  event.length = index;
  return hmi.Send(event);
}

ErrCode ToolHeadDualExtruder::HmiGetHotendTemp(SSTP_Event_t &event) {
  uint8_t buff[8];
  int16_t tmp_i16;

  int i = 0;

  tmp_i16 = (int16_t)thermalManager.degHotend(0);
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);
  tmp_i16 = (int16_t)thermalManager.degTargetHotend(0);
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);

  tmp_i16 = (int16_t)thermalManager.degHotend(1);
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);
  tmp_i16 = (int16_t)thermalManager.degTargetHotend(1);
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, tmp_i16, i);

  event.data   = buff;
  event.length = (uint16_t)i;
  return hmi.Send(event);
}

ErrCode ToolHeadDualExtruder::HmiRequestToolChange(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  uint8_t buf[2];

  switch (event.data[0]) {
    case 0:
      err = ToolChange(0);
      break;
    case 1:
      err = ToolChange(1);
      break;
    default:
      err = E_PARAM;
      break;
  }

  if (err == E_SUCCESS) {
    buf[0] = 0;
  } else {
    buf[0] = 1;
  }

  buf[1] = active_extruder;

  event.data    = buf;
  event.length  = 2;
  return hmi.Send(event);
}

ErrCode ToolHeadDualExtruder::HmiSetFanSpeed(SSTP_Event_t &event) {
  if (event.length != 2 || event.data[0] > 2) {
    return E_PARAM;
  }

  SetFan(event.data[0], event.data[1]);

  event.data = fan_speed_;
  event.length = 3;
  return hmi.Send(event);
}

ErrCode ToolHeadDualExtruder::HmiSetHotendOffset(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  uint8_t axis;

  float tmp_offset;
  float nozzle_offset[XYZ][HOTENDS] = DEFAULT_HOTEND_OFFSETS;


  if (event.length != 5 || event.data[0] >= 3) {
    LOG_E("HmiSetHotendOffset: parameter error!\n");
    err = E_PARAM;
    goto EXIT;
  }

  axis = event.data[0];

  PDU_TO_LOCAL_WORD(tmp_offset, event.data + 1);
  tmp_offset /= 1000;

  LOG_E("HmiSetHotendOffset: %.3f, axis: %u!\n", tmp_offset, axis);

  if (tmp_offset > HOTEND_OFFSET_MAX_DEVIATION || tmp_offset < -HOTEND_OFFSET_MAX_DEVIATION) {
    err = E_PARAM;
    LOG_E("invalid hotend offset!\n");
  }
  else {
    hotend_offset[axis][1] = nozzle_offset[axis][1] + tmp_offset;
    ModuleCtrlSaveHotendOffset(hotend_offset[axis][1], axis);
  }

EXIT:
  event.data    = &err;
  event.length  = 1;
  return hmi.Send(event);
}

ErrCode ToolHeadDualExtruder::HmiGetHotendOffset(SSTP_Event_t &event) {
  uint8_t buff[12];


  int32_t   tmp_i32;
  int i = 0;

  tmp_i32 = (int32_t)(hotend_offset[X_AXIS][1] * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);

  tmp_i32 = (int32_t)(hotend_offset[Y_AXIS][1] * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);

  tmp_i32 = (int32_t)(hotend_offset[Z_AXIS][1] * 1000);
  WORD_TO_PDU_BYTES_INDEX_MOVE(buff, tmp_i32, i);

  event.data    = buff;
  event.length  = (uint16_t)i;
  return hmi.Send(event);
}

ErrCode ToolHeadDualExtruder::HmiRequestGetActiveExtruder(SSTP_Event_t &event) {
  event.data    = &active_extruder;
  event.length  = 1;
  return hmi.Send(event);
}

void ToolHeadDualExtruder::ShowInfo() {
  ModuleCtrlHotendTypeSync();

  LOG_I("active_probe_sensor: %u\n", active_probe_sensor_);
  LOG_I("hotend_type: 0: %u, 1: %u\n", hotend_type_[0], hotend_type_[1]);
  LOG_I("hotend_diameter: 0: %.2f, 1:%.2f\n", hotend_diameter_[0], hotend_diameter_[1]);
  LOG_I("Kp: %.3f, Ki: %.3f, Kd: %.3f\n", pid_[0], pid_[1], pid_[2]);
  LOG_I("z_compensation: 0: %.3f, 1: %.3f\n", z_compensation_[0], z_compensation_[1]);
}

void ToolHeadDualExtruder::Process() {
  if (mac_index_ == MODULE_MAC_INDEX_INVALID)
    return;

  if (++timer_in_process_ < 100) return;
  timer_in_process_ = 0;

  if (hw_version_ == MODULE_HW_VER_INVALID) {
    GetHWVersion();
  }
}

bool ToolHeadDualExtruder::GetToolChangePrePosition(float *position, uint8_t size) {
  bool ret = false;
  if (position && size >= X_TO_E) {
    LOOP_X_TO_EN(i) position[i] = backup_current_position[i];
    ret = backup_position_valid;
  }
  return ret;
}
