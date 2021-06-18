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

#include "purifier.h"
#include "../common/debug.h"
#include "can_host.h"
#include "src/inc/MarlinConfig.h"

Purifier purifier;

static uint8_t * TAG = (uint8_t *)"Purifier";

void Purifier::DisplayErrInfo(uint8_t err) {
  if (!err) {
    return;
  }
  LOG_E("%s err code:%#X\n", TAG,err);
  if (TEST(err, ERR_ADDON_POWER))
    LOG_E("%s addon power err\n", TAG);
  if (TEST(err, ERR_EXTEND_POWER))
    LOG_E("%s extand power err\n", TAG);
  if (TEST(err, ERR_FAN_SPEED_TOO_LOW))
    LOG_E("%s fan speed too low\n", TAG);
  if (TEST(err, ERR_NO_FILTER))
    LOG_E("%s no filter\n", TAG);
  if (TEST(err, ERR_ELEC_TOO_HIGH))
    LOG_E("%s fan electricity too high\n", TAG);
}

void Purifier::DisplayInfo() {
  LOG_I("%s Filter lifetime:", TAG);
  switch (info_.lifetime) {
    case LIFETIME_NORMAL: LOG_I("normal\n"); break;
    case LIFETIME_MEDIUM: LOG_I("medium\n"); break;
    case LIFETIME_LOW: LOG_I("low\n"); break;
  }
  LOG_I("%s Fan work: %d\n", TAG, info_.is_work);
  LOG_I("%s Fan speed: %d rpm\n", TAG, info_.fan_speed);
  LOG_I("%s Fan out rate: %d%% \n", TAG, info_.fan_cur_out);
  LOG_I("%s Fan gears: %d\n", TAG, info_.fan_gears);
  LOG_I("%s Fan elec: %d mA\n", TAG, info_.fan_elec);
  LOG_I("%s Fan addon power: %d mv\n", TAG, info_.addon_power);
  LOG_I("%s Fan extend power: %d mv\n", TAG, info_.extend_power);
  DisplayErrInfo(info_.err);
}

void Purifier::DisplaySysStatus() {
  LOG_I("%s system status:%d\n", TAG, info_.sys_status_encode);
}

static void CallbackReportInfo(CanStdDataFrame_t &cmd) {
  purifier.UpdateInfo(cmd.data);
}


ErrCode Purifier::Init(MAC_t &mac, uint8_t mac_index) {
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

  LOG_I("\tGot Purifier Module!\n");

  function.channel   = mac.bits.channel;
  function.mac_index = mac_index;
  function.sub_index = 0;
  function.priority  = MODULE_FUNC_PRIORITY_DEFAULT;

  // register function ids to can host, it will assign message id
  for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    if (function.id == MODULE_FUNC_REPORT_PURIFIER)
      message_id[i] = canhost.RegisterFunction(function, CallbackReportInfo);
    else
      message_id[i] = canhost.RegisterFunction(function, NULL);
  }

  ret = canhost.BindMessageID(cmd, message_id);
  if (ret == E_SUCCESS) {
    online_ = PURIFIER_ONLINE;
    GetInfo(PURIFIER_INFO_ALL);
  }

  mac_index_ = mac_index;
  return ret;
}

void Purifier::SendCmd(uint16_t funcid, uint8_t *data, uint8_t len) {
  CanStdFuncCmd_t cmd;

  cmd.id     = funcid;
  cmd.data   = data;
  cmd.length = len;

  canhost.SendStdCmd(cmd, 0);
}

PurifierInfo_t Purifier::GetInfo(PURIFIER_INFO_E info, uint16_t timeout_ms) {
  uint32_t timeout = millis() + timeout_ms;
  volatile uint8_t flag = 0;
  update_info_flag_ = 0;
  uint8_t data = info;
  SendCmd(MODULE_FUNC_REPORT_PURIFIER, &data, 1);
  flag = (info == PURIFIER_INFO_ALL) ? ~(0xff<<PURIFIER_INFO_ALL) : (1 << info);
  while (timeout > millis()) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if ((update_info_flag_&flag) == flag) {
      break;
    }
  }
  if ((update_info_flag_&flag) != flag){
    LOG_E("purifier info update timeout:%x - %x!\n", update_info_flag_, flag);
  }
  return info_;
}

void Purifier::UpdateInfo(uint8_t data[8]) {
  switch (data[0]) {
    case PURIFIER_INFO_LIFETIME:
      UpdateLifetime(data[1]);
      break;
    case PURIFIER_INFO_ERR:
      UpdateErr(data[1]);
      break;
    case PURIFIER_INFO_FAN_STA:
      UpdateFanStatus(data[1], data[2] << 8 | data[3], data[4], data[5]);
      break;
    case PURIFIER_INFO_FAN_ELEC:
      UpdateElectricity(data[1] << 8 | data[2]);
      break;
    case PURIFIER_REPORT_POWER:
      UpdatePower(data[1] << 8 | data[2], data[3] << 8 | data[4]);
      break;
    case PURIFIER_REPORT_STATUS:
      info_.sys_status_encode = data[1];
      break;
    case PURIFIER_INFO_ALL:
      break;
  }
  update_info_flag_ |= 1 << data[0];
}

void Purifier::SetFanStatus(uint8_t is_open, uint16_t delay_close_s, uint8_t is_forced) {
  if (!is_open && delay_close_s) {
    delay_close_ = millis() + (delay_close_s * 1000);
    LOG_I("The purifier will be turned off in %d seconds\n", delay_close_s);
  } else {
    uint8_t data[3] = {PURIFIER_SET_FAN_STA, is_open, is_forced};
    SendCmd(MODULE_FUNC_SET_PURIFIER, data, 3);
    delay_close_ = 0;
    LOG_I("Set purifier fan open:%u, forced:%u\n", is_open, is_forced);
  }
  if (is_open) {
    ReportStatus();
  }
}

void Purifier::SetFanGears(uint8_t gears) {
  if (gears > PURIFIER_FAN_GEARS_3)
    gears = PURIFIER_FAN_GEARS_3;
  uint8_t data[2] = {PURIFIER_SET_FAN_GEARS, gears};
  SendCmd(MODULE_FUNC_SET_PURIFIER, data, 2);
  LOG_I("Set purifier Fan gears:%u\n", gears);
}

void Purifier::SetFanPower(uint8_t power) {
  uint8_t data[2] = {PURIFIER_SET_FAN_POWER, power};
  SendCmd(MODULE_FUNC_SET_PURIFIER, data, 2);
}

void Purifier::SetLightColor(uint8_t rgb[3]) {
  uint8_t data[4] = {PURIFIER_SET_LIGHT, rgb[0], rgb[1], rgb[2]};
  SendCmd(MODULE_FUNC_SET_PURIFIER, data, 4);
}

ErrCode Purifier::ReportStatus() {
  uint8_t buff[2];
  SSTP_Event_t event;
  if (IsOnline()) {
    GetInfo(PURIFIER_INFO_ERR, 200);
    if (!info_.err) {
      buff[0] = 0;  // online and normal
    } else if (TEST(info_.err, ERR_EXTEND_POWER)) {
      buff[0] = 2;  // online but power loss
    } else {
      buff[0] = 3;  // online but error
    }
  } else {
    buff[0] = 1;  // offline
  }
  event.id = EID_ADDON_ACK;
  event.op_code = ADDON_OPC_GET_PURIFIER_STATE;
  buff[1] = info_.err;
  event.length = 2;
  event.data = buff;
  LOG_I("SC req purifier sta:%d, err:0x%x\n", buff[0], buff[1]);

  return hmi.Send(event);
}

ErrCode Purifier::ReportLifetimeStatus() {
  uint8_t buff[1];
  SSTP_Event_t event;
  event.id = EID_ADDON_ACK;
  event.op_code = ADDON_OPC_GET_PURIFIER_TIMELIFE_STATE;
  buff[0] = info_.lifetime;
  event.length = 1;
  event.data = buff;
  LOG_I("SC req purifier lifetime sta:%d\n", buff[0]);

  return hmi.Send(event);
}

void Purifier::ErrCheckLoop() {
  if (last_err_ != info_.err) {
    last_err_ = info_.err;
    ReportStatus();
  }
  if (last_lifetime_ != info_.lifetime) {
    last_lifetime_ = info_.lifetime;
    ReportLifetimeStatus();
  }
}

void Purifier::Process() {
  if (delay_close_ && delay_close_ < millis()) {
    delay_close_ = 0;
    uint8_t data[3] = {PURIFIER_SET_FAN_STA, 0, 0};
    SendCmd(MODULE_FUNC_SET_PURIFIER, data, 3);
    LOG_I("The purifier be turned off\n");
  }
  ErrCheckLoop();
}
