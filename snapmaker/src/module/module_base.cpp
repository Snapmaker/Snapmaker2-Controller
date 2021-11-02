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
#include "module_base.h"
#include "can_host.h"

#include "../service/upgrade.h"
#include "../common/protocol_sstp.h"
#include "../common/debug.h"
#include "../hmi/event_handler.h"

#include "linear.h"
#include "enclosure.h"
#include "emergency_stop.h"
#include "rotary_module.h"
#include "toolhead_3dp.h"
#include "toolhead_cnc.h"
#include "toolhead_laser.h"
#include "purifier.h"

// marlin headers
#include "src/Marlin.h"
#include "src/inc/MarlinConfig.h"
#include "src/feature/bedlevel/abl/abl.h"
#include "src/module/configuration_store.h"
#include HAL_PATH(src/HAL, HAL.h)

extern ToolHead3DP printer_single;

ModuleBase *static_modules[] = {
  &linear,
  &printer_single,
  &laser_1_6_w,
  &cnc,
  &enclosure,
  &emergency_stop,
  &linear_tmc,
  &rotaryModule,
  &purifier,
  &laser_10w,
  NULL
};

bool ModuleBase::lock_marlin_uart_ = false;
LockMarlinUartSource ModuleBase::lock_marlin_source_ = LOCK_SOURCE_NONE;
uint16_t ModuleBase::timer_in_static_process_ = 0;
ModuleToolHeadType ModuleBase::toolhead_ = MODULE_TOOLHEAD_UNKNOW;


ErrCode ModuleBase::Upgrade(MAC_t &mac, uint32_t fw_addr, uint32_t fw_length) {
  ErrCode     ret;
  CanExtCmd_t cmd;

  uint32_t tmp_u32;
  int      i;

  uint16_t total_packet;
  uint16_t packet_length;
  uint16_t packet_index;

  cmd.mac  = mac;
  cmd.data = (uint8_t *)pvPortMalloc(528);
  if (!cmd.data) {
    LOG_I("Failed to apply mem\n");
    return E_NO_MEM;
  }

  LOG_I("\nStart upgrading: 0x%08X\n", mac.val);

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_START_UPGRADE_REQ;

  // read flag in fw
  tmp_u32 = *((uint32_t *)(fw_addr + UPGRADE_FW_OFFSET_FLAG));
  cmd.data[MODULE_EXT_CMD_INDEX_DATA] = (uint8_t)tmp_u32;

  // read version
  for (i = 0; i < VERSION_STRING_SIZE; i++) {
    cmd.data[2+i] = *((uint8_t *)(fw_addr + UPGRADE_FW_OFFSET_VERSION + i));
    if (cmd.data[2+i] == 0)
      break;
  }
  cmd.length = 2 + i;

  // 1. send upgrade request to module
  ret = canhost.SendExtCmdSync(cmd, 1000, 2);
  if (ret != E_SUCCESS) {
    LOG_I("Failed to req upgrade\n");
    goto out;
  }

  // module reject to be upgraded
  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] != 1) {
    ret = E_SAME_STATE;
    LOG_I("Reject to be upgraded\n");
    goto out;
  }

  // 2. waiting for module become ready to receive fw
  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_UPGRADE_STATUS_REQ;
  cmd.length = 1;
  ret = canhost.SendExtCmdSync(cmd, 500, 10);
  if (ret != E_SUCCESS) {
    LOG_I("Failed to get up status\n");
    goto out;
  }
  // timeout to be ready
  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] != 1) {
    LOG_I("Timeout to be ready\n");
    ret = E_INVALID_STATE;
    goto out;
  }

  // 3. broadcast we are going to start upgrading one module
  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_INFORM_UPGRADE_START;
  cmd.length = 1;
  canhost.SendExtCmd(cmd);

  total_packet = fw_length / UPGRADE_FW_OFFSET_FW_SIZE;
  if (fw_length % UPGRADE_FW_OFFSET_FW_SIZE)
    total_packet++;

  for (;;) {
    // 4. wait packet request from module
    cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_TRANS_FW_ACK;
    cmd.length = 528;
    ret = canhost.WaitExtCmdAck(cmd, 500, 10);
    if (ret != E_SUCCESS) {
      LOG_I("Time out to get pack\n");
      goto out;
    }

    // packet index from module
    packet_index = cmd.data[2]<<8 | cmd.data[3];
    if (packet_index  >= total_packet) {
      break;
    }

    // length of this packet
    if (fw_length - packet_index * MODULE_UPGRADE_PACKET_SIZE < MODULE_UPGRADE_PACKET_SIZE)
      packet_length = tmp_u32 % MODULE_UPGRADE_PACKET_SIZE;
    else
      packet_length = MODULE_UPGRADE_PACKET_SIZE;

    // start address of this packet
    tmp_u32 = fw_addr + 2048 + packet_index * MODULE_UPGRADE_PACKET_SIZE;

    for (i = 0; i < packet_length; i++) {
      cmd.data[2 + i] = *((uint8_t*)(tmp_u32 + i));
    }

    // 5. send packet to module
    cmd.data[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_TRANS_FW_REQ;
    cmd.data[MODULE_EXT_CMD_INDEX_DATA] = 0;
    cmd.length = 2 + packet_length;
    canhost.SendExtCmd(cmd);
  }

  LOG_I("Done\n");
  // 6. request to end upgrading
  cmd.data[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_END_UPGRADE_REQ;
  cmd.data[MODULE_EXT_CMD_INDEX_DATA] = 0;
  cmd.length = 2;
  canhost.SendExtCmd(cmd);

out:
  vPortFree(cmd.data);

  return ret;
}


ErrCode ModuleBase::InitModule8p(MAC_t &mac, int dir_pin, uint8_t index) {
  CanExtCmd_t cmd;
  uint8_t     buffer[16];

  cmd.data   = buffer;
  cmd.length = 2;
  cmd.mac    = mac;

  cmd.data[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_CONFIG_REQ;
  cmd.data[MODULE_EXT_CMD_INDEX_DATA] = index;

  WRITE(dir_pin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));

  // didn't get ack from module
  if (canhost.SendExtCmdSync(cmd, 500) != E_SUCCESS)
    return E_HARDWARE;

  // module didn;t detect HIGH in dir pin
  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] != 1)
    return E_INVALID_STATE;

  WRITE(dir_pin, LOW);

  return E_SUCCESS;
}


void ModuleBase::LockMarlinUart(LockMarlinUartSource source) {
  lock_marlin_uart_ = true;
  lock_marlin_source_ = max(lock_marlin_source_, source);
}


void ModuleBase::UnlockMarlinUart() {
  lock_marlin_uart_ = false;
  lock_marlin_source_ = LOCK_SOURCE_NONE;
}


void ModuleBase::ReportMarlinUart() {
  if (!lock_marlin_uart_)
    return;
  switch (lock_marlin_source_) {
    case LOCK_SOURCE_NONE:
    case LOCK_SOURCE_ENCLOSURE:
      SERIAL_ECHOLN(";Locked UART");
      break;
    case LOCK_SOURCE_EMERGENCY_STOP:
      break;
  }
}


ErrCode ModuleBase::SetMAC(SSTP_Event_t &event) {
  CanExtCmd_t cmd;
  uint8_t     buffer[8];

  int      i;
  uint32_t old_mac;

  PDU_TO_LOCAL_WORD(old_mac, event.data);

  cmd.data    = buffer;
  cmd.data[0] = MODULE_EXT_CMD_SSID_REQ;
  cmd.data[1] = 1;
  cmd.data[2] = event.data[4];
  cmd.data[3] = event.data[5];
  cmd.data[4] = event.data[6];
  cmd.data[5] = event.data[7];

  // error code to HMI
  event.data[0] = E_FAILURE;
  event.length = 1;

  old_mac &= MODULE_MAC_ID_MASK;
  for (i = 0; i < MODULE_SUPPORT_CONNECTED_MAX; i++) {
    if (old_mac == (canhost.mac(i) & MODULE_MAC_ID_MASK))
      goto out;
  }

  goto error;

out:
  cmd.mac.val = canhost.mac(i);
  event.data[0] = canhost.SendExtCmdSync(cmd, 500);

error:
  return hmi.Send(event);
}


ErrCode ModuleBase::GetMAC(SSTP_Event_t &event) {
  int i, j = 0;
  uint32_t tmp;
  uint8_t buffer[4 * MODULE_SUPPORT_CONNECTED_MAX];

  for(i = 0; i < MODULE_SUPPORT_CONNECTED_MAX; i++) {
    if ((tmp = canhost.mac(i)) == MODULE_MAC_ID_INVALID)
      break;

    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp, j);
  }

  event.data = buffer;
  event.length = (uint16_t)j;

  return hmi.Send(event);
}


extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;
void ModuleBase::SetToolhead(ModuleToolHeadType toolhead) {
  bool need_saved = false;

  // if plugged non-3DP toolhead, will reset leveling data
  if (toolhead != MODULE_TOOLHEAD_3DP) {
    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
        if (z_values[x][y] != DEFAUT_LEVELING_HEIGHT) {
          z_values[x][y] = DEFAUT_LEVELING_HEIGHT;
          need_saved = true;
        }
      }
  }

  toolhead_ = toolhead;
  set_min_planner_speed();
  if (need_saved)
    settings.save();
}
