#include "upgrade_service.h"
#include "snap_dbg.h"
#include <EEPROM.h>

#include "../Marlin.h"
#include "../module/CanModule.h"
#include "../module/CanBus.h"
#include "../module/StatusControl.h"
#include "../../HAL/HAL_GD32F1/HAL_watchdog_STM32F1.h"


UpgradeService upgrade;


ErrCode UpgradeService::RequestNextPacket() {
  Event_t event = {EID_UPGRADE_ACK, UPGRADE_OPC_TRANS_FW};

  uint8_t buff[2];

  event.length = 0;
  HWORD_TO_PDU_BYTES_INDE_MOVE(buff, req_pkt_counter_, event.length);

  event.data = buff;
  event.length = 2;

  return hmi.Send(event);
}

ErrCode UpgradeService::StartUpgrade(Event_t &event) {
  ErrCode err = E_SUCCESS;
  uint32_t addr;
  uint8_t pages;

  event.data = &err;
  event.length = 1;

  SysStatus sta = SystemStatus.GetCurrentStatus();

  if (sta != SYSTAT_IDLE) {
    LOG_E("cannot upgrade in current status: %u\n", sta);
    err = E_FAILURE;
    return hmi.Send(event);
  }

  taskENTER_CRITICAL();
  FLASH_Unlock();

  // erase update info
  pages = UPDATE_CONTENT_INFO_SIZE / 2048;
  addr = FLASH_UPDATE_CONTENT_INFO;
  for (int i = 0; i < pages; i++) {
    FLASH_ErasePage(addr);
    addr += 2048;
  }

  // erase flash for new fw
  pages = MARLIN_CODE_SIZE / 2048;
  addr = FLASH_UPDATE_CONTENT;
  for (int i = 0; i < pages; i++) {
    FLASH_ErasePage(addr);
    addr += 2048;
  }
  FLASH_Lock();
  taskEXIT_CRITICAL();

  received_fw_size_ = 0;
  upgrade_status_ = UPGRADE_STA_IS_UPGRADING;
  req_pkt_counter_ = 0;

  hmi.Send(event);

  return RequestNextPacket();
}


ErrCode UpgradeService::ReceiveFW(Event_t &event) {

  uint32_t addr;
  uint16_t packet_index;
  uint16_t tmp;

  uint16_t data_len;

  packet_index = (uint16_t)(event.data[0]<<8 | event.data[1]);

  if ((packet_index < max_packet_) && (upgrade_status_ == UPGRADE_STA_IS_UPGRADING) && (packet_index == req_pkt_counter_)) {

    data_len = (event.length - 2);

    // every packet should have 512 bytes besides the last packet
    received_fw_size_ = packet_index<<7 + data_len;

    addr = FLASH_UPDATE_CONTENT + packet_index<<7;

    // if we have no enough
    if ((data_len % 2) != 0) data_len++;

    taskENTER_CRITICAL();
    FLASH_Unlock();

    for (int i = 2; i < (data_len + 2); i = i + 2) {
      tmp = ((event.data[i + 1]<<8) | event.data[i]);

      FLASH_ProgramHalfWord(addr, tmp);

      addr = addr + 2;
    }

    FLASH_Lock();
    taskEXIT_CRITICAL();

    req_pkt_counter_++;
  }

  return RequestNextPacket();
}


ErrCode UpgradeService::EndUpgarde(Event_t &event) {
  ErrCode err = E_FAILURE;

  event.data = &err;
  event.length = 1;

  if (upgrade_status_ != UPGRADE_STA_IS_UPGRADING) {
    LOG_E("Not in upgrading!\n");
    return hmi.Send(event);
  }

  upgrade_status_ = UPGRADE_STA_IDLE;

  taskENTER_CRITICAL();
  FLASH_Unlock();

  FLASH_ProgramWord((uint32_t)(FLASH_UPDATE_CONTENT_INFO), 0xaa55ee11);

  FLASH_Lock();
  taskEXIT_CRITICAL();

  err = E_SUCCESS;

  WatchDogInit();

  return hmi.Send(event);
}


ErrCode UpgradeService::GetMainControllerVer(Event_t &event) {
  uint32_t addr;
  uint16_t i;

  uint8_t ver[33];

  //Request controller's firmware version
  addr = FLASH_BOOT_PARA + 2048;

  for (i = 0; i < 32; i++)
    ver[i] = *((uint8_t*)addr++);

  ver[i] = 0;

  event.data = &ver[10];
  event.length = 23;

  return hmi.Send(event);
}


ErrCode UpgradeService::CompareMCVer(Event_t &event) {
  uint32_t  addr;
  int       i;
  ErrCode   err = E_SUCCESS;
  uint8_t   cur_ver[33];

  addr = FLASH_BOOT_PARA + 2048;

  for (i = 0; i < 32; i++)
    cur_ver[i] = *((uint8_t*)addr++);

  cur_ver[32] = 0;

  for (i = 0; i < 32; i++) {
    if(cur_ver[i] != event.data[i]) {
      err = E_FAILURE;
      break;
    }

    if (cur_ver[i] == 0)
      break;
  }

  return hmi.Send(event);
}


ErrCode UpgradeService::GetUpgradeStatus(Event_t &event) {
  uint8_t up_status = (uint8_t) upgrade_status_;

  event.data = &up_status;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode UpgradeService::GetModuleVer(Event_t &event) {

  LOG_I("SC req MODULE ver\n");
  uint8_t buffer[VERSION_STRING_SIZE + 4];
  uint32_t mac;
  int l;

  event.data = buffer;

  for (int i = 0; i < CanBusControlor.ModuleCount; i++) {

    mac = CanBusControlor.ModuleMacList[i];

    l = 0;
    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, mac, l);

    if (CanModules.GetFirmwareVersion(BASIC_CAN_NUM, mac, (char *)(buffer + 4)) == true) {
      for (; l < (VERSION_STRING_SIZE + 4); l++) {
        if (buffer[l] == 0)
          break;
      }

      event.length = l;

      hmi.Send(event);
    }
  }

  return E_SUCCESS;
}


ErrCode UpgradeService::SendModuleUpgradeStatus(uint8_t sta) {
  Event_t event = {EID_UPGRADE_ACK, UPGRADE_OPC_SYNC_MODULE_UP_STATUS};

  event.data = &sta;
  event.length = 1;

  return hmi.Send(event);
}

