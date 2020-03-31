#include "upgrade_handler.h"
#include "snap_dbg.h"

#include "../Marlin.h"
#include "../module/CanModule.h"
#include "../module/CanBus.h"
#include "../../HAL/HAL_GD32F1/HAL_watchdog_STM32F1.h"


UpgradeHandler upgrade;


ErrCode UpgradeHandler::RequestNextPacket() {
  Event_t event = {EID_UPGRADE_ACK, UPGRADE_OPC_TRANS_FW};

  uint8_t buff[2];

  hmi.ToPDUBytes(buff, (uint8_t *)&req_pkt_counter_, 2);

  event.data = buff;
  event.length = 2;

  return hmi.Send(event);
}

ErrCode UpgradeHandler::StartUpgrade(Event_t &event) {
  ErrCode err = E_SUCCESS;
  uint32_t addr;
  uint8_t pages;

  //擦除FLASH
  FLASH_Unlock();

  //擦除升级文件信息
  pages = UPDATE_CONTENT_INFO_SIZE / 2048;
  addr = FLASH_UPDATE_CONTENT_INFO;
  for (int i = 0; i < pages; i++) {
    FLASH_ErasePage(addr);
    addr += 2048;
  }

  //擦除升级内容
  pages = MARLIN_CODE_SIZE / 2048;
  addr = FLASH_UPDATE_CONTENT;
  for (int i = 0; i < pages; i++) {
    FLASH_ErasePage(addr);
    addr += 2048;
  }
  FLASH_Lock();

  received_fw_size_ = 0;
  upgrade_status_ = UPGRADE_STA_IS_UPGRADING;
  req_pkt_counter_ = 0;

  event.data = &err;
  event.length = 1;
  hmi.Send(event);

  return RequestNextPacket();
}


ErrCode UpgradeHandler::ReceiveFW(Event_t &event) {

  uint32_t addr;
  uint16_t packet_index;
  uint16_t tmp;

  uint16_t data_len;

  uint8_t buff[2];

  packet_index = (uint16_t)(event.data[0]<<8 | event.data[1]);

  if ((packet_index < max_packet_) && (upgrade_status_ == UPGRADE_STA_IS_UPGRADING) && (packet_index == req_pkt_counter_)) {

    // every packet should have 512 bytes besides the last packet
    received_fw_size_ = packet_index<<7 + (event.length - 2);

    addr = FLASH_UPDATE_CONTENT + packet_index<<7;

    // if we have no enough
    if ((data_len % 2) != 0) data_len++;

    taskENTER_CRITICAL();
    FLASH_Unlock();

    for (int i = 2; i < data_len; i = i + 2) {
      tmp = ((event.data[i + 3]<<8) | event.data[i + 2]);

      FLASH_ProgramHalfWord(addr, tmp);

      addr = addr + 2;
    }

    FLASH_Lock();
    taskEXIT_CRITICAL();

    req_pkt_counter_++;
  }

  return RequestNextPacket();
}


ErrCode UpgradeHandler::EndUpgarde(Event_t &event) {
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
  hmi.Send(event);

  WatchDogInit();

  return err;
}


ErrCode UpgradeHandler::GetMainControllerVer(Event_t &event) {
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


ErrCode UpgradeHandler::CompareMCVer(Event_t &event) {
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


ErrCode UpgradeHandler::GetUpgradeStatus(Event_t &event) {
  uint8_t up_status = (uint8_t) upgrade_status_;

  event.data = &up_status;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode UpgradeHandler::GetModuleVer(Event_t &event) {

  LOG_I("SC req MODULE ver\n");
  uint8_t ver[32];
  uint32_t mac;
  int l;

  event.data = ver;

  for (int i = 0; i < CanBusControlor.ModuleCount; i++) {

    mac = CanBusControlor.ModuleMacList[i];

    if (CanModules.GetFirmwareVersion(BASIC_CAN_NUM, mac, (char *)ver) == true) {
      for (l = 0; l < 32; l++) {
        if (ver[l] == 0)
          break;
      }

      event.length = l;

      hmi.Send(event);
    }
  }

  return E_SUCCESS;
}
