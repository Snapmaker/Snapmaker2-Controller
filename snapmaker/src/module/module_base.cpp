#include "module_base.h"
#include "can_host.h"

#include "../service/upgrade.h"
#include "../common/protocol_sstp.h"
#include "../hmi/event_handler.h"

#include "linear.h"
#include "toolhead_3dp.h"
#include "toolhead_cnc.h"
#include "toolhead_laser.h"

// marlin headers
#include "src/inc/MarlinConfig.h"
#include HAL_PATH(src/HAL, fastio_STM32F1.h)

ModuleBase base(MODULE_DEVICE_ID_INVALID);

ModuleBase *static_modules[] = {&base, &linear, &printer, &laser, &cnc, NULL};

bool ModuleBase::lock_marlin_uart_ = false;
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
  if (cmd.data)
    return E_NO_MEM;

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
  if (ret != E_SUCCESS)
    goto out;
  // module reject to be upgraded
  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] != 1) {
    ret = E_SAME_STATE;
    goto out;
  }

  // 2. waiting for module become ready to receive fw
  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_UPGRADE_STATUS_REQ;
  cmd.length = 1;
  ret = canhost.SendExtCmdSync(cmd, 1000, 2);
  if (ret != E_SUCCESS)
    goto out;
  // timeout to be ready
  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] != 1) {
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
    ret = canhost.WaitExtCmdAck(cmd, 1000);
    if (ret != E_SUCCESS)
      goto out;

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
  vTaskDelay(portTICK_PERIOD_MS * 10);

  // didn't get ack from module
  if (canhost.SendExtCmdSync(cmd, 500) != E_SUCCESS)
    return E_HARDWARE;

  // module didn;t detect HIGH in dir pin
  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] != 1)
    return E_INVALID_STATE;

  WRITE(dir_pin, LOW);

  return E_SUCCESS;
}


void ModuleBase::LockMarlinUart() {
  lock_marlin_uart_ = true;
}


void ModuleBase::UnlockMarlinUart() {
  lock_marlin_uart_ = false;
}


void ModuleBase::ReportMarlinUart() {
  if (!lock_marlin_uart_)
    return;

  SERIAL_ECHOLN(";Locked UART");
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
