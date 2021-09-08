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
#include "can_host.h"
#include "linear.h"
#include "../common/debug.h"
#include "../snapmaker.h"

#include "src/Marlin.h"

#define CAN_CHANNEL_MASK      (0x3)
#define CAN_CHANNEL_SHIFT     (10)
#define CAN_CHANNEL(val)  ((val&CAN_CHANNEL_MASK)>>CAN_CHANNEL_SHIFT);

#define CAN_MESSAGE_ID_MASK   (0x1FF)
#define CAN_MESSAGE_ID_SHIFT  (0)
#define CAN_MESSAGE_ID(val)   ((val&CAN_MESSAGE_ID_MASK)>>CAN_MESSAGE_ID_SHIFT)

CanHost canhost;

/**
 * IRQ callback for can channel, to handle emergency message,
 * such as endstop state from linear module
 * Parameter:
 *  message_id  - message id from CAN ID field
 *  cmd         - command buffer
 *  length      - command length
 * Return:
 *  true        - message is handled
 *  false       - message is not handled
 */
static bool CANIrqCallback(CanStdDataFrame_t &frame) {
  return canhost.IrqCallback(frame);
}


bool CanHost::IrqCallback(CanStdDataFrame_t &frame) {
  // handle only emergency and high prio message in IRQ callback
  if (frame.id.bits.msg_id >= message_region_[MODULE_FUNC_PRIORITY_HIGH][0])
    return false;

  // didn't register callback for this message, just return true to bypass it
  if (!map_message_function_[frame.id.bits.msg_id].cb)
    return true;

  map_message_function_[frame.id.bits.msg_id].cb(frame);

  return true;
}


ErrCode CanHost::Init() {
  int i;

  for (i = 0; i < MODULE_SUPPORT_MESSAGE_ID_MAX; i++) {
    map_message_function_[i].cb       = 0;
    map_message_function_[i].function.id = MODULE_FUNCTION_ID_INVALID;
  }
  total_message_id_ = 0;

  for (i = 0; i < MODULE_SUPPORT_CONNECTED_MAX; i++) {
    mac_[i].val = MODULE_MAC_ID_INVALID;
  }
  total_mac_ = 0;

  // init parameters for standard command
  std_cmd_q_ = xMessageBufferCreate(CAN_STD_CMD_QUEUE_SIZE * CAN_STD_CMD_ELEMENT_SIZE);
  configASSERT(std_cmd_q_);

  for (i = 0; i < CAN_STD_WAIT_QUEUE_MAX; i++) {
    std_wait_q_[i].message = MODULE_MESSAGE_ID_INVALID;
    std_wait_q_[i].queue  = xMessageBufferCreate(CAN_STD_CMD_ELEMENT_SIZE);
    configASSERT(std_wait_q_[i].queue);
  }
  std_wait_lock_ = xSemaphoreCreateMutex();


  ext_cmd_q_ = xMessageBufferCreate(CAN_EXT_CMD_QUEUE_SIZE);
  configASSERT(ext_cmd_q_);

  // alloc continuous memory for the temp buffer of parser
  parser_buffer_  = (uint8_t *)pvPortMalloc(520);
  configASSERT(parser_buffer_);

  package_buffer_ = (uint8_t *)pvPortMalloc(1024);
  configASSERT(package_buffer_);

  ext_wait_q_.cmd    = SSTP_INVALID_EVENT_ID;
  // ext_wait_q_.mac.val = MODULE_MAC_ID_INVALID;
  ext_wait_q_.queue  = xMessageBufferCreate(CAN_EXT_CMD_QUEUE_SIZE);
  ext_wait_lock_     = xSemaphoreCreateMutex();

  // init can channels
  if (can.Init(CANIrqCallback) != E_SUCCESS)
    LOG_E("Failed to init can channel\n");

  AssignMessageRegion();

  return E_SUCCESS;
}


message_id_t CanHost::GetMessageID(func_id_t function_id, uint8_t sub_index) {
  // search redblack tree by function_id
  for (int i = 0; i < MODULE_SUPPORT_MESSAGE_ID_MAX; i++) {
    if (map_message_function_[i].function.id == function_id &&
        map_message_function_[i].function.sub_index == sub_index)
      return i;
  }

  return MODULE_MESSAGE_ID_INVALID;
}


ErrCode CanHost::SendStdCmd(CanStdMesgCmd_t &message) {
  CanPacket_t  packet;

  if (message.id > MODULE_SUPPORT_CONNECTED_MAX)
    return E_PARAM;

  if (map_message_function_[message.id].function.id == MODULE_FUNCTION_ID_INVALID)
    return E_NO_RESRC;

  packet.ch = (CanChannelNumber)map_message_function_[message.id].function.channel;
  packet.id = message.id;

  packet.ft     = CAN_FRAME_STD_DATA;
  packet.data   = message.data;
  packet.length = message.length;

  return can.Write(packet);
}


ErrCode CanHost::SendStdCmd(CanStdFuncCmd_t &function, uint8_t sub_index) {
  CanPacket_t  packet;
  message_id_t msg_id;
  ErrCode      ret = E_FAILURE;

  if ((msg_id = GetMessageID(function.id, sub_index)) == MODULE_MESSAGE_ID_INVALID)
    return E_PARAM;

  packet.ch = (CanChannelNumber)map_message_function_[msg_id].function.channel;
  packet.id = msg_id;

  packet.ft     = CAN_FRAME_STD_DATA;
  packet.data   = function.data;
  packet.length = function.length;

  ret = can.Write(packet);
  return ret;
}



void CanHost::SendHeartbeat() {
  CanPacket_t packet = {CAN_CH_1, CAN_FRAME_STD_REMOTE, 0x01, 0, 0};
  can.Write(packet);
  packet.ch = CAN_CH_2;
  can.Write(packet);
}

void CanHost::SendEmergencyStop() {
  CanPacket_t packet = {CAN_CH_1, CAN_FRAME_STD_REMOTE, 0x02, 0, 0};
  can.Write(packet);
  packet.ch = CAN_CH_2;
  can.Write(packet);
}

ErrCode CanHost::SendStdCmdSync(CanStdFuncCmd_t &cmd, uint32_t timeout_ms, uint8_t retry, uint8_t sub_index) {
  ErrCode  ret;
  uint16_t tmp_u16;
  int      i;

  if ((tmp_u16 = GetMessageID(cmd.id, sub_index)) == MODULE_MESSAGE_ID_INVALID)
    return E_PARAM;

  xSemaphoreTake(std_wait_lock_, 0);
  for (i = 0; i < CAN_STD_WAIT_QUEUE_MAX; i++) {
    // check if we have free node in wait queue
    if (std_wait_q_[i].message == MODULE_MESSAGE_ID_INVALID) {
      // got free node, for std message, give it unified MAC
      std_wait_q_[i].message = tmp_u16;
      break;
    }
  }
  xSemaphoreGive(std_wait_lock_);

  ret = SendStdCmd(cmd, sub_index);
  if (ret != E_SUCCESS) {
    goto out;
  }

  tmp_u16 = xMessageBufferReceive(std_wait_q_[i].queue, cmd.data, CAN_STD_CMD_ELEMENT_SIZE - 2, pdMS_TO_TICKS(timeout_ms));

  if (!tmp_u16) {
    ret = E_TIMEOUT;
    goto out;
  }

  cmd.length = tmp_u16;

out:
  // release node of wait queue
  xSemaphoreTake(std_wait_lock_, 0);

  std_wait_q_[i].message = MODULE_MESSAGE_ID_INVALID;

  xSemaphoreGive(std_wait_lock_);

  return ret;
}


ErrCode CanHost::SendExtCmd(CanExtCmd_t &cmd) {
  CanPacket_t packet;
  ErrCode     ret = E_FAILURE;
  uint16_t length = cmd.length;
  packet.data = package_buffer_;

  if (proto_sstp_.Package(cmd.data, packet.data, length) != E_SUCCESS)
    return E_FAILURE;

  packet.length = length;
  packet.id     = cmd.mac.bits.id;
  packet.ch     = (CanChannelNumber)cmd.mac.bits.channel;
  packet.ft     = CAN_FRAME_EXT_DATA;

  ret = can.Write(packet);
  return ret;
}


ErrCode CanHost::SendExtCmdSync(CanExtCmd_t &cmd, uint32_t timeout_ms, uint8_t retry) {
  ErrCode   ret = E_SUCCESS;
  uint16_t tmp_u16;

  xSemaphoreTake(ext_wait_lock_, 0);
    // check if we have free node in wait queue
    // if (ext_wait_q_.mac.val == MODULE_MAC_ID_INVALID) {
    //   // got free node, set the mac to make it be used
    //   ext_wait_q_.mac.bits.id = cmd.mac.bits.id;
    //   // ack = req + 1
    if (ext_wait_q_.cmd == SSTP_INVALID_EVENT_ID) {
      ext_wait_q_.cmd = cmd.data[MODULE_EXT_CMD_INDEX_ID] + 1;
    }
    else {
      xSemaphoreGive(ext_wait_lock_);
      return E_NO_RESRC;
    }
  xSemaphoreGive(ext_wait_lock_);

  // no free node in wait queue
  for (; retry > 0; retry--) {
    ret = SendExtCmd(cmd);
    if (ret != E_SUCCESS) {
      vTaskDelay(pdMS_TO_TICKS(timeout_ms));
      continue;
    }

  // just receive data field
    tmp_u16 = xMessageBufferReceive(ext_wait_q_.queue, cmd.data, CAN_EXT_CMD_QUEUE_SIZE, pdMS_TO_TICKS(timeout_ms));

    if (!tmp_u16) {
      ret = E_TIMEOUT;
      continue;
    }
    else {
      cmd.length = tmp_u16;
      break;
    }
  }

  // release node of wait queue
  xSemaphoreTake(ext_wait_lock_, 0);
  // ext_wait_q_.mac.val = MODULE_MAC_ID_INVALID;
  ext_wait_q_.cmd = SSTP_INVALID_EVENT_ID;
  xSemaphoreGive(ext_wait_lock_);

  return ret;
}


ErrCode CanHost::WaitExtCmdAck(CanExtCmd_t &cmd, uint32_t timeout_ms, uint8_t retry) {
  uint16_t tmp_u16;
  uint8_t  cmd_id;

  if (!cmd.data)
    return E_PARAM;

  cmd_id = cmd.data[MODULE_EXT_CMD_INDEX_ID];

  for (; retry > 0; retry--) {
    // handle extended command secondly
    tmp_u16 = xMessageBufferReceive(ext_cmd_q_, cmd.data, cmd.length, pdMS_TO_TICKS(timeout_ms));
    if (!tmp_u16)
      continue;

    if (cmd.data[MODULE_EXT_CMD_INDEX_ID] != cmd_id)
      continue;

    // tmp_u16 = xMessageBufferReceive(ext_cmd_q_, &mac, 4, pdMS_TO_TICKS(timeout_ms));
    // if (tmp_u16 != 4)
    //   continue;

    // if (cmd.mac.bits.id != mac.bits.id || cmd.data[MODULE_EXT_CMD_INDEX_DATA] != cmd_id)
    //   continue;

    return E_SUCCESS;
  }

  return E_TIMEOUT;
}
/** Set ReceiveHandler delay time
 * Adjust the speed according to the amount of data received
 */
void CanHost::SetReceiverSpeed(RECEIVER_SPEED_E speed) {
  switch (speed){
    case RECEIVER_SPEED_NORMAL:
      receiver_speed_ = CAN_RECV_SPEED_NORMAL;
      break;
    case RECEIVER_SPEED_HIGH:
      receiver_speed_ = CAN_RECV_SPEED_HIGH;
      break;
    default:
      receiver_speed_ = CAN_RECV_SPEED_NORMAL;
  }
}


/* To check if we got new event form CAN ISR
 * This function should be put in a independent task
 *
 */
void CanHost::ReceiveHandler(void *parameter) {
  CanStdDataFrame_t  std_cmd;
  uint16_t tmp_u16;

  MessageBufferHandle_t tmp_q;

  int i;

  for (;;) {
    tmp_q = NULL;

    // 1. check Std command
    if (can.Read(CAN_FRAME_STD_DATA, (uint8_t *)&std_cmd, CAN_STD_CMD_ELEMENT_SIZE) == CAN_STD_CMD_ELEMENT_SIZE) {
      // check if there is some one is wait for this message

      xSemaphoreTake(std_wait_lock_, 0);
      for (i = 0; i < CAN_STD_WAIT_QUEUE_MAX; i++) {
        if (std_wait_q_[i].message == std_cmd.id.bits.msg_id) {
          tmp_q = std_wait_q_[i].queue;
          break;
        }
      }
      xSemaphoreGive(std_wait_lock_);

      if (!tmp_q) {
        // send message to EventHandler()
        xMessageBufferSend(std_cmd_q_, &std_cmd, 2 + std_cmd.id.bits.length, pdMS_TO_TICKS(100));
      }
      else {
        // send message to SendStdMessageSync(), skip message id, which is the 2 bytes in begining
        xMessageBufferSend(tmp_q, std_cmd.data, std_cmd.id.bits.length, pdMS_TO_TICKS(100));
      }
    }

    // 2. check extended command
    if (proto_sstp_.Parse(can.ext_cmd(), parser_buffer_, tmp_u16) == E_SUCCESS) {
      // if we got a complete command in the ring buffer, one MAC id will be in the following 4 bytes
      // need to check it out, then we know who send us the command
      // can.ext_cmd().RemoveMulti((uint8_t *)&mac, 4);

      xSemaphoreTake(ext_wait_lock_, 0);
      // check if there is some one is wait for this ack
      // if (ext_wait_q_.mac.bits.id == mac.bits.id &&
      //     ext_wait_q_.cmd == parser_buffer_[MODULE_EXT_CMD_INDEX_ID])
      if (ext_wait_q_.cmd == parser_buffer_[MODULE_EXT_CMD_INDEX_ID]) {
        tmp_q = ext_wait_q_.queue;
      }
      xSemaphoreGive(ext_wait_lock_);


      if (tmp_q) {
        // send message to SendExtMessageSync(), just send data field, because it knows the event id and opcode
        // LOG_V("sync ext cmd: %u\n", parser_buffer_[MODULE_EXT_CMD_INDEX_ID]);
        xMessageBufferSend(tmp_q, parser_buffer_, tmp_u16, pdMS_TO_TICKS(100));
      }
      else {
        // send message to EventHandler()
        xMessageBufferSend(ext_cmd_q_, parser_buffer_, tmp_u16, pdMS_TO_TICKS(100));
        // LOG_V("async ext cmd: %u\n", parser_buffer_[MODULE_EXT_CMD_INDEX_ID]);
        // xMessageBufferSend(ext_cmd_q_, &mac, 4, pdMS_TO_TICKS(100));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(receiver_speed_));
  }
}


/* To handle async event from ReceiveHandler()
 * This function should be perfromed in a independent task
 * */
void CanHost::EventHandler(void *parameter) {
  CanStdDataFrame_t  std_cmd;
  uint16_t length = 0;
  MAC_t   mac;

  EventGroupHandle_t event_group = ((SnapmakerHandle_t)parameter)->event_group;

  CanPacket_t pkt = {CAN_CH_2, CAN_FRAME_EXT_REMOTE, 0x01, 0, 0};

  LOG_I("Scanning modules ...\n");
  vTaskDelay(pdMS_TO_TICKS(2000));

  if (can.Write(pkt) != E_SUCCESS)
    LOG_E("No module on CAN%u!\n", 2);
  // delay 1s, to get all mac form CAN2

  pkt.ch = CAN_CH_1;
  if (can.Write(pkt) != E_SUCCESS)
    LOG_E("No module on CAN%u!\n", 1);

  vTaskDelay(pdMS_TO_TICKS(1000));

  // read all mac
  while (can.Read(CAN_FRAME_EXT_REMOTE, (uint8_t *)&mac, 1)) {
    LOG_I("\nNew Module: 0x%08X\n", mac.val);
    InitModules(mac);
  }

  linear_p->UpdateMachineSize();

  for (int i = 0; static_modules[i] != NULL; i++) {
      static_modules[i]->PostInit();
  }
  // broadcase modules have been initialized
  xEventGroupSetBits(event_group, EVENT_GROUP_MODULE_READY);

  for (;;) {
    if (can.Read(CAN_FRAME_EXT_REMOTE, (uint8_t *)&mac, 1)) {
      LOG_I("New Module: 0x%08X\n", mac.val);
      InitModules(mac);
    }

    for (;;) {
      // check if we got standard command from modules
      length = xMessageBufferReceive(std_cmd_q_, &std_cmd, sizeof(CanStdDataFrame_t), 0);
      if (!length)
        break;

      // check if someone register callback for this message
      uint16_t message_id = std_cmd.id.bits.msg_id;
      if (message_id >= MODULE_SUPPORT_MESSAGE_ID_MAX)
        continue;

      if (map_message_function_[message_id].cb) {
        map_message_function_[message_id].cb(std_cmd);
      }

      // if no callback, maybe need to send it to screen?
    }

    ModuleBase::StaticProcess();
    for (int i = 0; static_modules[i] != NULL; i++)
      static_modules[i]->Process();

    vTaskDelay(pdMS_TO_TICKS(receiver_speed_));
  }
}


ErrCode CanHost::AssignMessageRegion() {
  uint8_t prio_of_function;
  uint8_t total_of_same_function;

  uint16_t total_message = 0;

  for (int i = 0; i < MODULE_FUNC_MAX; i++) {
    prio_of_function       = module_prio_table[i][0];
    total_of_same_function = module_prio_table[i][1];

    message_region_[prio_of_function][0] += total_of_same_function;
  }

  for (int i = 0; i < MODULE_FUNC_PRIORITY_MAX; i++) {
    total_message += message_region_[i][0];
  }

  if (total_message >= MODULE_SUPPORT_MESSAGE_ID_MAX) {
    SERIAL_ECHOLN("message is not enough for used!");
    return E_NO_RESRC;
  }

  message_region_[MODULE_FUNC_PRIORITY_EMERGENT][0] += MODULE_SPARE_MESSAGE_ID_EMERGENT;
  message_region_[MODULE_FUNC_PRIORITY_HIGH][0]     += (MODULE_SPARE_MESSAGE_ID_HIGH + message_region_[MODULE_FUNC_PRIORITY_EMERGENT][0]);
  message_region_[MODULE_FUNC_PRIORITY_MEDIUM][0]   += (MODULE_SPARE_MESSAGE_ID_MEDIUM + message_region_[MODULE_FUNC_PRIORITY_HIGH][0]);
  message_region_[MODULE_FUNC_PRIORITY_LOW][0]      = MODULE_SUPPORT_MESSAGE_ID_MAX;
  SERIAL_ECHOLN("Message ID region:");
  SERIAL_ECHOLNPAIR("emergent: ", 0, " - ", message_region_[MODULE_FUNC_PRIORITY_EMERGENT][0]-1);
  SERIAL_ECHOLNPAIR("high    : ", message_region_[MODULE_FUNC_PRIORITY_EMERGENT][0], " - ", message_region_[MODULE_FUNC_PRIORITY_HIGH][0]-1);
  SERIAL_ECHOLNPAIR("medium  : ", message_region_[MODULE_FUNC_PRIORITY_HIGH][0], " - ", message_region_[MODULE_FUNC_PRIORITY_MEDIUM][0]-1);
  SERIAL_ECHOLNPAIR("low     : ", message_region_[MODULE_FUNC_PRIORITY_MEDIUM][0], " - ", message_region_[MODULE_FUNC_PRIORITY_LOW][0]-1);
  SERIAL_EOL();

  return E_SUCCESS;
}

void CanHost::ShowModuleVersion(MAC_t mac) {
  CanExtCmd_t cmd;

  char buffer[50];
  if (mac.val == MODULE_MAC_ID_INVALID)
    return;

  // version of modules
  sprintf(buffer, "Module 0x%08X: ", (int)mac.bits.id);
  SERIAL_ECHOPAIR(buffer);
  cmd.data = (uint8_t *)buffer;
  cmd.mac     = mac;
  cmd.data[0] = MODULE_EXT_CMD_VERSION_REQ;
  cmd.length  = 1;
  if (canhost.SendExtCmdSync(cmd, 500) != E_SUCCESS) {
    SERIAL_ECHOPAIR("failed or failed to get ver\n");
  } else {
    buffer[cmd.length] = 0;
    SERIAL_ECHOPAIR(buffer+2, "\n");
  }
}

ErrCode CanHost::InitModules(MAC_t &mac) {
  int      i;
  uint16_t device_id = MODULE_GET_DEVICE_ID(mac.val);
  ErrCode  ret;

  bool     existed   = false;
  uint8_t  mac_index = total_mac_;

  if (total_mac_ >= MODULE_SUPPORT_CONNECTED_MAX)
    return E_NO_RESRC;

  ShowModuleVersion(mac);
  // check if this mac is configured
  for (i = 0; i < total_mac_; i++) {
    if (mac.bits.id == mac_[i].bits.id) {
      if (mac_[i].bits.configured) {
        // if yes, just re-bind function id to message id
        return BindMessageID(mac, i);
      }
      else {
        // if no, just try to configure it again
        existed = true;
        mac_index = i;
        break;
      }
    }
  }

  // check if it is static modules then init it
  for (i = 0; static_modules[i] != NULL; i++) {
    if (static_modules[i]->device_id() == device_id) {
      mac.bits.type = MODULE_TYPE_STATIC;

      ret = static_modules[i]->Init(mac, mac_index);
      if (ret == E_SUCCESS) {
        mac.bits.configured = 1;
      }

      goto out;
    }
  }

  // it is dynamic modules
  ret = InitDynamicModule(mac, mac_index);
  if (ret == E_SUCCESS) {
    mac.bits.configured = 1;
  }

out:
  // if doesn't exist, record it in array
  if (!existed)
    mac_[total_mac_++].val = mac.val;

  return E_SUCCESS;
}


ErrCode CanHost::InitDynamicModule(MAC_t &mac, uint8_t mac_index) {
  CanExtCmd_t cmd;
  uint8_t     func_buffer[MODULE_FUNCTION_MAX_IN_ONE*2 + 2];

  message_id_t message_id[MODULE_FUNCTION_MAX_IN_ONE] = { MODULE_MESSAGE_ID_INVALID };
  Function_t function;

  int i;

  cmd.mac    = mac;
  cmd.data   = func_buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_FUNCID_REQ;

  // try to get function ids from module
  if (SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;

  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] > MODULE_FUNCTION_MAX_IN_ONE)
    cmd.data[MODULE_EXT_CMD_INDEX_DATA] = MODULE_FUNCTION_MAX_IN_ONE;

  function.channel    = mac.bits.channel;
  function.mac_index  = mac_index;
  function.priority   = MODULE_FUNC_PRIORITY_DEFAULT;
  function.sub_index  = 0;  // if 0 is proper for two same dynamic module plugged?

  // register function ids to can host, it will assign message id
  for (i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id   = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    message_id[i] = RegisterFunction(function, NULL);
  }

  return BindMessageID(cmd, message_id);
}


/* bind message id to modules's function
 *
 */
ErrCode CanHost::BindMessageID(CanExtCmd_t &cmd, message_id_t *msg_buffer) {
  uint8_t     map_buffer[MODULE_FUNCTION_MAX_IN_ONE*4 + 2];
  uint8       *func_buffer = cmd.data;

  int i;

  // set message id for functions in linear modules
  map_buffer[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_SET_MESG_ID_REQ;
  map_buffer[MODULE_EXT_CMD_INDEX_DATA] = func_buffer[MODULE_EXT_CMD_INDEX_DATA];

  for (i = 0; i < func_buffer[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    map_buffer[4*i + 2] = msg_buffer[i]>>8;
    map_buffer[4*i + 3] = msg_buffer[i] & 0x00FF;
    map_buffer[4*i + 4] = func_buffer[2*i + 2];
    map_buffer[4*i + 5] = func_buffer[2*i + 3];
    LOG_I("\tFunction [%3u] <-> Message [%3u]\n", func_buffer[2*i + 3], msg_buffer[i]);
  }

  cmd.data = map_buffer;
  cmd.length = 4*i + 2;

  return SendExtCmd(cmd);
}


ErrCode CanHost::BindMessageID(MAC_t &mac, uint8_t mac_index) {
  CanExtCmd_t cmd;
  uint8_t     func_buffer[MODULE_FUNCTION_MAX_IN_ONE*2 + 2];

  func_id_t    function_id;
  message_id_t message_id[MODULE_FUNCTION_MAX_IN_ONE] = { MODULE_MESSAGE_ID_INVALID };

  int i;

  cmd.mac    = mac;
  cmd.data   = func_buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_FUNCID_REQ;

  // try to get function ids from module
  if (SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;


  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] > MODULE_FUNCTION_MAX_IN_ONE)
    cmd.data[MODULE_EXT_CMD_INDEX_DATA] = MODULE_FUNCTION_MAX_IN_ONE;

  // register function ids to can host, it will assign message id
  for (i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function_id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);

    // search by function_id and mac_index
    for (int msg_id = 0; msg_id < MODULE_SUPPORT_MESSAGE_ID_MAX; msg_id++) {
      if (map_message_function_[msg_id].function.id == function_id &&
          map_message_function_[msg_id].function.mac_index == mac_index)
        message_id[i] = msg_id;
    }
  }

  return BindMessageID(cmd, message_id);
}


message_id_t CanHost::RegisterFunction(Function_t const &function, CanStdCmdCallback_t callback) {
  int start, end;

  // cannot change priority in reference 'function'
  uint8_t prio = MODULE_FUNC_PRIORITY_LOW;

  if (function.priority == MODULE_FUNC_PRIORITY_DEFAULT &&
      function.id < MODULE_FUNC_MAX) {
    prio = module_prio_table[function.id][0];
  }

  switch (prio) {
  case MODULE_FUNC_PRIORITY_EMERGENT:
    start = 0;
    end   = message_region_[MODULE_FUNC_PRIORITY_EMERGENT][0];
    break;

  case MODULE_FUNC_PRIORITY_HIGH:
    start = message_region_[MODULE_FUNC_PRIORITY_EMERGENT][0];
    end   = message_region_[MODULE_FUNC_PRIORITY_HIGH][0];
    break;

  case MODULE_FUNC_PRIORITY_MEDIUM:
    start = message_region_[MODULE_FUNC_PRIORITY_HIGH][0];
    end   = message_region_[MODULE_FUNC_PRIORITY_MEDIUM][0];
    break;

  case MODULE_FUNC_PRIORITY_LOW:
    start = message_region_[MODULE_FUNC_PRIORITY_MEDIUM][0];
    end   = MODULE_SUPPORT_MESSAGE_ID_MAX;
    break;

  default:
    return MODULE_MESSAGE_ID_INVALID;
    break;
  }

  for (; start < end; start++) {
    if (map_message_function_[start].function.id == MODULE_FUNCTION_ID_INVALID) {
      goto assign_message_id;
    }
  }

  return MODULE_MESSAGE_ID_INVALID;

assign_message_id:
  map_message_function_[start].function = function;
  map_message_function_[start].function.priority = prio;
  map_message_function_[start].cb = callback;

  message_region_[prio][1]++;

  return (message_id_t)start;
}


ErrCode CanHost::UpgradeModules(uint32_t fw_addr, uint32_t length) {
  int   i;

  SetReceiverSpeed(RECEIVER_SPEED_HIGH);
  // upgrade dynamic modules
  for (i = 0; i < MODULE_SUPPORT_CONNECTED_MAX; i++) {

    // to the end of mac
    if (mac_[i].val == MODULE_MAC_ID_INVALID)
      break;

    ModuleBase::Upgrade(mac_[i], fw_addr, length);
  }
  SetReceiverSpeed(RECEIVER_SPEED_NORMAL);
  // Waiting module to enter app
  vTaskDelay(pdMS_TO_TICKS(200));
  LOG_I("All upgraded!\n");
  // Restart all module
  disable_power_domain(POWER_DOMAIN_1|POWER_DOMAIN_2);
  vTaskDelay(pdMS_TO_TICKS(1000));
  enable_power_domain(POWER_DOMAIN_1|POWER_DOMAIN_2);
  vTaskDelay(pdMS_TO_TICKS(1000));

  CanPacket_t pkt = {CAN_CH_2, CAN_FRAME_EXT_REMOTE, 0x01, 0, 0};

  LOG_I("Scanning modules ...\n");

  if (can.Write(pkt) != E_SUCCESS)
    LOG_E("No module on CAN%u!\n", 2);

  pkt.ch = CAN_CH_1;
  if (can.Write(pkt) != E_SUCCESS)
    LOG_E("No module on CAN%u!\n", 1);


  return E_SUCCESS;
}
