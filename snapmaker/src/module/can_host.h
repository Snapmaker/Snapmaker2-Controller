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
#ifndef SNAPMAKER_CAN_HOST_H_
#define SNAPMAKER_CAN_HOST_H_

#include "../common/error.h"
#include "../common/protocol_sstp.h"

#include "MapleFreeRTOS1030.h"

#include "can_channel.h"
#include "module_base.h"


#define CAN_STD_WAIT_QUEUE_MAX    (4)

#define CAN_RECV_SPEED_NORMAL 10  // ms
#define CAN_RECV_SPEED_HIGH 0

typedef void (*CanStdCmdCallback_t)(CanStdDataFrame_t &cmd);


typedef struct {
  Function_t          function;
  CanStdCmdCallback_t cb;
} __packed MessageMap_t;


typedef struct {
  MAC_t     mac;
  uint16_t  length;
  uint8_t   *data;
} CanExtCmd_t;


typedef struct {
  func_id_t id;
  uint8_t   length;
  uint8_t   *data;
} CanStdFuncCmd_t;


typedef struct {
  message_id_t id;
  uint8_t      length;
  uint8_t      *data;
} CanStdMesgCmd_t;

typedef struct {
  message_id_t          message;   /* command we are waiting for its ack */
  MessageBufferHandle_t queue;
} CanStdWaitNode_t;


typedef struct {
  // MAC_t     mac;   /* MAC ID of module which we are waiting for */
  uint16_t  cmd;   /* command we are waiting for its ack */
  MessageBufferHandle_t queue;
} CanExtWaitNode_t;

typedef enum {
  RECEIVER_SPEED_NORMAL,
  RECEIVER_SPEED_HIGH,
}RECEIVER_SPEED_E;

class CanHost {
  public:

    ErrCode Init();

    ErrCode SendStdCmd(CanStdMesgCmd_t &message);
    ErrCode SendStdCmd(CanStdFuncCmd_t &function, uint8_t sub_index=0);
    ErrCode SendStdCmdSync(CanStdFuncCmd_t &function, uint32_t timeout_ms=0, uint8_t retry=1, uint8_t sub_index=0);

    ErrCode SendExtCmd(CanExtCmd_t &cmd);
    ErrCode SendExtCmdSync(CanExtCmd_t &cmd, uint32_t timeout_ms=0, uint8_t retry=1);
    ErrCode WaitExtCmdAck(CanExtCmd_t &cmd, uint32_t timeout_ms=0, uint8_t retry=1);

    void SendHeartbeat();
    void SendEmergentStop();

    void ReceiveHandler(void *parameter);
    void EventHandler(void *parameter);

    ErrCode UpgradeModules(uint32_t fw_addr, uint32_t length);

    message_id_t RegisterFunction(Function_t const &function, CanStdCmdCallback_t callback);

    bool IrqCallback(CanStdDataFrame_t &frame);

    ErrCode BindMessageID(CanExtCmd_t &cmd, message_id_t *msg_buffer);
    void ShowModuleVersion(MAC_t mac);
    void SetReceiverSpeed(RECEIVER_SPEED_E speed);
    uint32_t mac(uint8_t index) {
      if (index < total_mac_)
        return mac_[index].val;
      else
        return MODULE_MAC_ID_INVALID;
    }

  private:
    message_id_t GetMessageID(func_id_t function_id, uint8_t sub_index = 0);
    ErrCode BindMessageID(MAC_t &mac, uint8_t mac_index);

    ErrCode InitModules(MAC_t &mac);
    ErrCode InitDynamicModule(MAC_t &mac, uint8_t mac_index);

    ErrCode AssignMessageRegion();

    ErrCode HandleExtCmd(uint8_t *cmd, uint16_t length);


  private:
    // command queue from ReceiveHandler() to EventHandler for standard command
    MessageBufferHandle_t std_cmd_q_;
    CanStdWaitNode_t      std_wait_q_[CAN_STD_WAIT_QUEUE_MAX];
    xSemaphoreHandle      std_wait_lock_;

    // parameters for extended command
    ProtocolSSTP proto_sstp_;           // SSTP protocol instance
    uint8_t      *parser_buffer_;       // buffer for parser of SSTP protocol
    uint8_t      *package_buffer_;      // buffer for packager of SSTP protocol
    MessageBufferHandle_t ext_cmd_q_;   // command queue from ReceiveHandler() to EventHandler for extend command
    CanExtWaitNode_t      ext_wait_q_;
    xSemaphoreHandle      ext_wait_lock_;
    uint8_t receiver_speed_ = CAN_RECV_SPEED_NORMAL;

    // map for message id and function id
    MessageMap_t  map_message_function_[MODULE_SUPPORT_MESSAGE_ID_MAX];
    uint16_t      total_message_id_;

    // in the second dimension, first element indicates begining we assign for this priority
    // second element indicates counts which has been used of this priority
    uint16_t      message_region_[MODULE_FUNC_PRIORITY_MAX][2];

    MAC_t   mac_[MODULE_SUPPORT_CONNECTED_MAX];
    uint8_t total_mac_;
};

extern CanHost canhost;

#endif  // #ifndef CAN_HOST_H_
