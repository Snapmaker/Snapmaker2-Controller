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
#ifndef SNAPMAKER_MODULE_BASE_H_
#define SNAPMAKER_MODULE_BASE_H_

#include "../common/error.h"
#include "../hmi/event_handler.h"

#define MODULE_MAC_ID_MASK        (0x1FFFFFFF)
#define MODULE_MAC_ID_INVALID     (0xFFFFFFFF)
#define MODULE_MAC_INDEX_INVALID   (0xFF)

#define MODULE_FUNCTION_ID_INVALID       (0xFFFF)
#define MODULE_FUNCTION_PRIORITY_INVALID (0xFF)
#define MODULE_FUNCTION_MAX_IN_ONE       (16) // upper limit of functions in one module

#define MODULE_MESSAGE_ID_INVALID   (0xFFFF)
#define MODULE_MESSAGE_ID_MASK      (0x1FF)

#define MODULE_DEVICE_ID_MASK       (0x1ff00000)
#define MODULE_DEVICE_ID_SHIFT      (20)

#define MODULE_MAKE_DEVICE_ID(id)   (id<<MODULE_DEVICE_ID_SHIFT)
#define MODULE_GET_DEVICE_ID(mac)   ((mac&MODULE_DEVICE_ID_MASK)>>MODULE_DEVICE_ID_SHIFT)

// to save memory, just support assign message id up to 64
#define MODULE_SUPPORT_MESSAGE_ID_MAX (128)
#define MODULE_SUPPORT_CONNECTED_MAX  (32)

#define MODULE_SUPPORT_SAME_DEVICE_MAX  (8)

#define MODULE_UPGRADE_PACKET_SIZE      (128)

#define MODULE_TYPE_STATIC  (1)
#define MODULE_TYPE_DYNAMIC (0)

typedef unsigned short func_id_t;
typedef unsigned short message_id_t;


typedef union {
  struct {
    uint32_t id:         29; // actual MAC id from modules
    uint32_t channel:    1;  // connected to CAN1 or CAN2
    uint32_t configured: 1;  // indicate its function id is bound with message id
    uint32_t type:       1;  // static or dynamic modules
  } bits;

  uint32_t val;
} MAC_t;


typedef struct {
    func_id_t id;
    uint16_t  priority:  4;
    uint16_t  sub_index: 3;
    uint16_t  channel:   1;
    uint16_t  mac_index: 8;
} Function_t;


enum ModuleDeviceID {
  MODULE_DEVICE_ID_3DP_SINGLE   ,    // 0
  MODULE_DEVICE_ID_CNC          ,    // 1
  MODULE_DEVICE_ID_1_6_W_LASER        ,    // 2
  MODULE_DEVICE_ID_LINEAR       ,    // 3
  MODULE_DEVICE_ID_LIGHT        ,    // 4
  MODULE_DEVICE_ID_ENCLOSURE    ,    // 5
  MODULE_DEVICE_ID_ROTARY       ,    // 6
  MODULE_DEVICE_ID_PURIFIER,         // 7
  MODULE_DEVICE_ID_EMERGENCY_STOP,   // 8
  MODULE_DEVICE_ID_CNC_TOOL_SETTING, // 9
  MODULE_DEVICE_ID_PRINT_V_SM1,      // 10
  MODULE_DEVICE_ID_FAN,              // 11
  MODULE_DEVICE_ID_LINEAR_TMC,       // 12
  MODULE_DEVICE_ID_3DP_DUAL,         // 13
  MODULE_DEVICE_ID_10W_LASER, // 14

  MODULE_DEVICE_ID_INVALID
};


enum ModuleFunctionPriority {
  MODULE_FUNC_PRIORITY_LOW,
  MODULE_FUNC_PRIORITY_MEDIUM,
  MODULE_FUNC_PRIORITY_HIGH,
  MODULE_FUNC_PRIORITY_EMERGENT,

  MODULE_FUNC_PRIORITY_MAX,
  MODULE_FUNC_PRIORITY_DEFAULT
};


/* following function id are known for controller, they are named as STATIC FUNCTION (ID)
 * in the future, there will be some new module plugged in system, and controller doesn't
 * know its functions, they are named as DYNAMIC FUNCTION (ID)
 */
enum ModuleFunctionID {
  MODULE_FUNC_ENDSTOP_STATE         ,  // 0
  MODULE_FUNC_PROBE_STATE           ,  // 1
  MODULE_FUNC_RUNOUT_SENSOR_STATE   ,  // 2
  MODULE_FUNC_STEPPER_CTRL          ,  // 3
  MODULE_FUNC_SET_SPINDLE_SPEED     ,  // 4
  MODULE_FUNC_GET_SPINDLE_SPEED     ,  // 5
  MODULE_FUNC_GET_NOZZLE_TEMP       ,  // 6
  MODULE_FUNC_SET_NOZZLE_TEMP       ,  // 7
  MODULE_FUNC_SET_FAN1              ,  // 8
  MODULE_FUNC_SET_FAN2              ,  // 9
  MODULE_FUNC_SET_3DP_PID           ,  // 10
  MODULE_FUNC_SET_CAMERA_POWER      ,  // 11
  MODULE_FUNC_SET_LASER_FOCUS       ,  // 12
  MODULE_FUNC_GET_LASER_FOCUS       ,  // 13
  MODULE_FUNC_SET_LIGHTBAR_COLOR    ,  // 14
  MODULE_FUNC_ENCLOSURE_DOOR_STATE  ,  // 15
  MODULE_FUNC_REPORT_3DP_PID        ,  // 16
  MODULE_FUNC_PROOFREAD_KNIFE       ,  // 17
  MODULE_FUNC_SET_ENCLOSURE_LIGHT   ,  // 18
  MODULE_FUNC_SET_ENCLOSURE_FAN     ,  // 19
  MODULE_FUNC_REPORT_EMERGENCY_STOP ,  // 20
  MODULE_FUNC_TMC_IOCTRL            ,  // 21
  MODULE_FUNC_TMC_PUBLISH           ,  // 22
  MODULE_FUNC_SET_PURIFIER          ,  // 23
  MODULE_FUNC_REPORT_PURIFIER       ,  // 24
  MODULE_FUNC_SET_AUTOFOCUS_LIGHT   ,  // 25
  MODULE_FUNC_REPORT_SECURITY_STATUS,  // 26
  MODULE_FUNC_ONLINE_SYNC           ,  // 27
  MODULE_FUNC_SET_PROTECT_TEMP      ,  // 28
  MODULE_FUNC_LASER_CTRL            ,  // 29
  MODULE_FUNC_GET_LASER_HW_VERSION  ,  // 30
  MODULE_FUNC_REPORT_PIN_STATE      ,  // 31
  MODULE_FUNC_CONFIRM_PIN_STATE     ,  // 32

  MODULE_FUNC_MAX
};

/* for dynamic functions, controller also need to assign message id to them
 * so we need to save some spare message id, below defined the spare message id account
 * for each priority except priority LOW
 */
#define MODULE_SPARE_MESSAGE_ID_EMERGENT 2
#define MODULE_SPARE_MESSAGE_ID_HIGH     5
#define MODULE_SPARE_MESSAGE_ID_MEDIUM   5


// index is function id
const uint8_t module_prio_table[][2] = {
  // FUNCID                               Priority(0-3)              possible total of function in network
  {/* MODULE_FUNC_ENDSTOP_STATE       */  MODULE_FUNC_PRIORITY_HIGH,      5}, // for now we have 5 axes in A250/A350
  {/* MODULE_FUNC_PROBE_STATE         */  MODULE_FUNC_PRIORITY_HIGH,      2}, // we may have 2 probes in system
  {/* MODULE_FUNC_RUNOUT_SENSOR_STATE */  MODULE_FUNC_PRIORITY_HIGH,      2},
  {/* MODULE_FUNC_STEPPER_CTRL        */  MODULE_FUNC_PRIORITY_LOW,       0},
  {/* MODULE_FUNC_SET_SPINDLE_SPEED   */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_GET_SPINDLE_SPEED   */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_GET_NOZZLE_TEMP     */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
  {/* MODULE_FUNC_SET_NOZZLE_TEMP     */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
  {/* MODULE_FUNC_SET_FAN1            */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
  {/* MODULE_FUNC_SET_FAN2            */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
  {/* MODULE_FUNC_SET_3DP_PID         */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
  {/* MODULE_FUNC_SET_CAMERA_POWER    */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_SET_LASER_FOCUS     */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_GET_LASER_FOCUS     */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_SET_LIGHTBAR_COLOR  */  MODULE_FUNC_PRIORITY_LOW,       0},
  {/* MODULE_FUNC_ENCLOSURE_STATE     */  MODULE_FUNC_PRIORITY_HIGH,      1},
  {/* MODULE_FUNC_REPORT_3DP_PID      */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_PROOFREAD_KNIFE     */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_SET_ENCLOSURE_LIGHT */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_SET_ENCLOSURE_FAN   */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* FUNC_REPORT_EMERGENCY_STOP      */  MODULE_FUNC_PRIORITY_EMERGENT,  1},
  {/* MODULE_FUNC_TMC_IOCTRL          */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_TMC_PUBLISH         */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_SET_PURIFIER        */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_REPORT_PURIFIER     */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
  {/* MODULE_FUNC_SET_AUTOFOCUS_LIGHT    */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
  {/* MODULE_FUNC_REPORT_SECURITY_STATUS */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
  {/* MODULE_FUNC_ONLINE_SYNC            */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
  {/* MODULE_FUNC_SET_PROTECT_TEMP       */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
  {/* MODULE_FUNC_LASER_CTRL        */  MODULE_FUNC_PRIORITY_MEDIUM,      1},
  {/* MODULE_FUNC_GET_LASER_HW_VERSION  */  MODULE_FUNC_PRIORITY_MEDIUM,  1},
  {/* MODULE_FUNC_REPORT_PIN_STATE   */  MODULE_FUNC_PRIORITY_MEDIUM,     1},
  {/* MODULE_FUNC_CONFIRM_PIN_STATE  */  MODULE_FUNC_PRIORITY_MEDIUM,     1},
};


#define MODULE_EXT_CMD_INDEX_ID   (0)
#define MODULE_EXT_CMD_INDEX_DATA (1)
enum ModuleExtendCommand {
  MODULE_EXT_CMD_CONFIG_REQ = 0,
  MODULE_EXT_CMD_CONFIG_ACK,

  MODULE_EXT_CMD_GET_FUNCID_REQ,
  MODULE_EXT_CMD_GET_FUNCID_ACK,

  MODULE_EXT_CMD_SET_MESG_ID_REQ,
  MODULE_EXT_CMD_SET_MESG_ID_ACK,

  MODULE_EXT_CMD_START_UPGRADE_REQ,
  MODULE_EXT_CMD_START_UPGRADE_ACK,

  MODULE_EXT_CMD_TRANS_FW_REQ,
  MODULE_EXT_CMD_TRANS_FW_ACK,

  MODULE_EXT_CMD_END_UPGRADE_REQ,

  MODULE_EXT_CMD_VERSION_REQ,
  MODULE_EXT_CMD_VERSION_ACK,

  MODULE_EXT_CMD_SSID_REQ,
  MODULE_EXT_CMD_SSID_ACK,

  MODULE_EXT_CMD_LINEAR_LENGTH_REQ,
  MODULE_EXT_CMD_LINEAR_LENGTH_ACK,

  MODULE_EXT_CMD_LINEAR_LEAD_REQ,
  MODULE_EXT_CMD_LINEAR_LEAD_ACK,

  MODULE_EXT_CMD_SET_ENDSTOP_POS_REQ,
  MODULE_EXT_CMD_SET_ENDSTOP_POS_ACK,

  MODULE_EXT_CMD_GET_UPGRADE_STATUS_REQ,
  MODULE_EXT_CMD_GET_UPGRADE_STATUS_ACK,

  MODULE_EXT_CMD_INFORM_UPGRADE_START,

  MODULE_EXT_CMD_INVALID
};


enum ModuleToolHeadType {
  MODULE_TOOLHEAD_UNKNOW,

  MODULE_TOOLHEAD_3DP,
  MODULE_TOOLHEAD_CNC,
  MODULE_TOOLHEAD_LASER,
  MODULE_TOOLHEAD_LASER_10W
};

enum LockMarlinUartSource {
  LOCK_SOURCE_NONE,
  LOCK_SOURCE_ENCLOSURE,
  LOCK_SOURCE_EMERGENCY_STOP,
};

class ModuleBase {
  public:
    ModuleBase(uint16_t id): device_id_(id) {}

    static ErrCode Upgrade(MAC_t &mac, uint32_t fw_addr, uint32_t length);
    static ErrCode InitModule8p(MAC_t &mac, int dir_pin, uint8_t index);

    static ModuleToolHeadType toolhead() { return toolhead_; }

    static bool lock_marlin_uart() { return lock_marlin_uart_; };
    static LockMarlinUartSource lock_marlin_source() { return lock_marlin_source_; };
    static void LockMarlinUart(LockMarlinUartSource source=LOCK_SOURCE_NONE);
    static void UnlockMarlinUart();
    static void ReportMarlinUart();
    static void StaticProcess() {
      if (++timer_in_static_process_ < 100) return;

      timer_in_static_process_ = 0;

      ReportMarlinUart();
    }

    static ErrCode SetMAC(SSTP_Event_t &event);
    static ErrCode GetMAC(SSTP_Event_t &event);

    virtual ErrCode Init(MAC_t &mac, uint8_t mac_index) { return E_SUCCESS; }
    virtual ErrCode PostInit() { return E_SUCCESS; }  // Called after all modules are initialized
    virtual void Process() { return; }

    virtual bool IsOnline(uint8_t sub_index = 0) { return false; }

    virtual uint32_t mac(uint8_t sub_index = 0) { return MODULE_MAC_ID_INVALID; }

    uint16_t device_id() { return device_id_; }

  protected:
    static void SetToolhead(ModuleToolHeadType toolhead);

  protected:
    uint16_t device_id_;

    static bool lock_marlin_uart_;
    static LockMarlinUartSource lock_marlin_source_;
    static uint16_t timer_in_static_process_;

  private:
    static ModuleToolHeadType toolhead_;
};

extern ModuleBase *static_modules[];

#endif  // #ifndef MODULE_BASE_H_
