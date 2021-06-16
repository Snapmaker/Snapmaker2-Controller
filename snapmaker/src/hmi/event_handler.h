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
#ifndef SNAPMAKER_EVENT_HANDLER_H_
#define SNAPMAKER_EVENT_HANDLER_H_

#include "../common/error.h"

#include "uart_host.h"

// event IDs
// gcode from PC
#define EID_GCODE_REQ         1
#define EID_GCODE_ACK         2
// gcode from file
#define EID_FILE_GCODE_REQ    3
#define EID_FILE_GCODE_ACK    4
// file operation
#define EID_FILE_OP_REQ       5
#define EID_FILE_OP_ACK       6
// status query
#define EID_SYS_CTRL_REQ      7
#define EID_SYS_CTRL_ACK      8

enum SysControlOpc: uint8_t {
  SYSCTL_OPC_UNUSED_0 = 0,
  SYSCTL_OPC_GET_STATUES,
  SYSCTL_OPC_GET_EXCEPTION,
  SYSCTL_OPC_START_WORK,
  SYSCTL_OPC_PAUSE,
  SYSCTL_OPC_RESUME,
  SYSCTL_OPC_STOP,
  SYSCTL_OPC_FINISH,
  SYSCTL_OPC_GET_LAST_LINE,
  SYSCTL_OPC_CLEAR_FAULT = 0xA,
  SYSCTL_OPC_RECOVER_POWER_LOSS = 0xB,
  SYSCTL_OPC_WAIT_EVENT = 0xC,
  SYSCTL_OPC_GET_HOME_STATUS = 0xE,
  SYSCTL_OPC_SET_LOG_LEVEL = 0xF,

  SYSCTL_OPC_TRANS_LOG,

  SYSCTL_OPC_MAX
};

// settings operation
#define EID_SETTING_REQ       9
#define EID_SETTING_ACK       0xa

enum SettingsOpc: uint8_t {
  SETTINGS_OPC_UNUSED_0 = 0,
  SETTINGS_OPC_SET_MACHINE_SIZE,
  SETTINGS_OPC_DO_AUTO_LEVELING,
  SETTINGS_OPC_SYNC_LEVEL_POINT,

  SETTINGS_OPC_DO_MANUAL_LEVELING = 4,
  SETTINGS_OPC_SET_LEVELING_PONIT,
  SETTINGS_OPC_ADJUST_Z_OFFSET,
  SETTINGS_OPC_SAVE_AND_EXIT_LEVELING,
  SETTINGS_OPC_EXIT_LEVELING,

  SETTINGS_OPC_RESTORE_TO_FACTORY = 9,
  SETTINGS_OPC_READ_FOCAL_LENGTH,
  SETTINGS_OPC_SET_FOCAL_LENGTH,
  SETTINGS_OPC_DO_MANUAL_FOCUSING,
  SETTINGS_OPC_DO_AUTO_FOCUSING,

  SETTINGS_OPC_DO_FAST_CALIBRATION = 0xe,
  SETTINGS_OPC_SET_RUNTIME_ENV,
  SETTINGS_OPC_GET_RUNTIME_ENV,
  SETTINGS_OPC_0x11,
  SETTINGS_OPC_0x12,
  SETTINGS_OPC_0x13,
  SETTINGS_OPC_GET_MACHINE_SIZE,

  SETTINGS_OPC_MAX
};


// movement command
#define EID_MOTION_REQ      0xb
#define EID_MOTION_ACK      0xc

enum MotionOpc: uint8_t {
  MOTION_OPC_UNUSED_0,
  MOTION_OPC_UNUSED_1,
  MOTION_OPC_DO_ABSOLUTE_MOVE,
  MOTION_OPC_DO_RELATIVE_MOVE,
  MOTION_OPC_DO_E_MOVE,

  MOTION_OPC_MAX
};


// laser&camera command set
#define EID_CAMERA_REQ    0xd
#define EID_CAMERA_ACK    0xe

enum CameraOpc: uint8_t {
  CAMERA_OPC_UNUSED_0,
  CAMERA_OPC_UNUSED_1,
  CAMERA_OPC_UNUSED_2,
  CAMERA_OPC_UNUSED_3,
  CAMERA_OPC_UNUSED_4,
  CAMERA_OPC_SET_BT_NAME,
  CAMERA_OPC_READ_BT_NAME,
  CAMERA_OPC_READ_BT_MAC,

  CAMERA_OPC_MAX
};


// Add-on command set
#define EID_ADDON_REQ      0x11
#define EID_ADDON_ACK      0x12

enum AddonOpc: uint8_t {
  ADDON_OPC_UNUSED_0,
  ADDON_OPC_GET_CHAMBER_STATUS,
  ADDON_OPC_SET_CHAMBER_LIGHT,
  ADDON_OPC_SET_CHAMBER_FAN,
  ADDON_OPC_SET_CHAMBER_DETECTION,
  ADDON_OPC_GET_ADDON_LIST,
  ADDON_OPC_GET_ADDON_INFO,
  ADDON_OPC_GET_ADDON_STOP,
  ADDON_OPC_GET_ROTATE_STATE,
  ADDON_OPC_GET_PURIFIER_STATE,
  ADDON_OPC_GET_PURIFIER_FAN_STATE,
  ADDON_OPC_SET_PURIFIER_FAN_STATE,
  ADDON_OPC_SET_PURIFIER_GEARS_STATE,
  ADDON_OPC_GET_PURIFIER_TIMELIFE_STATE,

  ADDON_OPC_MAX
};


// debug command set
#define EID_DEBUG_REQ     0x99
#define EID_DEBUG_ACK     0x9a

enum DebugOpc: uint8_t {
  DEBUG_OPC_UNUSED_0,
  DEBUG_OPC_SET_MODULE_MAC = 1,
  DEBUG_OPC_GET_MODULE_MAC,
  DEBUG_OPC_SET_LINEAR_LENGTH,
  DEBUG_OPC_GET_LINEAR_LENGTH,

  DEBUG_OPC_SET_LINEAR_LEAD = 5,
  DEBUG_OPC_GET_LINEAR_LEAD,

  DEBUG_OPC_MAX
};


// upgrade command set
#define EID_UPGRADE_REQ       0xa9
#define EID_UPGRADE_ACK       0xaa

enum UpgradeOpc: uint8_t {
  UPGRADE_OPC_START = 0,
  UPGRADE_OPC_TRANS_FW,
  UPGRADE_OPC_END,
  UPGRADE_OPC_GET_MC_VER,
  UPGRADE_OPC_COMPARE_VER,

  UPGRADE_OPC_GET_UP_STATUS = 5,
  UPGRADE_OPC_SYNC_MODULE_UP_STATUS,
  UPGRADE_OPC_GET_MODULE_VER = 7,

  UPGRADE_OPC_MAX
};

// gcode result
#define EID_GCODE_RESULT_ACK  0x06

enum GcodeResultOpc: uint8_t {
  GCODE_RESULT_OPC_SOF = 0,
  GCODE_RESULT_OPC_DATA,
  GCODE_RESULT_OPC_EOF,

  GCODE_RESULT_OPC_MAX
};


#define EVENT_IDX_EVENT_ID  0
#define EVENT_IDX_OP_CODE   1
#define EVENT_IDX_DATA0     2

#define UNDEFINED_CALLBACK  {0, NULL}


enum TaskOwner: uint8_t {
  TASK_OWN_MARLIN = 0,
  TASK_OWN_HMI,
  TASK_OWN_NONE
};

struct DispatcherParam {
  TaskOwner owner;
  uint8_t   *event_buff;
  uint16_t  size;
  MessageBufferHandle_t event_queue;
};

typedef struct DispatcherParam* DispatcherParam_t;

ErrCode DispatchEvent(DispatcherParam_t param);
void clear_hmi_gcode_queue();
void ack_gcode_event(uint8_t event_id, uint32_t line);

extern bool Screen_send_ok[];

extern UartHost hmi;

#endif  //#ifndef EVENT_HANDLER_H_
