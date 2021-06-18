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
#include "event_handler.h"

#include "../common/debug.h"

#include "../module/module_base.h"
#include "../module/can_host.h"
#include "../module/linear.h"
#include "../module/enclosure.h"
#include "../module/purifier.h"
#include "../module/emergency_stop.h"
#include "../module/toolhead_laser.h"
#include "../module/rotary_module.h"

#include "../service/bed_level.h"
#include "../service/upgrade.h"
#include "../service/system.h"

// marlin headers
#include "src/Marlin.h"
#include "src/module/motion.h"
#include "src/module/temperature.h"
#include "src/module/planner.h"
#include "src/libs/hex_print_routines.h"
#include "src/gcode/gcode.h"
#include "src/gcode/queue.h"


#define EVENT_ATTR_HAVE_MOTION  0x1
#define EVENT_ATTR_ABORT_MOTION 0x2

#define EVENT_ATTR_DEFAULT      0x0

#define EVENT_HANDLE_WITH_MARLIN  (EVENT_ATTR_HAVE_MOTION)

#define SUPPORT_MODULE_MAX  32

#define DEBUG_EVENT_HANDLER 0


typedef ErrCode (*CallbackFunc_t)(SSTP_Event_t &event);

struct EventCallback_t {
  uint8_t attr;
  CallbackFunc_t cb;
};


UartHost hmi;

uint8_t hmi_commands_in_queue = 0, hmi_cmd_queue_index_r = 0,
        hmi_cmd_queue_index_w = 0;
char hmi_command_queue[HMI_BUFSIZE][MAX_CMD_SIZE];
uint8_t hmi_send_opcode_queue[HMI_BUFSIZE];
uint32_t hmi_commandline_queue[HMI_BUFSIZE];

extern bool send_ok[BUFSIZE];
bool   Screen_send_ok[BUFSIZE];
uint8_t Screen_send_ok_opcode[BUFSIZE];
uint32_t CommandLine[BUFSIZE] = { INVALID_CMD_LINE };

extern uint8_t cmd_queue_index_w; // Ring buffer write position


/**
 *SC20 queue the gcdoe
 *para pgcode:the pointer to the gcode
 *para Lines:the line position of the gcode in the file
 *para Opcode:operation code
 * execution guaranteed
 */
void enqueue_hmi_to_marlin() {
  // guaranteed buffer available, shouldn't be missed, or screen status won't
  // sync. fetch as much command as possible
  while (commands_in_queue < BUFSIZE && hmi_commands_in_queue > 0) {
    // fetch from buffer queue
    strcpy(command_queue[cmd_queue_index_w],
           hmi_command_queue[hmi_cmd_queue_index_r]);
    Screen_send_ok[cmd_queue_index_w] = true;
    Screen_send_ok_opcode[cmd_queue_index_w] =
        hmi_send_opcode_queue[hmi_cmd_queue_index_r];
    CommandLine[cmd_queue_index_w] =
        hmi_commandline_queue[hmi_cmd_queue_index_r];
    send_ok[cmd_queue_index_w] = false;
    cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;

    hmi_cmd_queue_index_r = (hmi_cmd_queue_index_r + 1) % HMI_BUFSIZE;
    hmi_commands_in_queue--;
    commands_in_queue++;
  }
}


void Screen_enqueue_and_echo_commands(char *pgcode, uint32_t line,
                                      uint8_t opcode) {
  int i;

  // we put HMI command to Marlin queue firstly
  // to avoid jumping directly, we check the condition before call it
  if (commands_in_queue < BUFSIZE && hmi_commands_in_queue > 0)
    enqueue_hmi_to_marlin();

  if (hmi_commands_in_queue >= HMI_BUFSIZE) {
    LOG_E("HMI gcode buffer is full, losing line: %u\n", line);
    return;
  }

  // ignore comment
  if (pgcode[0] == ';') {
    ack_gcode_event(opcode, line);
    return;
  }

  // won't put comment part to queue, and limit cmd size to MAX_CMD_SIZE
  for (i = 0; i < MAX_CMD_SIZE; i++) {
    if (pgcode[i] == '\n' || pgcode[i] == '\r' || pgcode[i] == 0 ||
        pgcode[i] == ';') {
      hmi_command_queue[hmi_cmd_queue_index_w][i] = 0;
      break;
    }
    hmi_command_queue[hmi_cmd_queue_index_w][i] = pgcode[i];
  }

  if (i >= MAX_CMD_SIZE) {
    hmi_command_queue[hmi_cmd_queue_index_w][MAX_CMD_SIZE - 1] = 0;
    LOG_E("line[%u] too long: %s\n", line,
          hmi_command_queue[hmi_cmd_queue_index_w]);
    ack_gcode_event(opcode, line);
    return;
  }

  hmi_commandline_queue[hmi_cmd_queue_index_w] = line;
  hmi_send_opcode_queue[hmi_cmd_queue_index_w] = opcode;
  hmi_cmd_queue_index_w = (hmi_cmd_queue_index_w + 1) % HMI_BUFSIZE;
  hmi_commands_in_queue++;
}


/**
 * Check if the command came from HMI
 * return: true or false
 */
bool ok_to_HMI() {
  return Screen_send_ok[cmd_queue_index_r];
}


/**
 * Clear the Marlin command queue
 */
void clear_hmi_gcode_queue() {
  hmi_cmd_queue_index_r = hmi_cmd_queue_index_w = hmi_commands_in_queue = 0;
}


void ack_gcode_event(uint8_t event_id, uint32_t line) {
  SSTP_Event_t event = {event_id, SSTP_INVALID_OP_CODE};
  uint8_t buffer[4];

  event.length = 4;
  event.data = buffer;

  WORD_TO_PDU_BYTES(buffer, line);

  SNAP_DEBUG_SET_GCODE_LINE(line);

  hmi.Send(event);
}


static ErrCode HandleGcode(uint8_t *event_buff, uint16_t size) {
  event_buff[size] = 0;
  Screen_enqueue_and_echo_commands((char *)(event_buff + 5), INVALID_CMD_LINE, EID_GCODE_ACK);

  return E_SUCCESS;
}


static ErrCode HandleFileGcode(uint8_t *event_buff, uint16_t size) {
  uint32_t line;

  SysStatus   cur_sta = systemservice.GetCurrentStatus();
  WorkingPort port = systemservice.GetWorkingPort();

  if (port != WORKING_PORT_SC) {
    LOG_E("working port is not SC for file gcode!\n");
    return E_INVALID_STATE;
  }

  if (cur_sta != SYSTAT_WORK && cur_sta != SYSTAT_RESUME_WAITING) {
    LOG_E("not handle file Gcode in this status: %u\n", cur_sta);
    return E_INVALID_STATE;
  }

  // checkout the line number
  PDU_TO_LOCAL_WORD(line, event_buff+1);

  event_buff[size] = 0;

  if (line > systemservice.current_line() || systemservice.current_line() == 0) {
    systemservice.current_line(line);
    systemservice.hmi_cmd_timeout(millis());

    if (systemservice.is_waiting_gcode) {
      if (systemservice.is_laser_on) {
        systemservice.is_laser_on = false;
        laser.TurnOn();
      }
    }

    if (cur_sta == SYSTAT_RESUME_WAITING) {
      if (systemservice.ResumeOver() == E_SUCCESS) {
        LOG_I("cmd: %s\n\n", (char *)(event_buff + 5));
        Screen_enqueue_and_echo_commands((char *)(event_buff + 5), line, EID_FILE_GCODE_ACK);
      }
      else {
        ack_gcode_event(EID_FILE_GCODE_ACK, line);
      }
    }
    else {
      Screen_enqueue_and_echo_commands((char *)(event_buff + 5), line, EID_FILE_GCODE_ACK);
    }
  }
  else if (line == systemservice.current_line()) {
    if (line == debug.GetSCGcodeLine())
      ack_gcode_event(EID_FILE_GCODE_ACK, line);
  }
  else {
    LOG_E("recv line[%u] less than cur line[%u]\n", line, systemservice.current_line());
  }

  return E_SUCCESS;
}


static ErrCode SendStatus(SSTP_Event_t &event) {
  // won't send status to HMI while upgrading external module
  if (upgrade.GetState() != UPGRADE_STA_UPGRADING_EM)
    systemservice.SendStatus(event);
  return E_SUCCESS;
}


static ErrCode SendException(SSTP_Event_t &event) {
  return systemservice.SendException(event);
}


static ErrCode ChangeSystemStatus(SSTP_Event_t &event) {
  return systemservice.ChangeSystemStatus(event);
}


static ErrCode SendLastLine(SSTP_Event_t &event) {
  return systemservice.SendLastLine(event);
}


static ErrCode ClearException(SSTP_Event_t &event) {
  return systemservice.ClearException(event);
}


static ErrCode RecoverFromPowerLoss(SSTP_Event_t &event) {
  return systemservice.RecoverFromPowerLoss(event);
}


static ErrCode SendHomeAndCoordinateStatus(SSTP_Event_t &event) {
  return systemservice.SendHomeAndCoordinateStatus(event);
}


static ErrCode SetLogLevel(SSTP_Event_t &event) {
  return debug.SetLogLevel(event);
}

EventCallback_t sysctl_event_cb[SYSCTL_OPC_MAX] = {
  UNDEFINED_CALLBACK,
  /* [SYSCTL_OPC_GET_STATUES]        =  */{EVENT_ATTR_DEFAULT,      SendStatus},
  /* [SYSCTL_OPC_GET_EXCEPTION]      =  */{EVENT_ATTR_DEFAULT,      SendException},
  /* [SYSCTL_OPC_START_WORK]         =  */{EVENT_ATTR_DEFAULT,      ChangeSystemStatus},
  /* [SYSCTL_OPC_PAUSE]              =  */{EVENT_ATTR_ABORT_MOTION, ChangeSystemStatus},
  /* [SYSCTL_OPC_RESUME]             =  */{EVENT_ATTR_ABORT_MOTION, ChangeSystemStatus},
  /* [SYSCTL_OPC_STOP]               =  */{EVENT_ATTR_ABORT_MOTION, ChangeSystemStatus},
  /* [SYSCTL_OPC_FINISH]             =  */{EVENT_ATTR_HAVE_MOTION,  ChangeSystemStatus},
  /* [SYSCTL_OPC_GET_LAST_LINE]      =  */{EVENT_ATTR_DEFAULT,      SendLastLine},
  UNDEFINED_CALLBACK,
  /* [SYSCTL_OPC_CLEAR_FAULT]        =  */{EVENT_ATTR_DEFAULT,      ClearException},
  /* [SYSCTL_OPC_RECOVER_POWER_LOSS] =  */{EVENT_ATTR_HAVE_MOTION,  RecoverFromPowerLoss},
  UNDEFINED_CALLBACK,
  UNDEFINED_CALLBACK,
  /* [SYSCTL_OPC_GET_HOME_STATUS]    =  */{EVENT_ATTR_DEFAULT,      SendHomeAndCoordinateStatus},
  /* [SYSCTL_OPC_SET_LOG_LEVEL]      =  */{EVENT_ATTR_DEFAULT,      SetLogLevel},
  UNDEFINED_CALLBACK
};



static ErrCode DoAutoLeveling(SSTP_Event_t &event) {
  return levelservice.DoAutoLeveling(event);
}

static ErrCode DoManualLeveling(SSTP_Event_t &event) {
  return levelservice.DoManualLeveling(event);
}

static ErrCode SetManualLevelingPoint(SSTP_Event_t &event) {
  return levelservice.SetManualLevelingPoint(event);
}

static ErrCode AdjustZOffsetInLeveling(SSTP_Event_t &event) {
  return levelservice.AdjustZOffsetInLeveling(event);
}

static ErrCode SaveAndExitLeveling(SSTP_Event_t &event) {
  return levelservice.SaveAndExitLeveling(event);
}

static ErrCode ExitLeveling(SSTP_Event_t &event) {
  return levelservice.ExitLeveling(event);
}

static ErrCode GetFocalLength(SSTP_Event_t &event) {
  return laser.GetFocus(event);
}

static ErrCode SetFocalLength(SSTP_Event_t &event) {
  return laser.SetFocus(event);
}

static ErrCode DoManualFocusing(SSTP_Event_t &event) {
  return laser.DoManualFocusing(event);
}

static ErrCode DoAutoFocusing(SSTP_Event_t &event) {
  return laser.DoAutoFocusing(event);
}


static ErrCode ChangeRuntimeEnv(SSTP_Event_t &event) {
  return systemservice.ChangeRuntimeEnv(event);
}

static ErrCode GetRuntimeEnv(SSTP_Event_t &event) {
  return systemservice.GetRuntimeEnv(event);
}

static ErrCode GetMachineSize(SSTP_Event_t &event) {
  return systemservice.GetMachineSize(event);
}

EventCallback_t settings_event_cb[SETTINGS_OPC_MAX] = {
  UNDEFINED_CALLBACK,
  /* SETTINGS_OPC_SET_MACHINE_SIZE */ UNDEFINED_CALLBACK,
  /* [SETTINGS_OPC_DO_AUTO_LEVELING]       =  */{EVENT_ATTR_HAVE_MOTION,  DoAutoLeveling},
  /* SETTINGS_OPC_SYNC_LEVEL_POINT */ UNDEFINED_CALLBACK,
  /* [SETTINGS_OPC_DO_MANUAL_LEVELING]     =  */{EVENT_ATTR_HAVE_MOTION,  DoManualLeveling},
  /* [SETTINGS_OPC_SET_LEVELING_PONIT]     =  */{EVENT_ATTR_HAVE_MOTION,  SetManualLevelingPoint},
  /* [SETTINGS_OPC_ADJUST_Z_OFFSET]        =  */{EVENT_ATTR_HAVE_MOTION,  AdjustZOffsetInLeveling},
  /* [SETTINGS_OPC_SAVE_AND_EXIT_LEVELING] =  */{EVENT_ATTR_HAVE_MOTION,  SaveAndExitLeveling},
  /* [SETTINGS_OPC_EXIT_LEVELING]          =  */{EVENT_ATTR_HAVE_MOTION,  ExitLeveling},
  /* [SETTINGS_OPC_RESTORE_TO_FACTORY]     =  */ UNDEFINED_CALLBACK,
  /* [SETTINGS_OPC_READ_FOCAL_LENGTH]      =  */{EVENT_ATTR_DEFAULT,      GetFocalLength},
  /* [SETTINGS_OPC_SET_FOCAL_LENGTH]       =  */{EVENT_ATTR_DEFAULT,      SetFocalLength},
  /* [SETTINGS_OPC_DO_MANUAL_FOCUSING]     =  */{EVENT_ATTR_HAVE_MOTION,  DoManualFocusing},
  /* [SETTINGS_OPC_DO_AUTO_FOCUSING]       =  */{EVENT_ATTR_HAVE_MOTION,  DoAutoFocusing},
  /* [SETTINGS_OPC_DO_FAST_CALIBRATION]    =  */{EVENT_ATTR_HAVE_MOTION,  DoAutoLeveling},
  /* [SETTINGS_OPC_SET_RUNTIME_ENV]        =  */{EVENT_ATTR_HAVE_MOTION,  ChangeRuntimeEnv},
  /* [SETTINGS_OPC_GET_RUNTIME_ENV]        =  */{EVENT_ATTR_DEFAULT,      GetRuntimeEnv},
  UNDEFINED_CALLBACK,
  UNDEFINED_CALLBACK,
  UNDEFINED_CALLBACK,
  /* [SETTINGS_OPC_GET_MACHINE_SIZE]       =  */{EVENT_ATTR_HAVE_MOTION,  GetMachineSize}
};


static ErrCode DoXYZMove(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  float target_pos[XYZ];
  float speed = 0;

  if (event.length < 12) {
    LOG_E("Must specify target X, Y, Z in event\n");
    goto out;
  }

  PDU_TO_LOCAL_WORD(target_pos[X_AXIS], event.data);
  PDU_TO_LOCAL_WORD(target_pos[Y_AXIS], event.data+4);
  PDU_TO_LOCAL_WORD(target_pos[Z_AXIS], event.data+8);

  LOOP_XYZ(i) {
    target_pos[i] /= 1000;
  }

  if (event.length == 16) {
    PDU_TO_LOCAL_WORD(speed, event.data+12);
    speed /= 1000;
  }

  planner.synchronize();

  if (event.op_code == MOTION_OPC_DO_ABSOLUTE_MOVE) {
    move_to_limited_z(RAW_Z_POSITION(target_pos[Z_AXIS]), speed? speed : 10);
    move_to_limited_xy(RAW_X_POSITION(target_pos[X_AXIS]),
      RAW_Y_POSITION(target_pos[Y_AXIS]), speed? speed : 30);
  }
  else {
    // current_position[] is native position, so cannot use API 'do_blocking_move_to_logical_<axis>'
    // it only get logical position
    move_to_limited_z(current_position[Z_AXIS] + target_pos[Z_AXIS], speed? speed : 10);
    move_to_limited_xy(current_position[X_AXIS] + target_pos[X_AXIS],
      current_position[Y_AXIS] + target_pos[Y_AXIS], speed? speed : 30);
  }

  planner.synchronize();

  err = E_SUCCESS;

out:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


static ErrCode DoEMove(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  float extrude_len;
  float extrude_speed;
  float retract_len;
  float retract_speed;

  if (event.length < 17) {
    LOG_E("invalid parameter length: %u\n", event.length);
    goto out;
  }

  if (thermalManager.tooColdToExtrude(0)) {
    LOG_E("temperature is cool, cannot move E!\n");
    goto out;
  }

  PDU_TO_LOCAL_WORD(extrude_len, event.data+1);
  PDU_TO_LOCAL_WORD(extrude_speed, event.data+5);
  PDU_TO_LOCAL_WORD(retract_len, event.data+9);
  PDU_TO_LOCAL_WORD(retract_speed, event.data+13);

  extrude_len /= 1000;
  extrude_speed /= 60000;
  retract_len /=1000;
  retract_speed /= 60000;

  planner.synchronize();

  if (event.data[0] == 0) {
    // extrude firstly
    current_position[E_AXIS] += extrude_len;
    line_to_current_position(extrude_speed);
    planner.synchronize();
    current_position[E_AXIS] -= retract_len;
    line_to_current_position(retract_speed);
  }
  else {
    // retract firstly
    current_position[E_AXIS] -= retract_len;
    line_to_current_position(retract_speed);
    planner.synchronize();
    current_position[E_AXIS] += extrude_len;
    line_to_current_position(extrude_speed);
  }

  planner.synchronize();

  err = E_SUCCESS;

out:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

EventCallback_t motion_event_cb[MOTION_OPC_MAX] = {
  UNDEFINED_CALLBACK,
  UNDEFINED_CALLBACK,
  /* [MOTION_OPC_DO_ABSOLUTE_MOVE]     =  */{EVENT_ATTR_HAVE_MOTION,  DoXYZMove},
  /* [MOTION_OPC_DO_RELATIVE_MOVE]     =  */{EVENT_ATTR_HAVE_MOTION,  DoXYZMove},
  /* [MOTION_OPC_DO_E_MOVE]            =  */{EVENT_ATTR_HAVE_MOTION,  DoEMove}
};


static ErrCode SetCameraBtName(SSTP_Event_t &event) {
  return laser.SetCameraBtName(event);
}

static ErrCode GetCameraBtName(SSTP_Event_t &event) {
  return laser.GetCameraBtName(event);
}

static ErrCode GetCameraBtMAC(SSTP_Event_t &event) {
  return laser.GetCameraBtMAC(event);
}

EventCallback_t camera_event_cb[CAMERA_OPC_MAX] = {
  UNDEFINED_CALLBACK,
  UNDEFINED_CALLBACK,
  UNDEFINED_CALLBACK,
  UNDEFINED_CALLBACK,
  UNDEFINED_CALLBACK,
  /* [CAMERA_OPC_SET_BT_NAME]          =  */{EVENT_ATTR_DEFAULT,  SetCameraBtName},
  /* [CAMERA_OPC_READ_BT_NAME]         =  */{EVENT_ATTR_DEFAULT,  GetCameraBtName},
  /* [CAMERA_OPC_READ_BT_MAC]          =  */{EVENT_ATTR_DEFAULT,  GetCameraBtMAC}
};



// implement follow 4 function after rebase chamber branch
static ErrCode ReportEnclosureStatus(SSTP_Event_t &event) {
  return enclosure.ReportStatus(event);
}


static ErrCode SetEnclosureLight(SSTP_Event_t &event) {
  return enclosure.SetLightBar(event);
}


static ErrCode SetEnclosureFan(SSTP_Event_t &event) {
  return enclosure.SetFan(event);
}


static ErrCode SetEnclosureDetection(SSTP_Event_t &event) {
  return enclosure.SetDetection(event);
}

static ErrCode GetAddonList(SSTP_Event_t &event) {
  return E_SUCCESS;
}

static ErrCode GetAddonInfo(SSTP_Event_t &event) {
  return E_SUCCESS;
}

static ErrCode GetAddonStopStatus(SSTP_Event_t &event) {
  return emergency_stop.ReportStatus();
}

static ErrCode GetRotateStatus(SSTP_Event_t &event) {
  uint8_t state = rotaryModule.status();
  LOG_I("SC req rotate sta:%d\n", state);
  uint8_t buff[1];

  buff[0] = state;
  event.length = 1;
  event.data = buff;

  return hmi.Send(event);
}

static ErrCode GetPurifierStatus(SSTP_Event_t &event) {
  purifier.GetInfo(PURIFIER_INFO_ERR, 500);
  return purifier.ReportStatus();
}

static ErrCode GetPurifierFanStatus(SSTP_Event_t &event) {
  PurifierInfo_t info =purifier.GetInfo(PURIFIER_INFO_FAN_STA, 500);
  uint8_t buff[2];

  buff[0] = info.is_work;
  buff[1] = info.fan_gears;
  event.length = 2;
  event.data = buff;
  LOG_I("SC req purifier fan sta:%d, gears%d\n", buff[0], buff[1]);

  return hmi.Send(event);
}

static ErrCode SetPurifierFanStatus(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  uint16_t delay_close_s = 0;
  if (event.length < 1) {
    LOG_E("must specify fan status!\n");
    err = E_FAILURE;
    goto OUT;
  } else if (event.length == 3) {
    delay_close_s = (event.data[1] << 8) | event.data[2];
  }
  purifier.SetFanStatus(event.data[0], delay_close_s);

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

static ErrCode GetPurifierTimelifeStatus(SSTP_Event_t &event) {
  purifier.GetInfo(PURIFIER_INFO_LIFETIME, 500);
  return purifier.ReportLifetimeStatus();
}

static ErrCode SetPurifierGearsStatus(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  if (event.length < 1) {
    LOG_E("must specify fan gears!\n");
    err = E_FAILURE;
    goto OUT;
  }
  purifier.SetFanGears(event.data[0]);

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

EventCallback_t addon_event_cb[ADDON_OPC_MAX] = {
  UNDEFINED_CALLBACK,
  /* [ADDON_OPC_GET_CHAMBER_STATUS]    =  */{EVENT_ATTR_DEFAULT,  ReportEnclosureStatus},
  /* [ADDON_OPC_SET_CHAMBER_LIGHT]     =  */{EVENT_ATTR_DEFAULT,  SetEnclosureLight},
  /* [ADDON_OPC_SET_CHAMBER_FAN]       =  */{EVENT_ATTR_DEFAULT,  SetEnclosureFan},
  /* [ADDON_OPC_SET_CHAMBER_DETECTION] =  */{EVENT_ATTR_DEFAULT,  SetEnclosureDetection},
  /* [ADDON_OPC_GET_ADDON_LIST]        =  */{EVENT_ATTR_DEFAULT,  GetAddonList},
  /* [ADDON_OPC_GET_ADDON_INFO]        =  */{EVENT_ATTR_DEFAULT,  GetAddonInfo},
  /* [ADDON_OPC_GET_ADDON_STOP]        =  */{EVENT_ATTR_DEFAULT,  GetAddonStopStatus},
  /* [ADDON_OPC_GET_ROTATE_STATE]        =  */{EVENT_ATTR_DEFAULT,  GetRotateStatus},
  /* [ADDON_OPC_GET_PURIFIER_STATE]          = */{EVENT_ATTR_DEFAULT, GetPurifierStatus},
  /* [ADDON_OPC_GET_PURIFIER_FAN_STATE]      = */{EVENT_ATTR_DEFAULT, GetPurifierFanStatus},        
  /* [ADDON_OPC_SET_PURIFIER_FAN_STATE]      = */{EVENT_ATTR_DEFAULT, SetPurifierFanStatus},        
  /* [ADDON_OPC_SET_PURIFIER_GEARS_STATE]    = */{EVENT_ATTR_DEFAULT, SetPurifierGearsStatus},      
  /* [ADDON_OPC_GET_PURIFIER_TIMELIFE_STATE] = */{EVENT_ATTR_DEFAULT, GetPurifierTimelifeStatus},
};


static ErrCode SetModuleMAC(SSTP_Event_t &event) {
  return ModuleBase::SetMAC(event);
}


static ErrCode GetModuleMAC(SSTP_Event_t &event) {
  return ModuleBase::GetMAC(event);
}

static ErrCode SetLinearModuleLength(SSTP_Event_t &event) {
  return linear_p->SetLength(event);
}

static ErrCode GetLinearModuleLength(SSTP_Event_t &event) {
  return linear_p->GetLength(event);
}

static ErrCode SetLinearModuleLead(SSTP_Event_t &event) {
  return linear_p->SetLead(event);
}

static ErrCode GetLinearModuleLead(SSTP_Event_t &event) {
  return linear_p->GetLead(event);
}

EventCallback_t debug_event_cb[DEBUG_OPC_MAX] = {
  UNDEFINED_CALLBACK,
  /* [DEBUG_OPC_SET_MODULE_MAC]        =  */{EVENT_ATTR_DEFAULT,  SetModuleMAC},
  /* [DEBUG_OPC_GET_MODULE_MAC]        =  */{EVENT_ATTR_DEFAULT,  GetModuleMAC},
  /* [DEBUG_OPC_SET_LINEAR_LENGTH]     =  */{EVENT_ATTR_DEFAULT,  SetLinearModuleLength},
  /* [DEBUG_OPC_GET_LINEAR_LENGTH]     =  */{EVENT_ATTR_DEFAULT,  GetLinearModuleLength},
  /* [DEBUG_OPC_SET_LINEAR_LEAD]       =  */{EVENT_ATTR_DEFAULT,  SetLinearModuleLead},
  /* [DEBUG_OPC_GET_LINEAR_LEAD]       =  */{EVENT_ATTR_DEFAULT,  GetLinearModuleLead}
};


static ErrCode StartUpgrade(SSTP_Event_t &event) {
  return upgrade.StartUpgrade(event);
}

static ErrCode ReceiveFW(SSTP_Event_t &event) {
  return upgrade.ReceiveFW(event);
}

static ErrCode EndUpgarde(SSTP_Event_t &event) {
  return upgrade.EndUpgarde(event);
}

static ErrCode GetMainControllerVer(SSTP_Event_t &event) {
  return upgrade.GetMainControllerVer(event);
}

static ErrCode CompareMCVer(SSTP_Event_t &event) {
  return upgrade.CompareMCVer(event);
}

static ErrCode GetUpgradeStatus(SSTP_Event_t &event) {
  return upgrade.GetUpgradeStatus(event);
}

static ErrCode GetModuleVer(SSTP_Event_t &event) {
  return upgrade.GetModuleVer(event);
}

EventCallback_t upgrade_event_cb[UPGRADE_OPC_MAX] = {
  /* [UPGRADE_OPC_START]               =  */{EVENT_ATTR_DEFAULT,  StartUpgrade},
  /* [UPGRADE_OPC_TRANS_FW]            =  */{EVENT_ATTR_DEFAULT,  ReceiveFW},
  /* [UPGRADE_OPC_END]                 =  */{EVENT_ATTR_DEFAULT,  EndUpgarde},
  /* [UPGRADE_OPC_GET_MC_VER]          =  */{EVENT_ATTR_DEFAULT,  GetMainControllerVer},
  /* [UPGRADE_OPC_COMPARE_VER]         =  */{EVENT_ATTR_DEFAULT,  CompareMCVer},
  /* [UPGRADE_OPC_GET_UP_STATUS]       =  */{EVENT_ATTR_DEFAULT,  GetUpgradeStatus},
  /* [UPGRADE_OPC_SYNC_MODULE_UP_STATUS] =*/UNDEFINED_CALLBACK,
  /* [UPGRADE_OPC_GET_MODULE_VER]      =  */{EVENT_ATTR_DEFAULT,  GetModuleVer}
};


// need to known which task we running with
// then we won't send out event again
ErrCode DispatchEvent(DispatcherParam_t param) {
  ErrCode      err   = E_INVALID_CMD;
  SSTP_Event_t event = {SSTP_INVALID_EVENT_ID, SSTP_INVALID_OP_CODE};

  EventCallback_t *callbacks  = NULL;
  uint16_t        op_code_max = SSTP_INVALID_OP_CODE;

  bool send_to_marlin = false;
#if DEBUG_EVENT_HANDLER
  bool heartbeat = false;
#endif

  // if we are running in Marlin task, need to get command from the queue
  if (param->owner == TASK_OWN_MARLIN) {
    if (xMessageBufferIsEmpty(param->event_queue))
      return E_NO_RESRC;

    param->size = (uint16_t)xMessageBufferReceive(param->event_queue, param->event_buff,
                              SSTP_RECV_BUFFER_SIZE, 0);

    if (!param->size) {
      LOG_E("No data read from event queue!\n");
      return E_NO_RESRC;
    }
  }

  event.id = param->event_buff[EVENT_IDX_EVENT_ID];

  switch (event.id) {
  case EID_GCODE_REQ:
    if (param->owner == TASK_OWN_MARLIN) {
      return HandleGcode(param->event_buff, param->size);
    }
    else
      send_to_marlin = true;
    break;

  case EID_FILE_GCODE_REQ:
    if (param->owner == TASK_OWN_MARLIN) {
      return HandleFileGcode(param->event_buff, param->size);
    }
    else
      send_to_marlin = true;
    break;

  case EID_SYS_CTRL_REQ:
    callbacks = sysctl_event_cb;
#if DEBUG_EVENT_HANDLER
    if (param->event_buff[EVENT_IDX_OP_CODE] == 1)
      heartbeat = true;
#endif
    op_code_max = SYSCTL_OPC_MAX;
    break;

  case EID_SETTING_REQ:
    callbacks = settings_event_cb;
    op_code_max = SETTINGS_OPC_MAX;
    break;

  case EID_MOTION_REQ:
    callbacks = motion_event_cb;
    op_code_max = MOTION_OPC_MAX;
    break;

  case EID_CAMERA_REQ:
    callbacks = camera_event_cb;
    op_code_max = CAMERA_OPC_MAX;
    break;

  case EID_ADDON_REQ:
    callbacks = addon_event_cb;
    op_code_max = ADDON_OPC_MAX;
    break;

  case EID_DEBUG_REQ:
    callbacks = debug_event_cb;
    op_code_max = DEBUG_OPC_MAX;
    break;

  case EID_UPGRADE_REQ:
    callbacks = upgrade_event_cb;
    op_code_max = UPGRADE_OPC_MAX;
    break;

  default:
    break;
  }

  // well, per the event id, we know the event need to be handle by Marlin task
  // actually, there is only the Gcode event can go into it
  if (send_to_marlin) {
#if DEBUG_EVENT_HANDLER
    SERIAL_ECHOLNPAIR("new gcode, eid: ", event.id);
#endif
    // blocked 100ms for max duration to wait
    if (quickstop.isTriggered())
      LOG_I("Got G[%u] in QS\n", event.id);
    xMessageBufferSend(param->event_queue, param->event_buff, param->size, configTICK_RATE_HZ/10);
    return E_SUCCESS;
  }

  // increase the event id to get ACK id for event callback
  // or for return error to Screen
  event.id++;

  if (param->size < 2) {
    LOG_E("invalid event [0x%X], no operation code field\n", param->event_buff[EVENT_IDX_EVENT_ID]);
    goto out_err;
  }

  // if no callback or invalid op code for this event, should ack error to Screen
  if (!callbacks || (op_code_max == SSTP_INVALID_OP_CODE)) {
    LOG_E("didn't found callback for event [0x%X : 0x%X], size: [%u]\n",
      param->event_buff[EVENT_IDX_EVENT_ID], param->event_buff[EVENT_IDX_OP_CODE], param->size);
    goto out_err;
  }

  // if no callback or invalid op code for this event, should ack error to Screen
  if (param->event_buff[EVENT_IDX_OP_CODE] >= op_code_max) {
    LOG_E("invalid event [0x%X : 0x%X], opc is out of range (%d)\n",
      param->event_buff[EVENT_IDX_EVENT_ID], param->event_buff[EVENT_IDX_OP_CODE], op_code_max);
    goto out_err;
  }

  event.op_code = param->event_buff[EVENT_IDX_OP_CODE];

  // found relative callback group, but didn't add cabllback function for this operation code
  if (!callbacks[event.op_code].cb) {
    LOG_E("event[0x%X : 0x%X] doesn't have callback\n",
      param->event_buff[EVENT_IDX_EVENT_ID], param->event_buff[EVENT_IDX_OP_CODE]);
    goto out_err;
  }

  if ((callbacks[event.op_code].attr & EVENT_HANDLE_WITH_MARLIN) && (param->owner != TASK_OWN_MARLIN)) {
    // arrive here, we know the event need to be sent to Marlin by check event id & op code
#if DEBUG_EVENT_HANDLER
    SERIAL_ECHOLNPAIR("Marlin's event, id: ", hex_byte((uint8_t)event.id), ", opc: ", hex_byte((uint8_t)event.op_code));
#endif
    if (quickstop.isTriggered())
      LOG_I("Got E[%x:%x] in QS\n", event.id, event.op_code);
    xMessageBufferSend(param->event_queue, param->event_buff, param->size, configTICK_RATE_HZ/10);
    return E_SUCCESS;
  }
#if DEBUG_EVENT_HANDLER
  else {
  if (!heartbeat)
    SERIAL_ECHOLNPAIR("HMI's event, id: ", hex_byte((uint8_t)event.id), ", opc: ", hex_byte((uint8_t)event.op_code));
  }
#endif

  event.length = param->size - 2;
  event.data = param->event_buff + 2;
  return callbacks[event.op_code].cb(event);

out_err:
  event.data = &err;
  event.length = 1;
  hmi.Send(event);
  return err;
}

