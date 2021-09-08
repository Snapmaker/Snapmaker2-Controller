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

#include "debug.h"
#include "../service/system.h"
#include "../service/power_loss_recovery.h"
#include "../snapmaker.h"

// marlin headers
#include "src/Marlin.h"
#include "src/gcode/gcode.h"
#include "src/module/motion.h"
#include "src/core/minmax.h"

#if (SNAP_DEBUG == 1)

SnapDebug debug;

const static char *excoption_str[32] {
  "Didn't detect Executor!",
  "Didn't detect Linear Module!",
  "Port of Heated Bed is bad!",
  "Filemant has ran out!",
  "Lost settings!",
  "Lost Executor!",
  "Power loss happened!",
  "Hotend heating failed!",
  "Bed heating failed!",
  "Temperature runaway of Hotend!",
  "Temperature runaway of Bed!",
  "Thermistor of Hotend is Bad!",
  "Thermistor of Bed is Bad!",
  "Lost Linear Module!",
  "Temperature of Hotend is over Max Limit!",
  "Temperature of Bed is over Max Limit!",
  "Short circuit maybe appear in Heating tube of Hotend!",
  "Short circuit maybe appear in Heating tube of Bed!",
  "Thermistor of Hotend maybe come off!",
  "Thermistor of Bed maybe come off!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "Unknown Excption!"
};

#if defined (__GNUC__)                /* GNU GCC Compiler */
  /* the version of GNU GCC must be greater than 4.x */
  typedef __builtin_va_list       __gnuc_va_list;
  typedef __gnuc_va_list          va_list;
  #define va_start(v,l)           __builtin_va_start(v,l)
  #define va_end(v)               __builtin_va_end(v)
  #define va_arg(v,l)             __builtin_va_arg(v,l)
#else
  #error "Snap debug only support GNU compiler for now"
#endif

static SnapDebugLevel pc_msg_level = SNAP_DEBUG_LEVEL_INFO;
static SnapDebugLevel sc_msg_level = SNAP_DEBUG_LEVEL_INFO;
static char log_buf[SNAP_LOG_BUFFER_SIZE + 2];

const char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX] = {
  SNAP_TRACE_STR,
  SNAP_INFO_STR,
  SNAP_WARNING_STR,
  SNAP_ERROR_STR,
  SNAP_FATAL_STR
};


void SnapDebug::SendLog2Screen(SnapDebugLevel l) {
  SSTP_Event_t event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_TRANS_LOG};

  int size = strlen(log_buf+2);

  if (size == 0)
    return;
  else if (size >= 255) {
    size = 255;
    log_buf[255 + 2] = '\0';
  }

  // to include the end '\0'
  size++;

  log_buf[0] = l;
  log_buf[1] = size;

  event.length = size + 2;
  event.data = (uint8_t *)log_buf;

  hmi.Send(event);
}

// output debug message, will not output message whose level
// is less than msg_level
// param:
//    level - message level
//    fmt - format of messages
//    ... - args
void SnapDebug::Log(SnapDebugLevel level, const char *fmt, ...) {
  va_list args;

  if (level < pc_msg_level && level < sc_msg_level)
    return;

  va_start(args, fmt);

  vsnprintf(log_buf + 2, SNAP_LOG_BUFFER_SIZE, fmt, args);

  va_end(args);

  if (level >= pc_msg_level)
    CONSOLE_OUTPUT(log_buf + 2);

  if (level >= sc_msg_level)
    SendLog2Screen(level);
}


// set current debug level message level less than this level
// will not be outputed, set by M2000
void SnapDebug::SetLevel(uint8_t port, SnapDebugLevel l) {
  Log(SNAP_DEBUG_LEVEL_INFO, "old debug level: %d\n",
                            port? sc_msg_level : pc_msg_level);

  if (l > SNAP_DEBUG_LEVEL_MAX)
    return;

  if (port) {
    sc_msg_level = l;
  }
  else {
    pc_msg_level = l;
  }
}


ErrCode SnapDebug::SetLogLevel(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  if (event.length != 1) {
    LOG_E("Need to specify log level!\n");
    event.data = &err;
    event.length = 1;
  }
  else {
    // LOG_V("SC req change log level");
    sc_msg_level = (SnapDebugLevel)event.data[0];
    event.data[0] = E_SUCCESS;
  }

  return hmi.Send(event);
}


SnapDebugLevel SnapDebug::GetLevel() {
  return sc_msg_level < pc_msg_level? sc_msg_level : pc_msg_level;
}

// record the line number of last Gcode from screen
void SnapDebug::SetSCGcodeLine(uint32_t l) {
  info.last_line_num_of_sc_gcode = l;
}

// error count of uncorrect checksum of commands from screen
void SnapDebug::CmdChecksumError(bool screen) {
  if (screen)
    info.screen_cmd_checksum_err++;
  else
    info.pc_cmd_checksum_err++;
}

// show system debug info
void SnapDebug::ShowInfo() {
  char tmp_buf[100];

  SERIAL_ECHOPAIR("systat: ", systemservice.GetCurrentStatus(), "\n");
  SERIAL_ECHOPAIR("SC checksum error: ", info.screen_cmd_checksum_err, "\n");
  SERIAL_ECHOPAIR("Last recv line: ", systemservice.current_line(), "\n");
  SERIAL_ECHOPAIR("Last ack line: ", info.last_line_num_of_sc_gcode, "\n");
  SERIAL_ECHOPAIR("Last st line: ", pl_recovery.LastLine(), "\n");
  sprintf(tmp_buf, "Fault: 0x%08X, action ban: 0x%X, power ban: 0x%X\n",
        (int)systemservice.GetFaultFlag(), (int)action_ban, (int)power_ban);
  SERIAL_ECHOPAIR(tmp_buf);
  sprintf(tmp_buf, "Homing: 0x%X, axes_known: 0x%X\n", axis_homed, axis_known_position);
  SERIAL_ECHOPAIR(tmp_buf);
  SERIAL_ECHOPAIR("active coordinate: ", gcode.active_coordinate_system, "\n");
  SERIAL_ECHOPAIR("coordinate 1: X: ", gcode.coordinate_system[0][X_AXIS], "Y: ", gcode.coordinate_system[0][Y_AXIS], "Z: ", gcode.coordinate_system[0][Z_AXIS], "B: ", gcode.coordinate_system[0][B_AXIS], "\n");
}

void SnapDebug::ShowException() {
  uint8_t i;
  uint32_t fault_flag = systemservice.GetFaultFlag();

  if (!fault_flag) {
    Log(SNAP_DEBUG_LEVEL_INFO, "No excption happened!\n");
    return;
  }
  else
    Log(SNAP_DEBUG_LEVEL_INFO, "Excption info:\n");

  for (i=0; i<32; i++) {
    if (fault_flag & (0x00000001<<i))
      Log(SNAP_DEBUG_LEVEL_INFO, "%s\n", excoption_str[i]);
  }
}

#endif // #if (SNAP_DEBUG == 1)
