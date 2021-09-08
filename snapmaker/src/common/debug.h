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
#ifndef SNAPMAKER_DEBUG_H_
#define SNAPMAKER_DEBUG_H_

#include <stdio.h>
#include "config.h"

#include "src/core/macros.h"

#include "error.h"
#include "../hmi/event_handler.h"

// 1 = enable API for snap debug
#define SNAP_DEBUG 1

enum SnapDebugLevel : uint8_t {
  SNAP_DEBUG_LEVEL_TRACE = 0,
  SNAP_DEBUG_LEVEL_VERBOSE,
  SNAP_DEBUG_LEVEL_INFO,
  SNAP_DEBUG_LEVEL_WARNING,
  SNAP_DEBUG_LEVEL_ERROR,
  SNAP_DEBUG_LEVEL_FATAL,
  SNAP_DEBUG_LEVEL_MAX
};

#define SNAP_DEBUG_LEVEL_DEFAULT SNAP_DEBUG_LEVEL_INFO

// state for Gcode command
enum GcodeState : uint8_t {
  GCODE_STATE_RECEIVED,
  GCODE_STATE_CHK_ERR,
  GCODE_STATE_BUFFERED,
  GCODE_STATE_ACKED,
  GCODE_STATE_INVALID
};

#if (SNAP_DEBUG)

// massage will output to this interface
#define CONSOLE_OUTPUT(log) MYSERIAL0.print_directly(log)

// log buffer size, max length for one debug massage
#define SNAP_LOG_BUFFER_SIZE 256

#define SNAP_TRACE_STR    "SNAP_TRACE: "
#define SNAP_INFO_STR     "SNAP_INFO: "
#define SNAP_WARNING_STR  "SNAP_WARN: "
#define SNAP_ERROR_STR    "SNAP_ERR: "
#define SNAP_FATAL_STR    "SANP_FATAL: "

// information structure, anyone can add parameter
// 'M2000 S0' will show this info
struct SnapDebugInfo {
  GcodeState sc_gcode_state;          // state for gcode from screen
  GcodeState pc_gcode_state;          // state for gcode from PC

  uint32_t  screen_cmd_checksum_err;  // chceksum error for command from screen
  uint32_t  pc_cmd_checksum_err;      // chceksum error for command from screen

  uint32_t last_line_num_of_sc_gcode; // line number of last gcode acked to screen
};

class SnapDebug {
  public:
    void Log(SnapDebugLevel level, const char *fmt, ...);

    void ShowInfo();
    void SetLevel(uint8_t port, SnapDebugLevel l);
    SnapDebugLevel GetLevel();
    void CmdChecksumError(bool screen);
    void SetSCGcodeLine(uint32_t l);
    uint32_t GetSCGcodeLine() { return info.last_line_num_of_sc_gcode; }

    void ShowException();

    ErrCode SetLogLevel(SSTP_Event_t &event);

  private:
    void SendLog2Screen(SnapDebugLevel l);

    struct SnapDebugInfo info;
};

// interface for external use
// when SNAP_DEBUG is not defined, API is NONE

extern SnapDebug debug;

#define LOG_F(...) debug.Log(SNAP_DEBUG_LEVEL_FATAL, __VA_ARGS__)
#define LOG_E(...) debug.Log(SNAP_DEBUG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_W(...) debug.Log(SNAP_DEBUG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_I(...) debug.Log(SNAP_DEBUG_LEVEL_INFO, __VA_ARGS__)
#define LOG_V(...) debug.Log(SNAP_DEBUG_LEVEL_VERBOSE, __VA_ARGS__)
#define LOG_T(...) debug.Log(SNAP_DEBUG_LEVEL_TRACE, __VA_ARGS__)


#define SNAP_DEBUG_SHOW_INFO()            debug.ShowInfo();
#define SNAP_DEBUG_SHOW_EXCEPTION()       debug.ShowException();
#define SNAP_DEBUG_SET_LEVEL(p, l)        debug.SetLevel(p, l);
#define SNAP_DEBUG_CMD_CHECKSUM_ERROR(s)  debug.CmdChecksumError(s);
#define SNAP_DEBUG_SET_GCODE_LINE(l)      debug.SetSCGcodeLine(l);

#else

#define LOG_F(...)
#define LOG_E(...)
#define LOG_W(...)
#define LOG_I(...)
#define LOG_V(...)
#define LOG_T(...)

#define SNAP_DEBUG_SHOW_INFO()
#define SNAP_DEBUG_SHOW_EXCEPTION()
#define SNAP_DEBUG_SET_LEVEL(l)
#define SNAP_DEBUG_CMD_CHECKSUM_ERROR(s)
#define SNAP_DEBUG_SET_GCODE_LINE(l)

#endif // #ifdef SNAP_DEBUG

#endif  // #ifndef SNAPMAKER_DEBUG_H_
