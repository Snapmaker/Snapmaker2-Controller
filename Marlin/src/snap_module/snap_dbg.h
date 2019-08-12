#ifndef SNAP_DBG_H_
#define SNAP_DBG_H_

#include <stdio.h>
#include "../core/macros.h"

// 1 = enable API for snap debug
#define SNAP_DEBUG 1

// massage will output to this interface
#define CONSOLE_OUTPUT(log, length) MYSERIAL0.print(log)

// log buffer size, max length for one debug massage
#define SMAP_LOG_BUFFER_SIZE 128

#define SNAP_INFO_STR     "SNAP_INFO: "
#define SNAP_WARNING_STR  "SNAP_WARN: "
#define SNAP_ERROR_STR    "SNAP_ERR: "
#define SNAP_FATAL_STR    "SANP_FATAL: "

enum SnapDbgLevel : uint8_t {
  SNAP_INFO = 0,
  SNAP_WARNING,
  SNAP_ERROR,
  SNAP_FATAL,
  SNAP_DEBUG_LEVEL_MAX
};

#define SNAP_DBG_DEFAULT_LEVEL SNAP_INFO

// state for Gcode command
enum GcodeState : uint8_t {
  GCODE_STATE_RECEIVED,
  GCODE_STATE_CHK_ERR,
  GCODE_STATE_BUFFERED,
  GCODE_STATE_ACKED,
  GCODE_STATE_INVALID
};

// interface for external use
// when SNAP_DEBUG is not defined, API is NONE
#if (SNAP_DEBUG)

extern "C" void SetGcodeState(GcodeState s);
extern "C" void ShowDebugInfo();
extern "C" void SnapDbg(SnapDbgLevel lvl, const char *fmt, ...);
extern "C" void SetDbgLevel(SnapDbgLevel l);
extern "C" void CmdChecksumError(bool screen);
extern "C" void SetSCGcodeLine(uint32_t l);
#else

#define SnapDbg(lvl, fmt, ...)
#define SetGcodeState(s)
#define ShowDebugInfo()
#define SetDbgLevel(l)
#define CmdChecksumError(sc)
#define SetSCGcodeLine(l)

#endif // #ifdef SNAP_DEBUG

#endif  // #ifndef SNAP_DBG_H_
