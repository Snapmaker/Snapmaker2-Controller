
#include "snap_dbg.h"
#include "../module/StatusControl.h"

#if (SNAP_DEBUG == 1)

SnapDebug debug;

#if defined (__GNUC__)                /* GNU GCC Compiler */
  /* the version of GNU GCC must be greater than 4.x */
  typedef __builtin_va_list       __gnuc_va_list;
  typedef __gnuc_va_list          va_list;
  #define va_start(v,l)           __builtin_va_start(v,l)
  #define va_end(v)               __builtin_va_end(v)
  #define va_arg(v,l)             __builtin_va_arg(v,l)
#else
  #error "Snap debug only support GNU compiler"
#endif

static SnapDebugLevel msg_level = SNAP_DEBUG_LEVEL_INFO;

const char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX] = {
  SNAP_INFO_STR,
  SNAP_WARNING_STR,
  SNAP_ERROR_STR,
  SNAP_FATAL_STR
};


// output debug message, will not output message whose level
// is less than msg_level
// param:
//    lvl - message level
//    fmt - format of messages
//    ... - args
void SnapDebug::Log(SnapDebugLevel level, const char *fmt, ...) {
  int len;
  va_list args;
  char log_buf[SMAP_LOG_BUFFER_SIZE];

  if (level < msg_level)
    return;

  if (level >= SNAP_DEBUG_LEVEL_MAX) {
    level = (SnapDebugLevel)(SNAP_DEBUG_LEVEL_MAX - 1);
  }

  strcpy(log_buf, snap_debug_str[level]);

  len = strlen(log_buf);

  va_start(args, fmt);

  vsnprintf(log_buf + len, SMAP_LOG_BUFFER_SIZE - len - 1, fmt, args);

  CONSOLE_OUTPUT(log_buf, strlen(log_buf) + 1);

  va_end(args);
}


// set current debug level message level less than this level
// will not be outputed, set by M2000
void SnapDebug::SetLevel(SnapDebugLevel l) {
  Log(SNAP_DEBUG_LEVEL_INFO, "old debug level: %d\n", msg_level);
  if (l <= SNAP_DEBUG_LEVEL_MAX)
    msg_level = l;
}


// gcode state of Screen
void SnapDebug::SetGcodeState(GcodeState s) {
  info.sc_gcode_state = s;
}

void SnapDebug::SetSCGcodeLine(uint32_t l) {
  info.last_line_num_of_sc_gcode = l;
}

void SnapDebug::CmdChecksumError(bool screen) {
  if (screen)
    info.screen_cmd_checksum_err++;
  else
    info.pc_cmd_checksum_err++;
}

// show system debug info
void SnapDebug::ShowInfo() {
  Log(SNAP_DEBUG_LEVEL_INFO, "current status: %d\n", SystemStatus.GetCurrentPrinterStatus());
  Log(SNAP_DEBUG_LEVEL_INFO, "SC gcode state: %d\n", info.sc_gcode_state);
  Log(SNAP_DEBUG_LEVEL_INFO, "PC gcode state: %d\n", info.pc_gcode_state);
  Log(SNAP_DEBUG_LEVEL_INFO, "SC cmd chksum error count: %u\n", info.screen_cmd_checksum_err);
  Log(SNAP_DEBUG_LEVEL_INFO, "PC cmd chksum error count: %u\n", info.pc_cmd_checksum_err);
  Log(SNAP_DEBUG_LEVEL_INFO, "Last SC Gcode line: %d\n", info.last_line_num_of_sc_gcode);
}

#endif // #if (SNAP_DEBUG == 1)
