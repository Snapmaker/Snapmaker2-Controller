
#include "snap_dbg.h"
#include "../module/StatusControl.h"

#if (SNAP_DEBUG == 1)

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

static SnapDbgLevel msg_level = SNAP_DBG_DEFAULT_LEVEL;

char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX] = {
  SNAP_INFO_STR,
  SNAP_WARNING_STR,
  SNAP_ERROR_STR,
  SNAP_FATAL_STR
};


// information structure, anyone can add parameter
// 'M2000 S0' will show this info
struct SnapDebugInfo {
  GcodeState sc_gcode_state;  // state for gcode from screen
  GcodeState pc_gcode_state;  // state for gcode from PC

  uint32_t  sc_cmd_chk_error; // chceksum error for command from screen
  uint32_t  pc_cmd_chk_error; // chceksum error for command from screen

  uint32_t last_ack_sc_gcode; // line number of last gcode acked to screen
};

static struct SnapDebugInfo info;

// output debug message, will not output message whose level
// is less than msg_level
// param:
//    lvl - message level
//    fmt - format of messages
//    ... - args
void SnapDbg(SnapDbgLevel lvl, const char *fmt, ...) {
  int len;
  va_list args;
  char log_buf[SMAP_LOG_BUFFER_SIZE];

  if (lvl < msg_level)
    return;

  if (lvl >= SNAP_DEBUG_LEVEL_MAX) {
    lvl = (SnapDbgLevel)(SNAP_DEBUG_LEVEL_MAX - 1);
  }

  strcpy(log_buf, snap_debug_str[lvl]);

  va_start(args, fmt);

  len =  strlen(snap_debug_str[lvl]);

  vsnprintf(log_buf + len, SMAP_LOG_BUFFER_SIZE - len - 1, fmt, args);

  len = strlen(log_buf);

  // add new line if available length of buffer permitted
  if ((len < SMAP_LOG_BUFFER_SIZE - 2) && (log_buf[len - 1 ] != '\n' )) {
    log_buf[len] = '\n';
    log_buf[++len] = '\0';
  }

  CONSOLE_OUTPUT(log_buf, len);

  va_end(args);
}


// set current debug level message level less than this level
// will not be outputed, set by M2000
void SetDbgLevel(SnapDbgLevel l) {
  SnapDbg(SNAP_INFO, "old debug level: %d\n", msg_level);
  if (l <= SNAP_DEBUG_LEVEL_MAX)
   msg_level = l;
}


// gcode state of Screen
void SetGcodeState(GcodeState s) {
  info.sc_gcode_state = s;
}

void SetSCGcodeLine(uint32_t l) {
  info.last_ack_sc_gcode = l;
}

void CmdChecksumError(bool screen) {
  if (screen)
    info.sc_cmd_chk_error++;
  else
    info.pc_cmd_chk_error++;
}

// show system debug info
void ShowDebugInfo() {
  SnapDbg(SNAP_INFO, "current status: %d", SystemStatus.GetCurrentPrinterStatus());
  SnapDbg(SNAP_INFO, "SC gcode state: %d", info.sc_gcode_state);
  SnapDbg(SNAP_INFO, "PC gcode state: %d", info.pc_gcode_state);
  SnapDbg(SNAP_INFO, "SC cmd chksum count: %u", info.sc_cmd_chk_error);
  SnapDbg(SNAP_INFO, "PC cmd chksum count: %u", info.pc_cmd_chk_error);
  SnapDbg(SNAP_INFO, "Last SC Gcode line: %d", info.last_ack_sc_gcode);
}

#endif // #if (SNAP_DEBUG == 1)
