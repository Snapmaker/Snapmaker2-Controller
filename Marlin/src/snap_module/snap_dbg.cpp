
#include "snap_dbg.h"
#include "../module/StatusControl.h"
#include "../module/PowerPanic.h"
#include "../Marlin.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"

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

static SnapDebugLevel msg_level = SNAP_DEBUG_LEVEL_INFO;

const char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX] = {
  SNAP_TRACE_STR,
  SNAP_INFO_STR,
  SNAP_WARNING_STR,
  SNAP_ERROR_STR,
  SNAP_FATAL_STR
};

// output debug message, will not output message whose level
// is less than msg_level
// param:
//    level - message level
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
  Log(SNAP_DEBUG_LEVEL_INFO, "current status: %d\n", SystemStatus.GetCurrentStatus());
  Log(SNAP_DEBUG_LEVEL_INFO, "SC cmd chksum error count: %u\n", info.screen_cmd_checksum_err);
  Log(SNAP_DEBUG_LEVEL_INFO, "PC cmd chksum error count: %u\n", info.pc_cmd_checksum_err);
  Log(SNAP_DEBUG_LEVEL_INFO, "Last SC Gcode line: %d\n", info.last_line_num_of_sc_gcode);
  Log(SNAP_DEBUG_LEVEL_INFO, "Last save Gcode line: %d\n", powerpanic.Data.FilePosition);
  Log(SNAP_DEBUG_LEVEL_INFO, "Fault flag: 0x%08X, action ban: 0x%X, power ban: 0x%X\n",
        SystemStatus.GetFaultFlag(), action_ban, power_ban);
  Log(SNAP_DEBUG_LEVEL_INFO, "Homing: 0x%X, axes_known: 0x%X\n", axis_homed, axis_known_position);
  Log(SNAP_DEBUG_LEVEL_INFO, "active coordinate: %d\n", gcode.active_coordinate_system);
  Log(SNAP_DEBUG_LEVEL_INFO, "coordinate 1: X: %.3f, Y: %.3f, Z: %.3f\n",
      gcode.coordinate_system[0][X_AXIS], gcode.coordinate_system[0][Y_AXIS], gcode.coordinate_system[0][Z_AXIS]);
}

void SnapDebug::ShowException() {
  uint8_t i;
  uint32_t fault_flag = SystemStatus.GetFaultFlag();

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
