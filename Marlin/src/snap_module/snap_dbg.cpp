
#include "snap_dbg.h"
#include "../module/StatusControl.h"
#include "../module/PowerPanic.h"
#include "../Marlin.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"
#include "snap_cmd.h"
#include "../core/minmax.h"

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
static char log_buf[SNAP_LOG_BUFFER_SIZE];
static uint8_t log_head[12];

const char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX] = {
  SNAP_TRACE_STR,
  SNAP_INFO_STR,
  SNAP_WARNING_STR,
  SNAP_ERROR_STR,
  SNAP_FATAL_STR
};


void SnapDebug::SendLog2Screen(SnapDebugLevel l) {
  int i = 0;
  int j = 0;
  int size = strlen(log_buf);

  if (size == 0)
    return;
  else if (size >= 255) {
    size = 255;
    log_buf[255] = '\0';
  }

  // to include the end '\0'
  size++;

// SOF
  log_head[i++] = 0xAA;
  log_head[i++] = 0x55;

  // length
  log_head[i++] = (uint8_t) ((size + 4) >> 8);
  log_head[i++] = (uint8_t) (size + 4);

  // protocol version
  log_head[i++] = 0x00;

  // length checksum
  log_head[i++] = log_head[2] ^log_head[3];

  // checksum for data
  i++;
  i++;

  // EventID & operation code
  log_head[i++] = EID_STATUS_RESP;
  log_head[i++] = 0x10;

  // log level and length
  log_head[i++] = l;
  log_head[i++] = (uint8_t)size;

  uint32_t checksum = 0;
  checksum += (uint32_t) (((uint8_t) log_head[8] << 8) | (uint8_t) log_head[9]);
  checksum += (uint32_t) (((uint8_t) log_head[10] << 8) | (uint8_t) log_head[11]);

  for (j = 0; j < (size - 1); j = j + 2)
    checksum += (uint32_t) (((uint8_t) log_buf[j] << 8) | (uint8_t) log_buf[j + 1]);

  // if size is odd number
  if (size % 2) checksum += (uint8_t)log_buf[size - 1];

  while (checksum > 0xffff) checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
  checksum = ~checksum;
  log_head[6] = checksum >> 8;
  log_head[7] = checksum;

  for (j = 0; j < i; j++)
    HMISERIAL.write(log_head[j]);
  for (j = 0; j < size; j++)
    HMISERIAL.write(log_buf[j]);
}

// output debug message, will not output message whose level
// is less than msg_level
// param:
//    level - message level
//    fmt - format of messages
//    ... - args
void SnapDebug::Log(SnapDebugLevel level, const char *fmt, ...) {
  int len;
  va_list args;

  if (level < pc_msg_level && level < sc_msg_level)
    return;

  va_start(args, fmt);

  vsnprintf(log_buf, SNAP_LOG_BUFFER_SIZE, fmt, args);

  va_end(args);

  if (level >= pc_msg_level)
    CONSOLE_OUTPUT(log_buf);

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
  Log(SNAP_DEBUG_LEVEL_INFO, "current status: %d\n", SystemStatus.GetCurrentStatus());
  Log(SNAP_DEBUG_LEVEL_INFO, "SC cmd chksum error count: %u\n", info.screen_cmd_checksum_err);
  // Log(SNAP_DEBUG_LEVEL_INFO, "PC cmd chksum error count: %u\n", info.pc_cmd_checksum_err);
  Log(SNAP_DEBUG_LEVEL_INFO, "Last SC Gcode line: %d\n", info.last_line_num_of_sc_gcode);
  Log(SNAP_DEBUG_LEVEL_INFO, "Last save Gcode line: %d\n", powerpanic.LastLine());
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
