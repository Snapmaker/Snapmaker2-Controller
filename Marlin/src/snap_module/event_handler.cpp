#include "event_handler.h"
#include "snap_dbg.h"

#include "level_service.h"
#include "upgrade_service.h"

#include "../Marlin.h"
#include "../module/motion.h"
#include "../module/temperature.h"
#include "../module/planner.h"

#include "../gcode/gcode.h"

#include "../module/StatusControl.h"
#include "../module/PowerPanic.h"

#define EVENT_ATTR_HAVE_MOTION  0x1
#define EVENT_ATTR_WILL_BLOCKED 0x2
#define EVENT_ATTR_ABORT_MOTION 0x4

#define EVENT_ATTR_DEFAULT      0x0

#define EVENT_HANDLE_WITH_MARLIN  (~(EVENT_ATTR_HAVE_MOTION | EVENT_ATTR_WILL_BLOCKED))

#define SUPPORT_MODULE_MAX  32


static uint32_t current_line = 0;


typedef ErrCode (*CallbackFunc_t)(Event_t &event);

struct EventCallback_t {
  uint8_t attr;
  CallbackFunc_t cb;
};


static ErrCode HandleGcode(uint8_t *event_buff) {
  uint8_t ack_id = event_buff[0] + 1;
  uint32_t line;

  hmi.ToLocalBytes((uint8_t *)&line, event_buff + 1, 4);

  Screen_enqueue_and_echo_commands((char *)(event_buff + 5), line, ack_id);
}


typedef struct {
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t e;

  int16_t bed_current_temp;
  int16_t bed_target_temp;

  int16_t hotend_current_temp;
  int16_t hotend_target_temp;

  uint16_t  feedrate;    // mm/min

  uint32_t  laser_power; // (uint32_t)(float * 1000)
  uint32_t  cnc_rpm;

  uint8_t system_state;
  uint8_t addon_state;

  uint8_t executor_type;
} __packed SystemStatus_t;


static ErrCode CheckIfSendWaitEvent() {
  Event_t event;
  ErrCode err;

  // make sure we are working
  if (SystemStatus.GetCurrentStatus() == SYSTAT_WORK) {
    // and no movement planned
    if(!planner.movesplanned()) {
      // and we have replied screen
      if (current_line && current_line == debug.GetSCGcodeLine()) {
        // then we known maybe screen lost out last reply
        LOG_I("waiting HMI command, current line: %u\n", current_line);
        event.id = EID_SYS_CTRL_ACK;
        event.op_code = SYSCTL_OPC_WAIT_EVENT;
        event.data = &err;
        err = E_FAILURE;

        return hmi.Send(event);
      }
    }
  }

  return E_SUCCESS;
}


static ErrCode SendStatus(Event_t &event) {
  SystemStatus_t sta;

  int32_t tmp_i32;
  int16_t tmp_i16;

  uint32_t tmp_u32;
  uint16_t tmp_u16;

  CheckIfSendWaitEvent();

  // save to use original event to construct new event
  event.data = (uint8_t *)&sta;
  event.length = sizeof(SystemStatus_t);

  // current logical position
  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[X_AXIS], X_AXIS) * 1000);
  hmi.ToPDUBytes((uint8_t *)&sta.x, (uint8_t *)&tmp_i32, 4);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[Y_AXIS], Y_AXIS) * 1000);
  hmi.ToPDUBytes((uint8_t *)&sta.y, (uint8_t *)&tmp_i32, 4);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[Z_AXIS], Z_AXIS) * 1000);
  hmi.ToPDUBytes((uint8_t *)&sta.z, (uint8_t *)&tmp_i32, 4);

  tmp_i32 = (int32_t) (NATIVE_TO_LOGICAL(current_position[E_AXIS], E_AXIS) * 1000);
  hmi.ToPDUBytes((uint8_t *)&sta.e, (uint8_t *)&tmp_i32, 4);

  // temperatures of Bed
  tmp_i16 = (int16_t)thermalManager.degBed();
  hmi.ToPDUBytes((uint8_t *)&sta.bed_current_temp, (uint8_t *)&tmp_i16, 2);

  tmp_i16 = (int16_t)thermalManager.degTargetBed();
  hmi.ToPDUBytes((uint8_t *)&sta.bed_target_temp, (uint8_t *)&tmp_i16, 2);

  // temperatures of hotend
  tmp_i16 = (int16_t)thermalManager.degHotend(0);
  hmi.ToPDUBytes((uint8_t *)&sta.hotend_current_temp, (uint8_t *)&tmp_i16, 2);
  tmp_i16 = (int16_t)thermalManager.degTargetHotend(0);
  hmi.ToPDUBytes((uint8_t *)&sta.hotend_target_temp, (uint8_t *)&tmp_i16, 2);

  // save last feedrate
  tmp_i16 = (int16_t)last_feedrate;
  hmi.ToPDUBytes((uint8_t *)&sta.feedrate, (uint8_t *)&tmp_i16, 2);

  // laser power
  tmp_u32 = ExecuterHead.Laser.GetPower();
  hmi.ToPDUBytes((uint8_t *)&sta.laser_power, (uint8_t *)&tmp_u32, 2);

  // RPM of CNC
  tmp_u32 = ExecuterHead.CNC.GetRPM();
  hmi.ToPDUBytes((uint8_t *)&sta.cnc_rpm, (uint8_t *)&tmp_u32, 2);

  // system status
  sta.system_state = (uint8_t)SystemStatus.MapCurrentStatusForSC();

  // Add-On status
  sta.addon_state = (uint8_t)SystemStatus.GetPeriphDeviceStatus();

  // executor type
  sta.executor_type = ExecuterHead.MachineType;

  return hmi.Send(event);
}


static ErrCode SendException(Event_t &event) {
  uint8_t buff[4];

  uint32_t fault = SystemStatus.GetFaultFlag();

  LOG_I("SC req Exception\n");

  event.length = 0;
  event.data = buff;

  hmi.ToPDUBytes(buff, (uint8_t *)&fault, 4);

  return hmi.Send(event);
}


static ErrCode ChangeSystemStatus(Event_t &event) {
  ErrCode err = E_FAILURE;

  switch (event.op_code)
  {
  case SYSCTL_OPC_REQ_START_WORK:
    LOG_I("SC req START work\n");
    err = SystemStatus.StartWork(TRIGGER_SOURCE_SC);
    if (err == E_SUCCESS)
      current_line = 0;
    break;

  case SYSCTL_OPC_REQ_PAUSE:
    LOG_I("SC req PAUSE\n");
    err = SystemStatus.PauseTrigger(TRIGGER_SOURCE_SC);
    break;

  case SYSCTL_OPC_REQ_RESUME:
    LOG_I("SC req RESUME\n");
    err = SystemStatus.ResumeTrigger(TRIGGER_SOURCE_SC);
    if (err == E_SUCCESS) {
      if (powerpanic.Data.FilePosition > 0)
        current_line =  powerpanic.Data.FilePosition - 1;
      else
        current_line = 0;
      SNAP_DEBUG_SET_GCODE_LINE(0);
      powerpanic.SaveCmdLine(powerpanic.Data.FilePosition);
    }
    break;

  case SYSCTL_OPC_REQ_STOP:
  case SYSCTL_OPC_REQ_FINISH:
    LOG_I("SC req %s\n", (event.op_code == SYSCTL_OPC_REQ_STOP)? "STOP" : "FINISH");
    err = SystemStatus.StopTrigger(TRIGGER_SOURCE_SC, event.op_code);
    break;

  default:
    break;
  }

  if (err != E_SUCCESS) {
    LOG_I("SC req -> Sucess\n");
  }
  else {
    event.length = 1;

    event.data = (uint8_t *)&err;

    LOG_I("SC req -> failed\n");

    hmi.Send(event);
  }
}


static ErrCode SendLastLine(Event_t &event) {
  uint8_t buff[4];

  event.data = buff;
  event.length = 0;

  if (SystemStatus.GetCurrentStage() != SYSTAGE_PAUSE) {
    hmi.ToPDUBytes(buff, (uint8_t *)&powerpanic.pre_data_.FilePosition, 4);
  }
  else {
    hmi.ToPDUBytes(buff, (uint8_t *)&powerpanic.Data.FilePosition, 4);
  }

  return hmi.Send(event);
}


static ErrCode ClearException(Event_t &event) {
  ErrCode err = E_SUCCESS;

  event.data = &err;
  event.length = 1;

  if (event.length == 0) {
    LOG_I("SC req clear power loss bits\n");
    SystemStatus.ClearExceptionByFaultFlag(FAULT_FLAG_POWER_LOSS);
    if (powerpanic.pre_data_.Valid == 1) {
      // clear flash data
      LOG_I("mask flash data ...");
      powerpanic.MaskPowerPanicData();
      powerpanic.pre_data_.Valid = 0;
      LOG_I("Done!\n");
    }
  }
  else if (event.length == 4) {
    uint32_t bit_to_clear = 0;

    hmi.ToLocalBytes((uint8_t *)&bit_to_clear, event.data, 4);
    LOG_I("SC req clear exception, fault bits: 0x%08X\n", bit_to_clear);

    bit_to_clear &= FAULT_FLAG_SC_CLEAR_MASK;
    SystemStatus.ClearExceptionByFaultFlag(bit_to_clear);
  }
  else {
    LOG_E("too many data: %d\n", event.length);
    err = E_FAILURE;
  }

  hmi.Send(event);
}


static ErrCode RecoverFromPowerLoss(Event_t &event) {
  ErrCode err = E_SUCCESS;

  LOG_I("SC trigger restore from power-loss\n");

  event.data = &err;
  event.length = 1;

  SysStatus cur_status = SystemStatus.GetCurrentStatus();

  if (cur_status != SYSTAT_IDLE) {
    LOG_E("cannot trigger recovery at current status: %d\n", cur_status);
    err = E_NO_SWITCHING_STA;
  }
  else {
    // screen bug: why will we receive two consecutive recovery command @TODO
    SystemStatus.SetCurrentStatus(SYSTAT_RESUME_TRIG);
    err = powerpanic.ResumeWork();
    if (err == E_SUCCESS) {
      SystemStatus.SetCurrentStatus(SYSTAT_RESUME_WAITING);
      SystemStatus.SetWorkingPort(WORKING_PORT_SC);
      powerpanic.Data.FilePosition = powerpanic.pre_data_.FilePosition;
      if (powerpanic.Data.FilePosition > 0)
        current_line =  powerpanic.Data.FilePosition - 1;
      else
        current_line = 0;
      SNAP_DEBUG_SET_GCODE_LINE(0);
      powerpanic.SaveCmdLine(powerpanic.Data.FilePosition);
      LOG_I("trigger RESTORE: ok\n");
    }
    else {
      LOG_I("trigger RESTORE: failed, err = %d\n", err);
      SystemStatus.SetCurrentStatus(cur_status);
    }
  }

  return hmi.Send(event);
}


static ErrCode SendHomeAndCoordinateStatus(Event_t &event) {
  uint8_t buff[16] = {0};
  int32_t pos_shift[XYZ];

  int i = 0;

  event.data = buff;

  if (all_axes_homed()) {
    buff[i++] = 0;
  }
  else {
    buff[i++] = 1;
  }

  if (gcode.active_coordinate_system < 0) {
    // number
    buff[i++] = 0;
    // state
    buff[i++] = 0;
    pos_shift[X_AXIS] = (int32_t)(position_shift[X_AXIS] * 1000);
    pos_shift[Y_AXIS] = (int32_t)(position_shift[Y_AXIS] * 1000);
    pos_shift[Z_AXIS] = (int32_t)(position_shift[Z_AXIS] * 1000);
  }
  else {
    buff[i++] = gcode.active_coordinate_system + 1;
    // check state
    if ((position_shift[X_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS]) &&
        (position_shift[Y_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS]) &&
        (position_shift[Z_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS])) {
      buff[i++] = 0;
    }
    else {
      buff[i++] = 1;
    }
    pos_shift[X_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS] * 1000);
    pos_shift[Y_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS] * 1000);
    pos_shift[Z_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS] * 1000);
  }

  hmi.ToPDUBytes(buff, (uint8_t *)&pos_shift[X_AXIS], 4);
  hmi.ToPDUBytes(buff, (uint8_t *)&pos_shift[Y_AXIS], 4);
  hmi.ToPDUBytes(buff, (uint8_t *)&pos_shift[Z_AXIS], 4);
  i += 12;

  event.length = i;

  return hmi.Send(event);
}


static ErrCode SetLogLevel(Event_t &event) {
  ErrCode err = E_FAILURE;

  if (event.length != 1) {
    LOG_E("Need to specify log level!\n");
    event.data = &err;
    event.length = 1;
  }
  else {
    LOG_V("SC req change log level");
    SNAP_DEBUG_SET_LEVEL(1, (SnapDebugLevel)event.data[0]);
    event.data[0] = E_SUCCESS;
  }

  return hmi.Send(event);
}

EventCallback_t sysctl_event_cb[SYSCTL_OPC_MAX] = {
  [SYSCTL_OPC_REQ_STATUES]            = {EVENT_ATTR_DEFAULT,    SendStatus},
  [SYSCTL_OPC_REQ_EXCEPTION]          = {EVENT_ATTR_DEFAULT,    SendException},
  [SYSCTL_OPC_REQ_START_WORK]         = {EVENT_ATTR_DEFAULT,    ChangeSystemStatus},
  [SYSCTL_OPC_REQ_PAUSE]              = {EVENT_ATTR_DEFAULT,    ChangeSystemStatus},
  [SYSCTL_OPC_REQ_RESUME]             = {EVENT_ATTR_DEFAULT,    ChangeSystemStatus},
  [SYSCTL_OPC_REQ_STOP]               = {EVENT_ATTR_DEFAULT,    ChangeSystemStatus},
  [SYSCTL_OPC_REQ_FINISH]             = {EVENT_ATTR_DEFAULT,    ChangeSystemStatus},
  [SYSCTL_OPC_REQ_LAST_LINE]          = {EVENT_ATTR_DEFAULT,    SendLastLine},
  [SYSCTL_OPC_REQ_CLEAR_FAULT]        = {EVENT_ATTR_DEFAULT,    ClearException},
  [SYSCTL_OPC_REQ_RECOVER_POWER_LOSS] = {EVENT_ATTR_DEFAULT,    RecoverFromPowerLoss},
  [SYSCTL_OPC_REQ_HOME_STATUS]        = {EVENT_ATTR_DEFAULT,    SendHomeAndCoordinateStatus},
  [SYSCTL_OPC_SET_LOG_LEVEL]          = {EVENT_ATTR_DEFAULT,    SetLogLevel}
};


static ErrCode ChangeRuntimeEnv(Event_t &event) {
  return hmi.Send(event);
}

EventCallback_t settings_event_cb[SETTINGS_OPC_MAX] = {
  [SETTINGS_OPC_SET_MACHINE_SIZE]       = {EVENT_ATTR_DEFAULT,      NULL},
  [SETTINGS_OPC_REQ_AUTO_LEVELING]      = {EVENT_ATTR_HAVE_MOTION,  levelhandler.DoAutoLeveling},
  [SETTINGS_OPC_REQ_MANUAL_LEVELING]    = {EVENT_ATTR_HAVE_MOTION,  levelhandler.DoManualLeveling},
  [SETTINGS_OPC_SET_LEVELING_PONIT]     = {EVENT_ATTR_HAVE_MOTION,  levelhandler.SetManualLevelingPoint},
  [SETTINGS_OPC_ADJUST_Z_OFFSET]        = {EVENT_ATTR_HAVE_MOTION,  levelhandler.AdjustZOffsetInLeveling},
  [SETTINGS_OPC_SAVE_AND_EXIT_LEVELING] = {EVENT_ATTR_HAVE_MOTION,  levelhandler.SaveAndExitLeveling},
  [SETTINGS_OPC_EXIT_LEVELING]          = {EVENT_ATTR_HAVE_MOTION,  levelhandler.ExitLeveling},
  [SETTINGS_OPC_RESTORE_TO_FACTORY]     = {EVENT_ATTR_DEFAULT,      NULL},
  [SETTINGS_OPC_READ_FOCAL_LENGTH]      = {EVENT_ATTR_DEFAULT,      ExecuterHead.Laser.GetFocalLength},
  [SETTINGS_OPC_SET_FOCAL_LENGTH]       = {EVENT_ATTR_DEFAULT,      ExecuterHead.Laser.SetFocalLength},
  [SETTINGS_OPC_REQ_MANUAL_FOCUSING]    = {EVENT_ATTR_HAVE_MOTION,  ExecuterHead.Laser.DoManualFocusing},
  [SETTINGS_OPC_REQ_AUTO_FOCUSING]      = {EVENT_ATTR_HAVE_MOTION,  ExecuterHead.Laser.DoAutoFocusing},
  [SETTINGS_OPC_REQ_FAST_CALIBRATION]   = {EVENT_ATTR_HAVE_MOTION,  levelhandler.DoAutoLeveling},
  [SETTINGS_OPC_CHANGE_RUNTIME_ENV]     = {EVENT_ATTR_DEFAULT,      ChangeRuntimeEnv},
  [SETTINGS_OPC_RESET_LEVELING]         = {EVENT_ATTR_WILL_BLOCKED, levelhandler.ResetLeveling}
};


static ErrCode DoXYZMove(Event_t &event) {
  ErrCode err = E_FAILURE;

  float target_pos[XYZ];
  float speed = 0;

  if (event.length < 12) {
    LOG_E("Must specify target X, Y, Z in event\n");
    goto out;
  }

  hmi.ToLocalBytes((uint8_t *)&target_pos[X_AXIS], event.data, 4);
  hmi.ToLocalBytes((uint8_t *)&target_pos[Y_AXIS], event.data+4, 4);
  hmi.ToLocalBytes((uint8_t *)&target_pos[Z_AXIS], event.data+8, 4);

  LOOP_XYZ(i) {
    target_pos[i] /= 1000;
  }

  if (event.length == 16) {
    hmi.ToLocalBytes((uint8_t *)&speed, event.data+12, 4);
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


static ErrCode DoEMove(Event_t &event) {
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

  hmi.ToLocalBytes((uint8_t *)&extrude_len, event.data+1, 4);
  hmi.ToLocalBytes((uint8_t *)&extrude_speed, event.data+5, 4);
  hmi.ToLocalBytes((uint8_t *)&retract_len, event.data+9, 4);
  hmi.ToLocalBytes((uint8_t *)&retract_speed, event.data+13, 4);

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
  [MOTION_OPC_DO_ABSOLUTE_MOVE]     = {EVENT_ATTR_HAVE_MOTION,  DoXYZMove},
  [MOTION_OPC_DO_RELATIVE_MOVE]     = {EVENT_ATTR_HAVE_MOTION,  DoXYZMove},
  [MOTION_OPC_DO_E_MOVE]            = {EVENT_ATTR_HAVE_MOTION,  DoEMove}
};


EventCallback_t camera_event_cb[CAMERA_OPC_MAX] = {
  [CAMERA_OPC_SET_BT_NAME]          = {EVENT_ATTR_DEFAULT,  ExecuterHead.Laser.SetCameraBtName},
  [CAMERA_OPC_READ_BT_NAME]         = {EVENT_ATTR_DEFAULT,  ExecuterHead.Laser.GetCameraBtName},
  [CAMERA_OPC_READ_BT_MAC]          = {EVENT_ATTR_DEFAULT,  ExecuterHead.Laser.GetCameraBtMAC}
};


// implement follow 4 function after rebase chamber branch
static ErrCode GetChamerStatus(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode SetChamberLight(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode SetChamberFAN(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode SetChamberDetection(Event_t &event) {
  return hmi.Send(event);
}

EventCallback_t addon_event_cb[ADDON_OPC_MAX] = {
  [ADDON_OPC_GET_CHAMBER_STATUS]    = {EVENT_ATTR_DEFAULT,  GetChamerStatus},
  [ADDON_OPC_SET_CHAMBER_LIGHT]     = {EVENT_ATTR_DEFAULT,  SetChamberLight},
  [ADDON_OPC_SET_CHAMBER_FAN]       = {EVENT_ATTR_DEFAULT,  SetChamberFAN},
  [ADDON_OPC_SET_CHAMBER_DETECTION] = {EVENT_ATTR_DEFAULT,  SetChamberDetection}
};


static ErrCode SetModuleMAC(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint32_t old_mac;
  uint32_t new_mac;

  hmi.ToLocalBytes((uint8_t *)&old_mac, event.data, 4);
  hmi.ToLocalBytes((uint8_t *)&new_mac, event.data+4, 4);

  if(CanModules.SetMacID(old_mac, new_mac) == true)
    err = E_SUCCESS;

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


static ErrCode GetModuleMAC(Event_t &event) {
  int i;
  uint32_t tmp;
  uint8_t buffer[4 * SUPPORT_MODULE_MAX];

  for(i = 0; i < CanModules.LinearModuleCount; i++) {
    tmp = CanModules.LinearModuleID[i];
    hmi.ToPDUBytes(buffer + 4*i, (uint8_t *)&tmp, 4);
  }

  event.data = buffer;
  event.length = 4 * i;

  return hmi.Send(event);
}


static ErrCode SetLinearModuleLength(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint32_t new_length;
  uint32_t target_mac;

  hmi.ToLocalBytes((uint8_t *)&target_mac, event.data, 4);
  hmi.ToLocalBytes((uint8_t *)&new_length, event.data+4, 4);

  target_mac = ((uint32_t)target_mac << 1);
  new_length = new_length / 1000.0f;

  LOG_I("ID", target_mac, "New Len:", new_length);
  if(CanModules.SetAxesLength(target_mac, new_length) == true)
    err = E_SUCCESS;

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


static ErrCode GetLinearModuleLength(Event_t &event) {
  int i;
  uint32_t mac;
  uint32_t length;
  uint8_t buffer[8 * SUPPORT_MODULE_MAX];

  CanModules.GetAxesLength();

  for(i = 0; i < CanModules.LinearModuleCount; i++) {
    mac = CanModules.LinearModuleID[i];
    hmi.ToPDUBytes(buffer + 8*i, (uint8_t *)&mac, 4);

    length = CanModules.GetLinearModuleLength(i) * 1000.0f;
    hmi.ToPDUBytes(buffer + 8*i + 4, (uint8_t *)&length, 4);
  }

  event.data = buffer;
  event.length = 8 * i;

  return hmi.Send(event);
}


static ErrCode SetLinearModuleLead(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint32_t new_lead;
  uint32_t target_mac;

  hmi.ToLocalBytes((uint8_t *)&target_mac, event.data, 4);
  hmi.ToLocalBytes((uint8_t *)&new_lead, event.data+4, 4);

  target_mac = ((uint32_t)target_mac << 1);

  new_lead = new_lead / 1000.0f;

  SERIAL_ECHOLNPAIR("ID", target_mac, "New Lead:", new_lead);

  if(CanModules.SetAxesLead(target_mac, new_lead) == true)
    err = E_SUCCESS;

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


static ErrCode GetLinearModuleLead(Event_t &event) {
  int i;
  uint32_t mac;
  uint32_t lead;
  uint8_t buffer[8 * SUPPORT_MODULE_MAX];

  CanModules.GetAxesLead();

  for(i = 0; i < CanModules.LinearModuleCount; i++) {
    mac = CanModules.LinearModuleID[i];
    hmi.ToPDUBytes(buffer + 8*i, (uint8_t *)&mac, 4);

    lead = CanModules.GetLinearModuleLead(i) * 1000.0f;
    hmi.ToPDUBytes(buffer + 8*i + 4, (uint8_t *)&lead, 4);
  }

  event.data = buffer;
  event.length = 8 * i;

  return hmi.Send(event);
}

EventCallback_t debug_event_cb[DEBUG_OPC_MAX] = {
  [DEBUG_OPC_SET_MODULE_MAC]        = {EVENT_ATTR_DEFAULT,  SetModuleMAC},
  [DEBUG_OPC_GET_MODULE_MAC]        = {EVENT_ATTR_DEFAULT,  GetModuleMAC},
  [DEBUG_OPC_SET_LINEAR_LENGTH]     = {EVENT_ATTR_DEFAULT,  SetLinearModuleLength},
  [DEBUG_OPC_GET_LINEAR_LENGTH]     = {EVENT_ATTR_DEFAULT,  GetLinearModuleLength},
  [DEBUG_OPC_SET_LINEAR_LEAD]       = {EVENT_ATTR_DEFAULT,  SetLinearModuleLead},
  [DEBUG_OPC_GET_LINEAR_LEAD]       = {EVENT_ATTR_DEFAULT,  GetLinearModuleLead}
};


EventCallback_t upgrade_event_cb[UPGRADE_OPC_MAX] = {
  [UPGRADE_OPC_START]               = {EVENT_ATTR_DEFAULT,  upgrade.StartUpgrade},
  [UPGRADE_OPC_TRANS_FW]            = {EVENT_ATTR_DEFAULT,  upgrade.ReceiveFW},
  [UPGRADE_OPC_END]                 = {EVENT_ATTR_DEFAULT,  upgrade.EndUpgarde},
  [UPGRADE_OPC_GET_MC_VER]          = {EVENT_ATTR_DEFAULT,  upgrade.GetMainControllerVer},
  [UPGRADE_OPC_REQ_COMPARE_VER]     = {EVENT_ATTR_DEFAULT,  upgrade.CompareMCVer},
  [UPGRADE_OPC_GET_UP_STATUS]       = {EVENT_ATTR_DEFAULT,  upgrade.GetUpgradeStatus},
  [UPGRADE_OPC_GET_MODULE_VER]      = {EVENT_ATTR_DEFAULT,  upgrade.GetModuleVer}
};


// need to known which task we running with
// then we won't send out event again
ErrCode DispatchEvent(DispatcherParam_t param) {
  ErrCode err = E_INVALID_CMD;
  Event_t event = {INVALID_EVENT_ID, INVALID_OP_CODE};

  EventCallback_t *callbacks = NULL;
  uint8_t op_code_max = INVALID_OP_CODE;

  bool send_to_marlin = false;

  // if we are running in Marlin task, need to get command from the queue
  if (param->owner == TASK_OWN_MARLIN) {
    if (xMessageBufferIsEmpty(param->event_queue))
      return E_NO_RESRC;

    param->size = (uint16_t)xMessageBufferReceive(param->event_queue, param->event_buff,
                              HMI_RECV_BUFFER_SIZE, configTICK_RATE_HZ/1000);

    if (!param->size) {
      LOG_E("No data read from event queue!\n");
      return E_NO_RESRC;
    }
  }

  event.id = param->event_buff[EVENT_IDX_EVENT_ID];

  switch (event.id) {
  case EID_GCODE_REQ:
  case EID_FILE_GCODE_REQ:
    if (param->owner == TASK_OWN_MARLIN) {
      return HandleGcode(param->event_buff);
    }
    else
      send_to_marlin = true;
    break;

  case EID_SYS_CTRL_REQ:
    callbacks = sysctl_event_cb;
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
    // blocked 100ms for max duration to wait
    xMessageBufferSend(param->event_queue, param->event_buff, param->size, configTICK_RATE_HZ/10);
    return E_SUCCESS;
  }

  // increase the event id to get ACK id for event callback
  // or for return error to Screen
  event.id++;

  if (param->size < 2) {
    LOG_E("invalid event[%X], no operation code field\n", param->event_buff[EVENT_IDX_EVENT_ID]);
    goto out_err;
  }

  // if no callback or invalid op code for this event, should ack error to Screen
  if (!callbacks || (op_code_max == INVALID_OP_CODE) || param->event_buff[EVENT_IDX_OP_CODE] >= op_code_max) {
    LOG_E("invalid event： [%X : %X], max opc: %d\n",
      param->event_buff[EVENT_IDX_EVENT_ID], param->event_buff[EVENT_IDX_OP_CODE], op_code_max);
    goto out_err;
  }

  // found relative callback group, but didn't add cabllback function for this operation code
  if (!callbacks[event.op_code].cb) {
    LOG_E("event[0x%X]: op code [0x%X] doesn't have callback\n",
      param->event_buff[EVENT_IDX_EVENT_ID], param->event_buff[EVENT_IDX_OP_CODE]);
    goto out_err;
  }

  if ((callbacks[event.op_code].attr & EVENT_HANDLE_WITH_MARLIN) && (param->owner != TASK_OWN_MARLIN)) {
    // arrive here, we know the event need to send to Marlin by check event id & op code
    xMessageBufferSend(param->event_queue, param->event_buff, param->size, configTICK_RATE_HZ/10);
    return E_SUCCESS;
  }

  event.op_code = param->event_buff[EVENT_IDX_OP_CODE];

  return callbacks[event.op_code].cb(event);

out_err:
  event.data = &err;
  event.length = 1;
  hmi.Send(event);
  return err;
}



