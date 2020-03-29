#include "event_handler.h"
#include "snap_dbg.h"
#include "../module/motion.h"
#include "../module/temperature.h"

#include "../module/StatusControl.h"
#include "../module/PowerPanic.h"

#define EVENT_ATTR_HAVE_MOTION  0x1
#define EVENT_ATTR_WILL_BLOCKED 0x2
#define EVENT_ATTR_ABORT_MOTION 0x4

#define EVENT_ATTR_DEFAULT      0x0

#define EVENT_HANDLE_WITH_MARLIN  (~(EVENT_ATTR_HAVE_MOTION | EVENT_ATTR_WILL_BLOCKED))

typedef ErrCode (*CallbackFunc_t)(Event_t &event);


struct EventCallback_t {
  uint8_t attr;
  CallbackFunc_t cb;
};


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


static ErrCode SendStatus(Event_t &event) {
  SystemStatus_t sta;

  int32_t tmp_i32;
  int16_t tmp_i16;

  uint32_t tmp_u32;
  uint16_t tmp_u16;

  // save to use original event to construct new event
  event.id++;
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

  event.id++;
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
    event.id++;
    event.length = 1;

    event.data = (uint8_t *)&err;

    LOG_I("SC req -> failed\n");

    hmi.Send(event);
  }
}


static ErrCode SendLastLine(Event_t &event) {
  uint8_t buff[4];
  uint32_t last_line = powerpanic.;

  event.id++;
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

  event.id++;
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

  event.id++;
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
      MarkNeedReack(0);
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

  event.id++;
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

  event.id++;

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
  [SYSCTL_OPC_REQ_STATUES]            = {EVENT_ATTR_DEFAULT,               SendStatus},
  [SYSCTL_OPC_REQ_EXCEPTION]          = {EVENT_ATTR_DEFAULT,               SendException},
  [SYSCTL_OPC_REQ_START_WORK]         = {EVENT_ATTR_DEFAULT,               ChangeSystemStatus},
  [SYSCTL_OPC_REQ_PAUSE]              = {EVENT_ATTR_DEFAULT,               ChangeSystemStatus},
  [SYSCTL_OPC_REQ_RESUME]             = {EVENT_ATTR_DEFAULT,               ChangeSystemStatus},
  [SYSCTL_OPC_REQ_STOP]               = {EVENT_ATTR_DEFAULT,               ChangeSystemStatus},
  [SYSCTL_OPC_REQ_FINISH]             = {EVENT_ATTR_DEFAULT,               ChangeSystemStatus},
  [SYSCTL_OPC_REQ_LAST_LINE]          = {EVENT_ATTR_DEFAULT,               SendLastLine},
  [SYSCTL_OPC_REQ_CLEAR_FAULT]        = {EVENT_ATTR_DEFAULT,               ClearException},
  [SYSCTL_OPC_REQ_RECOVER_POWER_LOSS] = {EVENT_ATTR_DEFAULT,               RecoverFromPowerLoss},
  [SYSCTL_OPC_REQ_HOME_STATUS]        = {EVENT_ATTR_DEFAULT,               SendHomeAndCoordinateStatus},
  [SYSCTL_OPC_SET_LOG_LEVEL]          = {EVENT_ATTR_DEFAULT,               SetLogLevel}
};


static ErrCode DoAutoLeveling(Event_t &event) {

  return hmi.Send(event);
}


static ErrCode DoManualLeveling(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode SetManualLevelingPoint(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode AdjustZOffsetInLeveling(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode SaveAndExitLeveling(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode ExitLeveling(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode GetFocalLength(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode SetFocalLength(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode DoManualFocusing(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode DoAutoFocusing(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode DoFastLeveling(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode ChangeRuntimeEnv(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode ResetLeveling(Event_t &event) {
  return hmi.Send(event);
}

EventCallback_t settings_event_cb[SETTINGS_OPC_MAX] = {
  [SETTINGS_OPC_SET_MACHINE_SIZE]       = {EVENT_ATTR_DEFAULT,      NULL},
  [SETTINGS_OPC_REQ_AUTO_LEVELING]      = {EVENT_ATTR_HAVE_MOTION,  DoAutoLeveling},
  [SETTINGS_OPC_REQ_MANUAL_LEVELING]    = {EVENT_ATTR_HAVE_MOTION,  DoManualLeveling},
  [SETTINGS_OPC_SET_LEVELING_PONIT]     = {EVENT_ATTR_HAVE_MOTION,  SetManualLevelingPoint},
  [SETTINGS_OPC_ADJUST_Z_OFFSET]        = {EVENT_ATTR_HAVE_MOTION,  AdjustZOffsetInLeveling},
  [SETTINGS_OPC_SAVE_AND_EXIT_LEVELING] = {EVENT_ATTR_HAVE_MOTION,  SaveAndExitLeveling},
  [SETTINGS_OPC_EXIT_LEVELING]          = {EVENT_ATTR_HAVE_MOTION,  ExitLeveling},
  [SETTINGS_OPC_RESTORE_TO_FACTORY]     = {EVENT_ATTR_DEFAULT,      NULL},
  [SETTINGS_OPC_READ_FOCAL_LENGTH]      = {EVENT_ATTR_DEFAULT,      GetFocalLength},
  [SETTINGS_OPC_SET_FOCAL_LENGTH]       = {EVENT_ATTR_DEFAULT,      SetFocalLength},
  [SETTINGS_OPC_REQ_MANUAL_FOCUSING]    = {EVENT_ATTR_HAVE_MOTION,  DoManualFocusing},
  [SETTINGS_OPC_REQ_AUTO_FOCUSING]      = {EVENT_ATTR_HAVE_MOTION,  DoAutoFocusing},
  [SETTINGS_OPC_REQ_FAST_CALIBRATION]   = {EVENT_ATTR_HAVE_MOTION,  DoFastLeveling},
  [SETTINGS_OPC_CHANGE_RUNTIME_ENV]     = {EVENT_ATTR_DEFAULT,      ChangeRuntimeEnv},
  [SETTINGS_OPC_RESET_LEVELING]         = {EVENT_ATTR_DEFAULT,      ResetLeveling}
};


static ErrCode DoAbsoluteMove(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode DoRelativeMove(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode DoEMove(Event_t &event) {
  return hmi.Send(event);
}

EventCallback_t motion_event_cb[MOTION_OPC_MAX] = {
  [MOTION_OPC_DO_ABSOLUTE_MOVE]     = {EVENT_ATTR_HAVE_MOTION,  DoAbsoluteMove},
  [MOTION_OPC_DO_RELATIVE_MOVE]     = {EVENT_ATTR_HAVE_MOTION,  DoRelativeMove},
  [MOTION_OPC_DO_E_MOVE]            = {EVENT_ATTR_HAVE_MOTION,  DoEMove}
};


static ErrCode SetCameraBtName(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode GetCameraBtName(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode GetCameraBtMAC(Event_t &event) {
  return hmi.Send(event);
}

EventCallback_t camera_event_cb[CAMERA_OPC_MAX] = {
  [CAMERA_OPC_SET_BT_NAME]          = {EVENT_ATTR_DEFAULT,  SetCameraBtName},
  [CAMERA_OPC_READ_BT_NAME]         = {EVENT_ATTR_DEFAULT,  GetCameraBtName},
  [CAMERA_OPC_READ_BT_MAC]          = {EVENT_ATTR_DEFAULT,  GetCameraBtMAC}
};


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
  return hmi.Send(event);
}


static ErrCode GetModuleMAC(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode SetLinearModuleLength(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode GetLinearModuleLength(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode SetLinearModuleLead(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode GetLinearModuleLead(Event_t &event) {
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


static ErrCode StartUpgrade(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode TansferFW(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode EndUpgarde(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode GetMainControllerVer(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode CompareMCVer(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode GetUpgradeStatus(Event_t &event) {
  return hmi.Send(event);
}


static ErrCode GetModuleVer(Event_t &event) {
  return hmi.Send(event);
}

EventCallback_t upgrade_event_cb[UPGRADE_OPC_MAX] = {
  [UPGRADE_OPC_START]               = {EVENT_ATTR_DEFAULT,  StartUpgrade},
  [UPGRADE_OPC_TRANS_FW]            = {EVENT_ATTR_DEFAULT,  TansferFW},
  [UPGRADE_OPC_END]                 = {EVENT_ATTR_DEFAULT,  EndUpgarde},
  [UPGRADE_OPC_GET_MC_VER]          = {EVENT_ATTR_DEFAULT,  GetMainControllerVer},
  [UPGRADE_OPC_REQ_COMPARE_VER]     = {EVENT_ATTR_DEFAULT,  CompareMCVer},
  [UPGRADE_OPC_GET_UP_STATUS]       = {EVENT_ATTR_DEFAULT,  GetUpgradeStatus},
  [UPGRADE_OPC_GET_MODULE_VER]      = {EVENT_ATTR_DEFAULT,  GetModuleVer}
};


#define EVENT_HANDLER_MARLIN  0
#define EVENT_HANDLER_HMI     1
// need to known which task we running with
// then we won't send out event again
ErrCode HandleEvent(uint8_t *cmd, uint16_t size, MessageBufferHandle_t cmd_queue=NULL) {
  Event_t event;
  bool send_to_marlin = false;

  uint8_t handler;

  if (cmd_queue) {
    handler = EVENT_HANDLER_HMI;
  }
  else

  event.id = cmd[CMD_IDX_EVENT_ID];

  if (size <= 0)
    return;

  switch (event.id) {
  case EID_GCODE_REQ:
  case EID_FILE_GCODE_REQ:
    send_to_marlin = true;
    break;

  case EID_SYS_CTRL_REQ:
    event.op_code = cmd[CMD_IDX_OP_CODE];

    if (event.op_code >= SYSCTL_OPC_MAX) {
      LOG_E("event[0x%X]: op code [0x%X] is out of range: %d\n", event.id, event.op_code, SYSCTL_OPC_MAX);
      return E_PARAM;
    }

    if (!sysctl_event_cb[event.op_code].cb) {
      LOG_E("event[0x%X]: op code [0x%X] doesn't have callback\n", event.id, event.op_code);
      return E_NO_RESRC;
    }

    if (sysctl_event_cb[event.op_code].attr & EVENT_HANDLE_WITH_MARLIN) {
      send_to_marlin = true;
      break;
    }

    return sysctl_event_cb[event.op_code].cb(event);
    break;

  case EID_SETTING_REQ:
    break;

  case EID_MOVEMENT_REQ:
    break;

  case EID_LAS_CAM_OP_REQ:
    break;

  case EID_ADDON_OP_REQ:
    break;

  case EID_LASER_CALIBRATE_REQ:
    break;

  case EID_UPGRADE_REQ:
    break;

  default:
    break;
  }


  if (cmd_queue && send_to_marlin)
    xMessageBufferSend(cmd_queue, cmd, size, portMAX_DELAY);

  return;
}



