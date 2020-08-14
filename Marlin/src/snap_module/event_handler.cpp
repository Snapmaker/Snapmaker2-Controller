#include "event_handler.h"
#include "snap_dbg.h"

#include "level_service.h"
#include "upgrade_service.h"

#include "../Marlin.h"
#include "../module/motion.h"
#include "../module/temperature.h"
#include "../module/planner.h"
#include "../libs/hex_print_routines.h"

#include "../gcode/gcode.h"

#include "../module/StatusControl.h"
#include "../module/PowerPanic.h"
#include "../module/PeriphDevice.h"


#define EVENT_ATTR_HAVE_MOTION  0x1
#define EVENT_ATTR_ABORT_MOTION 0x2

#define EVENT_ATTR_DEFAULT      0x0

#define EVENT_HANDLE_WITH_MARLIN  (EVENT_ATTR_HAVE_MOTION)

#define SUPPORT_MODULE_MAX  32

#define DEBUG_EVENT_HANDLER 0

#define HEART_BEAT_NITIFICATION (60*10)


typedef ErrCode (*CallbackFunc_t)(Event_t &event);

struct EventCallback_t {
  uint8_t attr;
  CallbackFunc_t cb;
};


static ErrCode HandleGcode(uint8_t *event_buff, uint16_t size) {
  event_buff[size] = 0;
  Screen_enqueue_and_echo_commands((char *)(event_buff + 5), INVALID_CMD_LINE, EID_GCODE_ACK);

  return E_SUCCESS;
}

static ErrCode HandleFileGcode(uint8_t *event_buff, uint16_t size) {
  uint32_t line;

  SysStatus   cur_sta = SystemStatus.GetCurrentStatus();
  WorkingPort port = SystemStatus.GetWorkingPort();

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

  if (line > SystemStatus.current_line() || SystemStatus.current_line() == 0) {
    SystemStatus.current_line(line);

    if (cur_sta == SYSTAT_RESUME_WAITING) {
      if (SystemStatus.ResumeOver() == E_SUCCESS) {
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
  else if (line == SystemStatus.current_line()) {
    if (line == debug.GetSCGcodeLine())
      ack_gcode_event(EID_FILE_GCODE_ACK, line);
  }
  else {
    LOG_E("recv line[%u] less than cur line[%u]\n", line, SystemStatus.current_line());
  }

  return E_SUCCESS;
}


static ErrCode SendStatus(Event_t &event) {
  // comment temporarily
  // if (snap_tasks && snap_tasks->heartbeat)
  //   xTaskNotify(snap_tasks->heartbeat, HEART_BEAT_NITIFICATION, eSetBits);

  if (upgrade.GetState() != UPGRADE_STA_UPGRADING_EM)
    SystemStatus.SendStatus(event);
  return E_SUCCESS;
}


static ErrCode SendException(Event_t &event) {
  return SystemStatus.SendException(event);
}


static ErrCode ChangeSystemStatus(Event_t &event) {
  return SystemStatus.ChangeSystemStatus(event);
}


static ErrCode SendLastLine(Event_t &event) {
  return SystemStatus.SendLastLine(event);
}


static ErrCode ClearException(Event_t &event) {
  return SystemStatus.ClearException(event);
}


static ErrCode RecoverFromPowerLoss(Event_t &event) {
  return SystemStatus.RecoverFromPowerLoss(event);
}


static ErrCode SendHomeAndCoordinateStatus(Event_t &event) {
  return SystemStatus.SendHomeAndCoordinateStatus(event);
}


static ErrCode SetLogLevel(Event_t &event) {
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



static ErrCode DoAutoLeveling(Event_t &event) {
  return levelservice.DoAutoLeveling(event);
}

static ErrCode DoManualLeveling(Event_t &event) {
  return levelservice.DoManualLeveling(event);
}

static ErrCode SetManualLevelingPoint(Event_t &event) {
  return levelservice.SetManualLevelingPoint(event);
}

static ErrCode AdjustZOffsetInLeveling(Event_t &event) {
  return levelservice.AdjustZOffsetInLeveling(event);
}

static ErrCode SaveAndExitLeveling(Event_t &event) {
  return levelservice.SaveAndExitLeveling(event);
}

static ErrCode ExitLeveling(Event_t &event) {
  return levelservice.ExitLeveling(event);
}

static ErrCode GetFocalLength(Event_t &event) {
  return ExecuterHead.Laser.GetFocalLength(event);
}

static ErrCode SetFocalLength(Event_t &event) {
  return ExecuterHead.Laser.SetFocalLength(event);
}

static ErrCode DoManualFocusing(Event_t &event) {
  return ExecuterHead.Laser.DoManualFocusing(event);
}

static ErrCode DoAutoFocusing(Event_t &event) {
  return ExecuterHead.Laser.DoAutoFocusing(event);
}


static ErrCode ChangeRuntimeEnv(Event_t &event) {
  return SystemStatus.ChangeRuntimeEnv(event);
}

static ErrCode GetRuntimeEnv(Event_t &event) {
  return SystemStatus.GetRuntimeEnv(event);
}

static ErrCode GetMachineSize(Event_t &event) {
  return SystemStatus.GetMachineSize(event);
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


static ErrCode DoXYZMove(Event_t &event) {
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


static ErrCode SetCameraBtName(Event_t &event) {
  return ExecuterHead.Laser.SetCameraBtName(event);
}

static ErrCode GetCameraBtName(Event_t &event) {
  return ExecuterHead.Laser.GetCameraBtName(event);
}

static ErrCode GetCameraBtMAC(Event_t &event) {
  return ExecuterHead.Laser.GetCameraBtMAC(event);
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
static ErrCode ReportEnclosureStatus(Event_t &event) {
  return Periph.ReportEnclosureStatus(event);
}


static ErrCode SetEnclosureLight(Event_t &event) {
  return Periph.SetEnclosureLight(event);
}


static ErrCode SetEnclosureFan(Event_t &event) {
  return Periph.SetEnclosureFan(event);
}


static ErrCode SetEnclosureDetection(Event_t &event) {
  return Periph.SetEnclosureDetection(event);
}

EventCallback_t addon_event_cb[ADDON_OPC_MAX] = {
  UNDEFINED_CALLBACK,
  /* [ADDON_OPC_GET_CHAMBER_STATUS]    =  */{EVENT_ATTR_DEFAULT,  ReportEnclosureStatus},
  /* [ADDON_OPC_SET_CHAMBER_LIGHT]     =  */{EVENT_ATTR_DEFAULT,  SetEnclosureLight},
  /* [ADDON_OPC_SET_CHAMBER_FAN]       =  */{EVENT_ATTR_DEFAULT,  SetEnclosureFan},
  /* [ADDON_OPC_SET_CHAMBER_DETECTION] =  */{EVENT_ATTR_DEFAULT,  SetEnclosureDetection}
};


static ErrCode SetModuleMAC(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint32_t old_mac;
  uint32_t new_mac;

  PDU_TO_LOCAL_WORD(old_mac, event.data);
  PDU_TO_LOCAL_WORD(new_mac, event.data+4);

  if(CanModules.SetMacID(old_mac, new_mac) == true)
    err = E_SUCCESS;

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


static ErrCode GetModuleMAC(Event_t &event) {
  int i, j = 0;
  uint32_t tmp;
  uint8_t buffer[4 * SUPPORT_MODULE_MAX];

  for(i = 0; i < CanModules.LinearModuleCount; i++) {
    tmp = CanModules.LinearModuleID[i];
    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, tmp, j);
  }

  event.data = buffer;
  event.length = (uint16_t)j;

  return hmi.Send(event);
}


static ErrCode SetLinearModuleLength(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint32_t new_length;
  uint32_t target_mac;

  PDU_TO_LOCAL_WORD(target_mac, event.data);
  PDU_TO_LOCAL_WORD(new_length, event.data+4);

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
  int i, j = 0;
  uint32_t mac;
  uint32_t length;
  uint8_t buffer[8 * SUPPORT_MODULE_MAX];

  CanModules.GetAxesLength();

  for(i = 0; i < CanModules.LinearModuleCount; i++) {
    mac = CanModules.LinearModuleID[i];
    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, mac, j);

    length = CanModules.GetLinearModuleLength(i) * 1000.0f;
    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, length, j);
  }

  event.data = buffer;
  event.length = (uint16_t)j;

  return hmi.Send(event);
}


static ErrCode SetLinearModuleLead(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint32_t new_lead;
  uint32_t target_mac;

  PDU_TO_LOCAL_WORD(target_mac, event.data);
  PDU_TO_LOCAL_WORD(new_lead, event.data+4);

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
  int i, j = 0;
  uint32_t mac;
  uint32_t lead;
  uint8_t buffer[8 * SUPPORT_MODULE_MAX];

  CanModules.GetAxesLead();

  for(i = 0; i < CanModules.LinearModuleCount; i++) {
    mac = CanModules.LinearModuleID[i];
    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, mac, j);


    lead = CanModules.GetLinearModuleLead(i) * 1000.0f;
    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, lead, j);
  }

  event.data = buffer;
  event.length = (uint16_t)j;

  return hmi.Send(event);
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


static ErrCode StartUpgrade(Event_t &event) {
  return upgrade.StartUpgrade(event);
}

static ErrCode ReceiveFW(Event_t &event) {
  return upgrade.ReceiveFW(event);
}

static ErrCode EndUpgarde(Event_t &event) {
  return upgrade.EndUpgarde(event);
}

static ErrCode GetMainControllerVer(Event_t &event) {
  return upgrade.GetMainControllerVer(event);
}

static ErrCode CompareMCVer(Event_t &event) {
  return upgrade.CompareMCVer(event);
}

static ErrCode GetUpgradeStatus(Event_t &event) {
  return upgrade.GetUpgradeStatus(event);
}

static ErrCode GetModuleVer(Event_t &event) {
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
  ErrCode err = E_INVALID_CMD;
  Event_t event = {INVALID_EVENT_ID, INVALID_OP_CODE};

  EventCallback_t *callbacks = NULL;
  uint8_t op_code_max = INVALID_OP_CODE;

  bool send_to_marlin = false;
#if DEBUG_EVENT_HANDLER
  bool heartbeat = false;
#endif

  // if we are running in Marlin task, need to get command from the queue
  if (param->owner == TASK_OWN_MARLIN) {
    if (xMessageBufferIsEmpty(param->event_queue))
      return E_NO_RESRC;

    param->size = (uint16_t)xMessageBufferReceive(param->event_queue, param->event_buff,
                              HMI_RECV_BUFFER_SIZE, 0);

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
  if (!callbacks || (op_code_max == INVALID_OP_CODE)) {
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

