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
#ifndef SNAPMAKER_SYSTEM_H_
#define SNAPMAKER_SYSTEM_H_

#include "../common/config.h"
#include "../common/error.h"

#include "../hmi/event_handler.h"
#include "quick_stop.h"

#include "src/inc/MarlinConfig.h"


#define FAULT_FLAG_NO_EXECUTOR        1       // lost executor
#define FAULT_FLAG_NO_LINEAR          (1<<1)  // no any linear module detect
#define FAULT_FLAG_BED_PORT           (1<<2)  // NMOS for controlling bed is breakdown
#define FAULT_FLAG_FILAMENT	          (1<<3)
#define FAULT_FLAG_LOST_SETTING	      (1<<4)
#define FAULT_FLAG_LOST_EXECUTOR      (1<<5)
#define FAULT_FLAG_POWER_LOSS	        (1<<6)
#define FAULT_FLAG_HOTEND_HEATFAIL    (1<<7)  /* invalid power panic data */
#define FAULT_FLAG_BED_HEATFAIL       (1<<8)  /* EEPROM settings lost*/
#define FAULT_FLAG_HOTEND_RUNWAWY     (1<<9) // temperature of hotend is runaway
#define FAULT_FLAG_BED_RUNAWAY        (1<<10) // temperature of bed is runaway
#define FAULT_FLAG_HOTEND_SENSOR_BAD  (1<<11) // temperature sensor of hotend is abnormal
#define FAULT_FLAG_BED_SENSOR_BAD     (1<<12) // temperature sensor of bed is abnormal
#define FAULT_FLAG_LOST_LINEAR        (1<<13)
#define FAULT_FLAG_HOTEND_MAXTEMP     (1<<14)
#define FAULT_FLAG_BED_MAXTEMP        (1<<15)
#define FAULT_FLAG_HOTEND_SHORTCIRCUIT    (1<<16)
#define FAULT_FLAG_BED_SHORTCIRCUIT       (1<<17)
#define FAULT_FLAG_HOTEND_SENSOR_COMEOFF  (1<<18)
#define FAULT_FLAG_BED_SENSOR_COMEOFF     (1<<19)
#define FAULT_FLAG_UNKNOW_MODEL       (1<<20)
#define FAULT_FLAG_DOOR_OPENED        (1<<21)
#define FAULT_FLAG_POWER_DETECT_ERR   (1<<22)
#define FAULT_FLAG_UNKNOW             (1<<31)

// this macro mask the bits which are allow to be cleared by screen
#define FAULT_FLAG_SC_CLEAR_MASK      (FAULT_FLAG_POWER_LOSS | FAULT_FLAG_FILAMENT | FAULT_FLAG_LOST_SETTING \
                                      | FAULT_FLAG_HOTEND_HEATFAIL | FAULT_FLAG_BED_HEATFAIL | FAULT_FLAG_HOTEND_RUNWAWY \
                                      | FAULT_FLAG_BED_RUNAWAY | FAULT_FLAG_HOTEND_MAXTEMP | FAULT_FLAG_BED_MAXTEMP \
                                      | FAULT_FLAG_UNKNOW | FAULT_FLAG_DOOR_OPENED)

// exception actions
#define EACTION_NONE                0
#define EACTION_PAUSE_WORKING       0x1
#define EACTION_STOP_WORKING        (0x1<<1)
#define EACTION_STOP_HEATING_BED    (0x1<<2)
#define EACTION_STOP_HEATING_HOTEND (0x1<<3)

#define EXCEPTION_ISR_BUFFSER_SIZE  4

enum ExceptionHost : int8_t {
  EHOST_CHAMBER = -2,  /* -2 ~ 6 just are to match Thermal Manage of Marlin */
  EHOST_BED = -1,
  EHOST_HOTEND0 = 0,
  EHOST_HOTEND1 = 1,
  EHOST_HOTEND2 = 2,
  EHOST_EXECUTOR = 7,  /* all executors, 0-6 is reserved for extruders */
  EHOST_LINEAR,        /* linear modules */
  EHOST_MC,            /* main control */

  EHOST_INVALID
};

enum ExceptionType : uint8_t {
  ETYPE_NO_HOST = 0,
  ETYPE_LOST_HOST,
  ETYPE_RUNOUT,
  ETYPE_LOST_CFG,
  ETYPE_PORT_BAD,
  ETYPE_POWER_LOSS,
  ETYPE_HEAT_FAIL,
  ETYPE_TEMP_RUNAWAY,
  ETYPE_TEMP_REDUNDANCY,
  ETYPE_SENSOR_BAD,
  ETYPE_BELOW_MINTEMP,
  ETYPE_OVERRUN_MAXTEMP,
  ETYPE_OVERRUN_MAXTEMP_AGAIN,
  ETYPE_ABRUPT_TEMP_DROP,
  ETYPE_SENSOR_COME_OFF,
  ETYPE_LINEAR_MODULE_DIFF_DRIVER,
  ETYPE_LINEAR_MODULE_LEAD_ERROR,

  ETYPE_INVALID
};


enum TriggerSource : uint8_t {
  TRIGGER_SOURCE_NONE,
  TRIGGER_SOURCE_SC,              // trigger by screen
  TRIGGER_SOURCE_PC,              // trigger by PC
  TRIGGER_SOURCE_RUNOUT,          // trigger by filament runout
  TRIGGER_SOURCE_DOOR_OPEN,       // trigger by door opened
  TRIGGER_SOURCE_DOOR_CLOSE,      // trigger by door closed
  TRIGGER_SOURCE_STOP_BUTTON,     // trigger by emergency button
  TRIGGER_SOURCE_EXCEPTION,       // trigger by exception
  TRIGGER_SOURCE_SC_LOST,         // lost screen when working with it

  TRIGGER_SOURCE_INVALID
};


enum StopType : uint8_t {
  STOP_TYPE_ABORTED = 0,
  STOP_TYPE_FINISH,

  STOP_TYPE_INVALID
};


// status of system
enum SysStatus: uint8_t {
  SYSTAT_INIT = 0,

  SYSTAT_IDLE,            // generally, we are in this status after initialization

  SYSTAT_WORK,            // working

  SYSTAT_PAUSE_TRIG,      // pause is triggered by some events
  SYSTAT_PAUSE_STOPPED,   // quick stop is handled
  SYSTAT_PAUSE_FINISH,    // the continuous status for pause

  SYSTAT_RESUME_TRIG,     // resume op is triggered
  SYSTAT_RESUME_MOVING,   // moving to the position when pause
  SYSTAT_RESUME_WAITING,  // waiting the Gcode

  SYSTAT_END_TRIG,
  SYSTAT_END_FINISH,
  SYSTAT_INVALID
};

// stage of system, some one just focus on stage
// but some one focus on detail status
enum SysStage : uint8_t {
  SYSTAGE_INIT,
  SYSTAGE_IDLE,
  SYSTAGE_WORK,
  SYSTAGE_PAUSE,
  SYSTAGE_RESUMING,
  SYSTAGE_END,
  SYSTAGE_INVALID
};

enum WorkingPort : uint8_t {
  WORKING_PORT_NONE,  // no work started from any port
  WORKING_PORT_PC,    // started a work from PC port
  WORKING_PORT_SC,    // started a work from Screen port
  WORKING_PORT_INVALID
};

enum RuntimeEnvType : uint8_t {
  RENV_TYPE_FEEDRATE,
  RENV_TYPE_HOTEND_TEMP,
  RENV_TYPE_BED_TEMP,
  RENV_TYPE_LASER_POWER,
  RENV_TYPE_ZOFFSET,
  RENV_TYPE_CNC_POWER,

  RENV_TYPE_INVALID
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

  uint32_t  laser_power_cnc_rpm; // laser_power (uint32_t)(float * 1000)

  uint32_t b;  // B axis current logical position

  uint8_t system_state;
  uint8_t addon_state;

  uint8_t executor_type;
  uint32_t cur_gcode_line;
} __packed SystemStatus_t;


class SystemService {
public:
  void Init();
  uint32_t GetSystemFault();
  uint8_t GetPeriphDeviceStatus();

  void ClearSystemFaultBit(uint32_t BitsToClear);
  void SetSystemFaultBit(uint32_t BitsToSet);

  void CheckException();
  ErrCode ThrowException(ExceptionHost h, ExceptionType t);
  ErrCode ThrowExceptionISR(ExceptionHost h, ExceptionType t);
  ErrCode ClearException(ExceptionHost h, ExceptionType t);
  ErrCode ClearExceptionByFaultFlag(uint32_t flag);

  ErrCode PauseTrigger(TriggerSource type);
  ErrCode StopTrigger(TriggerSource type, uint16_t event_opc=SSTP_INVALID_OP_CODE);
  ErrCode ResumeTrigger(TriggerSource s);
  ErrCode ResumeOver();
  ErrCode ResumeProcess();
  ErrCode StartWork(TriggerSource s);

  uint8_t MapCurrentStatusForSC();

  SysStatus FORCE_INLINE GetCurrentStatus() {
    return cur_status_;
  }

  ErrCode FORCE_INLINE SetCurrentStatus(SysStatus s) {
    if (s < SYSTAT_INVALID) {
      cur_status_ = s;
      return E_SUCCESS;
    }

    return E_PARAM;
  }

  WorkingPort FORCE_INLINE GetWorkingPort() { return work_port_;  }
  ErrCode FORCE_INLINE SetWorkingPort(WorkingPort p) {
    if (p < WORKING_PORT_INVALID) {
      work_port_ = p;
      return E_SUCCESS;
    }

    return E_PARAM;
  }

  SysStage GetCurrentStage();
  uint32_t GetFaultFlag() { return fault_flag_; }

  void CallbackOpenDoor();
  void CallbackCloseDoor();

  // event callback
  ErrCode ChangeRuntimeEnv(SSTP_Event_t &event);
  ErrCode GetRuntimeEnv(SSTP_Event_t &event);
  ErrCode SendStatus(SSTP_Event_t &event);
  ErrCode SendException(SSTP_Event_t &event);
  ErrCode SendSecurityStatus();
  ErrCode SendPause();
  ErrCode ChangeSystemStatus(SSTP_Event_t &event);
  ErrCode SendLastLine(SSTP_Event_t &event);
  ErrCode ClearException(SSTP_Event_t &event);
  ErrCode RecoverFromPowerLoss(SSTP_Event_t &event);
  ErrCode SendHomeAndCoordinateStatus(SSTP_Event_t &event);
  ErrCode GetMachineSize(SSTP_Event_t &event);

  ErrCode SendException(uint32_t fault);
  ErrCode FinishSystemStatusChange(uint8_t op_code, uint8_t result);

  ErrCode CallbackPreQS(QuickStopSource source);
  ErrCode CallbackPostQS(QuickStopSource source);

  uint32_t current_line() { return current_line_; }
  void     current_line(uint32_t line) { current_line_ = line; }
  ErrCode CheckIfSendWaitEvent();
  uint32_t hmi_cmd_timeout() {return hmi_cmd_timeout_;}
  void hmi_cmd_timeout(uint32_t time) {hmi_cmd_timeout_ = time;}

private:
  void inline resume_3dp(void);
  void inline resume_cnc(void);
  void inline resume_laser(void);
  void inline RestoreXYZ(void);

  ErrCode PreProcessStop();

  void MapFaultFlagToException(uint32_t flag, ExceptionHost &host, ExceptionType &type);

public:
  uint32_t PeriphDeviceStatus;

  bool is_waiting_gcode = false;
  bool is_laser_on = false;

private:
  uint8_t TriggleStat;
  TriggerSource pause_source_;  // record latest pause source
  TriggerSource resume_source_;
  TriggerSource stop_source_;
  StopType stop_type_;
  uint32_t fault_flag_;

  int8_t isr_exception[EXCEPTION_ISR_BUFFSER_SIZE][2];
  uint8_t isr_e_rindex_;
  uint8_t isr_e_windex_;
  uint8_t isr_e_len_;

  SysStatus cur_status_;
  WorkingPort work_port_;    // indicates we are handling Gcode from which UART

  uint32_t  current_line_;
  uint32_t hmi_cmd_timeout_;
};


extern SystemService systemservice;

#endif // #ifndef SNAPMAKER_SYSTEM_H_
