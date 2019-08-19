#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "periphdevice.h"
#include "../HAL/HAL_GD32F1/HAL_exti_STM32F1.h"
#include "CanBus.h"
#include "CanModule.h"
#include "StatusControl.h"
#include "../snap_module/error.h"

PeriphDevice Periph;

/**
 * Init
 */
void PeriphDevice::Init()
{
  latest_enclosure_event_ = ENCLOSURE_EVENT_NONE;
}

#if ENABLED(CAN_FAN)
/**
 * SetFanSpeed:Set Fan Speed
 * para DelayTime:
 * para index:
 * para percent:
 */
void PeriphDevice::SetFanSpeed(uint8_t index, uint8_t DelayTime, uint8_t s_value)
{
  uint8_t Data[3];

  Data[0] = index;
  Data[1] = DelayTime;
  Data[2] = s_value;
  FanSpeed[index] = s_value;
}
#endif

#if ENABLED(DOOR_SWITCH)

bool PeriphDevice::IsDoorOpened() {
  #if DISABLED(CAN_ENCLOSURE)
    return (IOSwitch & PERIPH_IOSW_DOOR) && READ(DOOR_PIN));
  #else
    return (IOSwitch & PERIPH_IOSW_DOOR) && TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE);
  #endif
}


/**
 * DoorSwitchInit:Initialze the door switch IO
 * para Enable:true enable ,false disable
 */
void PeriphDevice::DoorSwitchInit() {
  #if DISABLED(CAN_ENCLOSURE)
    pinMode(DOOR_PIN, INPUT_PULLUP);
  #endif
}

/**
 * SetDoorCheck:enable or disable Door Sensor
 * para Enable:true enable ,false disable
 */
void PeriphDevice::SetDoorCheck(bool Enable) {
  if(Enable) CBI(IOSwitch, PERIPH_IOSW_DOOR);
  else SBI(IOSwitch, PERIPH_IOSW_DOOR);
}

/**
 * StartDoorCheck:Start Door Sensor working
 * para percent:
 */
void PeriphDevice::StartDoorCheck() {

}

/**
 * StopDoorCheck:Stop Door Sensor working
 * para percent:
 */
void PeriphDevice::StopDoorCheck() {
}

/**
 * set new latest enclosure event
 * para percent:
 */
void PeriphDevice::LatestEnclosureEvent(EnclosureEvent e) {
  if (e < ENCLOSURE_EVENT_INVALID)
    latest_enclosure_event_ = e;
}

#endif //ENABLED(DOOR_SWITCH)

void PeriphDevice::Process() {

#if ENABLED(DOOR_SWITCH)
  if (IOSwitch & PERIPH_IOSW_DOOR) {
    if (TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE) && (latest_enclosure_event_ != ENCLOSURE_EVENT_OPEN)) {
      SystemStatus.PauseTrigger(TRIGGER_SOURCE_DOOR_OPEN);
      latest_enclosure_event_ = ENCLOSURE_EVENT_OPEN;
    }

    if (TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE) && (latest_enclosure_event_ == ENCLOSURE_EVENT_OPEN)
        && (SystemStatus.GetCurrentStatus() == SYSTAT_PAUSE_FINISH)) {
      // TODO: tell screen we have paused by door opened

      latest_enclosure_event_ = ENCLOSURE_EVENT_OPEN_FINISH;
    }

    if (!TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE) && (latest_enclosure_event_ == ENCLOSURE_EVENT_OPEN_FINISH)
          && SystemStatus.GetCurrentStatus() == SYSTAT_PAUSE_FINISH) {
      if (SystemStatus.ResumeTrigger(TRIGGER_SOURCE_DOOR_CLOSE) == E_SUCCESS)
        latest_enclosure_event_ = ENCLOSURE_EVENT_CLOSE;
    }

    if ((latest_enclosure_event_ = ENCLOSURE_EVENT_CLOSE) && (SystemStatus.GetCurrentStatus() == SYSTAT_RESUME_WAITING
          || SystemStatus.GetCurrentStage() == SYSTAGE_WORK)) {
      // TODO: tell screen we have finish reuming

      latest_enclosure_event_ = ENCLOSURE_EVENT_NONE;
    }
  }
#endif

}
