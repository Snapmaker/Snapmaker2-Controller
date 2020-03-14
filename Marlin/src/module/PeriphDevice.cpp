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
#include "../snap_module/snap_dbg.h"

PeriphDevice Periph;

/**
 * Init
 */
void PeriphDevice::Init()
{
  uint8_t i;
  millis_t delay_100ms = millis() + 100;

  cb_state_ = CHAMBER_STA_NONE;
  lock_uart_ = false;
  IOSwitch = 0;
  online_ = 0;

  if (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
    return;

  // check if chamber is exist and if door is opened or not
  for (i = 0; i < CanBusControlor.ExtendModuleCount; i++) {
    if (CanBusControlor.ExtendModuleMacList[i] & MAKE_ID(MODULE_ENCLOSER)) {
      LOG_I("Chamber is connected\n");

      // query door state
      CanModules.SetFunctionValue(EXTEND_CAN_NUM, FUNC_REPORT_ENCLOSURE, NULL, 0);

      // set online flag
      SBI(online_, PERIPH_IOSW_DOOR);

      // enable door checking by defualt
      SBI(IOSwitch, PERIPH_IOSW_DOOR);

      // delay to get door state
      while (PENDING((millis()), delay_100ms));

      // init chamber state per current state
      if (TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)) {
        LOG_I("door is opened!\n");
        ExecuterHead.CallbackOpenDoor();
        SystemStatus.CallbackOpenDoor();
        cb_state_ = CHAMBER_STA_OPEN;
      }
      break;
    }
  }

}

#if ENABLED(CAN_FAN)
/**
 * SetEnclosureFanSpeed:Set enclosure fan speed
 * para DelayTime:
 * para index:
 * para percent:
 */
void PeriphDevice::SetEnclosureFanSpeed(uint8_t s_value)
{
  uint8_t Data[2];

  Data[0] = 0;
  Data[1] = s_value;
  CanModules.SetFunctionValue(EXTEND_CAN_NUM, FUNC_SET_FAN_MODULE, Data, 2);
}

#endif

#if ENABLED(DOOR_SWITCH)

bool PeriphDevice::IsDoorOpened() {
  #if DISABLED(CAN_ENCLOSURE)
    return TEST(IOSwitch, PERIPH_IOSW_DOOR) && READ(DOOR_PIN));
  #else
    return TEST(IOSwitch, PERIPH_IOSW_DOOR) && TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE);
  #endif
}

/**
 * SetDoorCheck:enable or disable Door Sensor
 * para Enable:true enable ,false disable
 */
void PeriphDevice::SetDoorCheck(bool Enable) {
  if (Enable && !TEST(IOSwitch, PERIPH_IOSW_DOOR)) {
    LOG_I("enable door checking!\n");

    SBI(IOSwitch, PERIPH_IOSW_DOOR);

    if (TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)) {
      LOG_I("door is opened!\n");
      ExecuterHead.CallbackOpenDoor();
      SystemStatus.CallbackOpenDoor();
      cb_state_ = CHAMBER_STA_OPEN;
    }
  }
  else if (!Enable && TEST(IOSwitch, PERIPH_IOSW_DOOR)) {
    LOG_I("disable door checking!\n");

    CBI(IOSwitch, PERIPH_IOSW_DOOR);

    if (TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)) {
      switch (cb_state_) {
      case CHAMBER_STA_OPEN:
      case CHAMBER_STA_OPEN_HANDLED:
        ExecuterHead.CallbackCloseDoor();
        SystemStatus.CallbackCloseDoor();
        break;

      default:
        break;
      }
    }

    cb_state_ = CHAMBER_STA_NONE;
  }
}

/**
 * set new latest enclosure event
 * para percent:
 */
#endif //ENABLED(DOOR_SWITCH)

void PeriphDevice::CheckChamberDoor() {
  if (!TEST(IOSwitch, PERIPH_IOSW_DOOR))
    return;

  switch (cb_state_) {
  case CHAMBER_STA_NONE:
    // door is just opened
    if(TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)) {
      LOG_I("door opened!\n");
      ExecuterHead.CallbackOpenDoor();
      SystemStatus.CallbackOpenDoor();
      cb_state_ = CHAMBER_STA_OPEN;
    }
    break;

  case CHAMBER_STA_OPEN:
    cb_state_ = CHAMBER_STA_OPEN_HANDLED;
    break;

  case CHAMBER_STA_OPEN_HANDLED:
    // query if door is closed
    if (!TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)) {
      LOG_I("door closed!\n");
      ExecuterHead.CallbackCloseDoor();
      SystemStatus.CallbackCloseDoor();
      cb_state_ = CHAMBER_STA_CLOSED;
    }
    break;

  case CHAMBER_STA_CLOSED:
    cb_state_ = CHAMBER_STA_NONE;
    break;

  default:
    LOG_E("invalid chamber door state!\n");
    break;
  }
}

void PeriphDevice::SetUartLock(bool f) {
  lock_uart_ = f;
  if (f) {
    next_ms_ = millis() + 1000;
    SERIAL_ECHOLN("Lock Uart");
  }
  else {
    next_ms_ = 0;
    SERIAL_ECHOLN("Unlock Uart");
  }
}

void PeriphDevice::TellUartState() {
  if (!GetHoldUart())
    return;

  millis_t  now = millis();

  if (ELAPSED(now, next_ms_)) {
    SERIAL_ECHOLN(";Locked UART");
    next_ms_ = millis() + 1000;
  }
}

void PeriphDevice::ReportStatus() {
  if (TEST(IOSwitch, PERIPH_IOSW_DOOR)) {
    SERIAL_ECHOLN("Enclosure: On");
    SERIAL_ECHO("Door: ");
    SERIAL_ECHOLN(TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)? "Open" : "Closed");
  }
  else {
    SERIAL_ECHOLN("Enclosure: Off");
  }
}

void PeriphDevice::TriggerDoorEvent(bool open) {
  if (open)
    SBI(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE);
  else
    CBI(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE);
}

void PeriphDevice::Process() {
  CheckChamberDoor();

  TellUartState();
}
