#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "PeriphDevice.h"
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

  // check if chamber is exist and if door is opened or not
  for (i = 0; i < CanBusControlor.ExtendModuleCount; i++) {
    if (CanBusControlor.ExtendModuleMacList[i] & MAKE_ID(MODULE_ENCLOSER)) {
      LOG_I("Chamber is connected\n");

      // query door state
      CanModules.SetFunctionValue(EXTEND_CAN_NUM, FUNC_REPORT_ENCLOSURE, NULL, 0);

      // set online flag
      SBI(online_, PERIPH_IOSW_DOOR);

      if (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)
        return;

      // enable door checking by defualt
      SBI(IOSwitch, PERIPH_IOSW_DOOR);

      // delay to get door state
      while (PENDING((millis()), delay_100ms));

      // init chamber state per current state
      if (TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)) {
        LOG_I("door is opened!\n");
        OpenDoorTrigger();
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
void PeriphDevice::SetEnclosureFanSpeed(uint8_t percent)
{
  uint8_t Data[2];
  uint8_t s_value = ((percent > 100) ? 100 : percent) * 255 / 100;
  Data[0] = 0;
  Data[1] = s_value;
  enclosure_fan_speed_ = percent;
  CanModules.SetFunctionValue(EXTEND_CAN_NUM, FUNC_SET_FAN_MODULE, Data, 2);
  SERIAL_ECHO("Enclosure fan power: ");
  SERIAL_PRINTLN(enclosure_fan_speed_, DEC);
}
#endif

/**
 * SetEnclosureLightPower:Set enclosure light power
 * para percent: power percentage
 */
void PeriphDevice::SetEnclosureLightPower(uint8_t percent)
{
  uint8_t s_value = ((percent > 100) ? 100 : percent) * 255 / 100;
  uint8_t Buff[8];
  Buff[0] = 1;
  Buff[1] = s_value;
  Buff[2] = s_value;
  Buff[3] = s_value;

  CanModules.SetFunctionValue(EXTEND_CAN_NUM, FUNC_SET_ENCLOSURE_LIGHT, Buff, 4);
  enclosure_light_power_ = percent;
  SERIAL_ECHO("Enclosure light power: ");
  SERIAL_PRINTLN(enclosure_light_power_, DEC);
}


#if ENABLED(DOOR_SWITCH)

bool PeriphDevice::IsDoorOpened() {
  #if DISABLED(CAN_ENCLOSURE)
    return TEST(IOSwitch, PERIPH_IOSW_DOOR) && READ(DOOR_PIN));
  #else
    return TEST(IOSwitch, PERIPH_IOSW_DOOR) && TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE);
  #endif
}

uint8_t PeriphDevice::GetEnclosureLightPower() {
    return enclosure_light_power_;
}

uint8_t PeriphDevice::GetEnclosureFanSpeed() {
    return enclosure_fan_speed_;
}


void PeriphDevice::OpenDoorTrigger() {
  ExecuterHead.CallbackOpenDoor();
  SystemStatus.CallbackOpenDoor();
  cb_state_ = CHAMBER_STA_OPEN;
}

void PeriphDevice::CloseDoorTrigger() {
  ExecuterHead.CallbackCloseDoor();
  SystemStatus.CallbackCloseDoor();
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
      OpenDoorTrigger();
    }
  }
  else if (!Enable && TEST(IOSwitch, PERIPH_IOSW_DOOR)) {
    LOG_I("disable door checking!\n");

    CBI(IOSwitch, PERIPH_IOSW_DOOR);

    if (TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)) {
      switch (cb_state_) {
      case CHAMBER_STA_OPEN:
      case CHAMBER_STA_OPEN_HANDLED:
        CloseDoorTrigger();
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
      OpenDoorTrigger();
    }
    break;

  case CHAMBER_STA_OPEN:
    cb_state_ = CHAMBER_STA_OPEN_HANDLED;
    break;

  case CHAMBER_STA_OPEN_HANDLED:
    // query if door is closed
    if (!TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)) {
      LOG_I("door closed!\n");
      CloseDoorTrigger();
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
  if (IsOnline(PERIPH_IOSW_DOOR)) {
    SERIAL_ECHOLN("Enclosure online: On");
    SERIAL_ECHO("Enclosure: ");
    SERIAL_ECHOLN(TEST(IOSwitch, PERIPH_IOSW_DOOR)? "On" : "Off");
    SERIAL_ECHO("Enclosure door: ");
    SERIAL_ECHOLN(TEST(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE)? "Open" : "Closed");
    SERIAL_ECHO("Enclosure light power: ");
    SERIAL_PRINTLN(enclosure_light_power_, DEC);
    SERIAL_ECHO("Enclosure fan power: ");
    SERIAL_PRINTLN(enclosure_fan_speed_, DEC);
  }
  else {
    SERIAL_ECHOLN("Enclosure online: Off");
  }
}

void PeriphDevice::TriggerDoorEvent(bool open) {
  if (open)
    SBI(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE);
  else
    CBI(CanModules.PeriphSwitch, CAN_IO_ENCLOSURE);
}

void PeriphDevice::CheckStatus() {
  CheckChamberDoor();

  TellUartState();
}

// callback for HMI events
ErrCode PeriphDevice::ReportEnclosureStatus(Event_t &event) {
  uint8_t buff[4];

  LOG_I("SC req enclosure sta\n");

  if (TEST(online_, PERIPH_IOSW_DOOR)) {
    buff[0] = E_SUCCESS;
  }
  else {
    buff[0] = E_FAILURE;
  }

  buff[1] = enclosure_light_power_;
  buff[2] = enclosure_fan_speed_;

  if (TEST(IOSwitch, PERIPH_IOSW_DOOR)) {
    buff[3] = 1;
  }
  else {
    buff[3] = 0;
  }

  event.length = 4;
  event.data = buff;

  return hmi.Send(event);
}

ErrCode PeriphDevice::SetEnclosureLight(Event_t &event) {
  ErrCode err = E_SUCCESS;

  if (event.length < 1) {
    LOG_E("must specify light power!\n");
    err = E_FAILURE;
    goto OUT;
  }

  SetEnclosureLightPower(event.data[0]);

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

ErrCode PeriphDevice::SetEnclosureFan(Event_t &event) {
  ErrCode err = E_SUCCESS;

  if (event.length < 1) {
    LOG_E("must specify Fan speed!\n");
    err = E_FAILURE;
    goto OUT;
  }

  SetEnclosureFanSpeed(event.data[0]);

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

ErrCode PeriphDevice::SetEnclosureDetection(Event_t &event) {
  ErrCode err = E_SUCCESS;

  if (event.length < 1) {
    LOG_E("must tell me what to do for enclosure detection!\n");
    err = E_FAILURE;
    goto OUT;
  }

  SetDoorCheck(event.data[0]);

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

