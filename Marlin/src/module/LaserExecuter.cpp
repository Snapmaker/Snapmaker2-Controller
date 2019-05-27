#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "LaserExecuter.h"
#include "CanBus.h"

static const uint16_t LaserPowerTable[]=
{
  0, 
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};


/**
 * Init
 */
void LaserExecuter::Init()
{
  Tim1PwmInit();
}

/**
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserPower(float Percent)
{
  int integer;
  float decimal;
  uint16_t pwmvalue;
  LastPercent = Percent;
  integer = Percent;
  decimal = Percent - integer;
  pwmvalue = LaserPowerTable[integer] + (LaserPowerTable[integer + 1] - LaserPowerTable[integer]) * decimal;
  Tim1SetCCR2(pwmvalue);
}

/**
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserPower(uint16_t PwmValue)
{
  Tim1SetCCR2(PwmValue);
}

/**
 * LaserOff:Laser off without changing the power
 */
void LaserExecuter::LaserOff()
{
  Tim1SetCCR2(0);
}

/**
 * LaserOn:Laser on and use the last power
 */
void LaserExecuter::LaserOn()
{
  SetLaserPower(LastPercent);
}


/**
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserLowPower()
{
  SetLaserPower(0.5f);
}

#if ENABLED(EXECUTER_CANBUS_SUPPORT)
/**
 * SaveFocusHeight:Save the focus height
 * para height:the height of focus
 */
void LaserExecuter::SaveFocusHeight(uint8_t index, float height)
{
  uint8_t Data[8];
  uint32_t intheight;
  intheight = height * 1000;

  Data[0] = index;
  Data[1] = 0;
  Data[2] = (uint8_t)(intheight >> 24);
  Data[3] = (uint8_t)(intheight >> 16);
  Data[4] = (uint8_t)(intheight >> 8);
  Data[5] = (uint8_t)(intheight);
  if(CanBusControlor.SendData(1, CAN_IDS_LASER, Data, 6) == true)
  {
    FocusHeight = height;
    LastSetIndex = index;
  }
}

/**
 * SaveFocusHeight:Save the focus height
 */
void LaserExecuter::SaveFocusHeight()
{
  uint8_t Data[8];
  uint32_t intheight;
  intheight = FocusHeight * 1000;

  Data[0] = LastSetIndex;
  Data[1] = 0;
  Data[2] = (uint8_t)(intheight >> 24);
  Data[3] = (uint8_t)(intheight >> 16);
  Data[4] = (uint8_t)(intheight >> 8);
  Data[5] = (uint8_t)(intheight);
  CanBusControlor.SendData(1, CAN_IDS_LASER, Data, 6);
}

/**
 * LoadFocusHeight:Load focus height from laser executer
 * return :true if load success or else false
 */
bool LaserExecuter::LoadFocusHeight()
{
  uint8_t Data[3];

  Data[0] = LastSetIndex;
  Data[1] = 2;
  if(CanBusControlor.SendData(1, CAN_IDS_LASER, Data, 2) == true)
  {
    return CanBusControlor.WaitReply(1, CAN_IDS_LASER, Data, 2, 300);
  }
  return false;
}
#endif // ENABLED(EXECUTER_CANBUS_SUPPORT)

/**
 * SavePlatformHeight:Save the focus platform heigh
 *para height:The value of the platform height
 */
void LaserExecuter::SavePlatformHeight(float height)
{
  PlatformHeight = height;
  settings.save();
}

/**
 * LoadPlatformHeight:Load the focus height
 * para height:the height of focus
 */
void LaserExecuter::LoadPlatformHeight()
{
  settings.load();
}


