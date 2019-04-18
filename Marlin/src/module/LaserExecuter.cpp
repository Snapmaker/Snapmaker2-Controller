#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "../../HAL/HAL_GD32F1/HAL_LaserPwm_STM32F1.h"
#include "LaserExecuter.h"

static const uint16_t LaserPowerTable[]=
{
  0, 
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};


LaserExecuter Laser;

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
void LaserExecuter::SetLaserPower(uint8_t Percent)
{
  SERIAL_ECHOLNPAIR("PWM:", (uint16_t)LaserPowerTable[Percent]);
  Tim1SetCCR2((uint16_t)LaserPowerTable[Percent]);
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
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserLowPower()
{
  Tim1SetCCR2((uint16_t)30);
}

