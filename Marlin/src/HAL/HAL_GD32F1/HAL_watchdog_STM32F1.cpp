#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "std_library/inc/stm32f10x_iwdg.h"
#include "std_library/inc/stm32f10x_rcc.h"
#include "std_library/inc/misc.h"
#include "HAL_watchdog_STM32F1.h"

/**
 * WatchDogInit:
 */
void WatchDogInit() 
{
  //Reset Time = 1 / (40K / Prescaler / Reload)
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	IWDG_SetReload(200);
	IWDG_ReloadCounter();
	IWDG_Enable();
}


#endif // def __GD32F1__
