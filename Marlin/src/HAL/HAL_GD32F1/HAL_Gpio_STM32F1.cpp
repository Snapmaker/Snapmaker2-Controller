#if defined(__GD32F1__)
// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "Hal_LaserPwm_STM32F1.h"
#include "std_library/inc/stm32f10x_gpio.h"
#include "std_library/inc/stm32f10x_rcc.h"

/**
 *HAL_GPIOInit:Initialize GPIO for All Port
 */
void HAL_GPIOInit() {

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
}


#endif
