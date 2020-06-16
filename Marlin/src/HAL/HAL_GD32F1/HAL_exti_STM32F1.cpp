#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "std_library/inc/stm32f10x_gpio.h"
#include "std_library/inc/stm32f10x_rcc.h"
#include "std_library/inc/stm32f10x_exti.h"
#include "std_library/inc/misc.h"
#include "HAL_exti_STM32F1.h"

EXTITrigger_TypeDef Edge[] = {EXTI_Trigger_Falling, EXTI_Trigger_Rising, EXTI_Trigger_Rising_Falling};

const uint8_t PortSource[] = {
  GPIO_PortSourceGPIOA, 
  GPIO_PortSourceGPIOB, 
  GPIO_PortSourceGPIOC, 
  GPIO_PortSourceGPIOD, 
  GPIO_PortSourceGPIOE,
  GPIO_PortSourceGPIOF,
  GPIO_PortSourceGPIOG
};

const uint8_t EXTI_IRQN[] = {
  EXTI0_IRQn,
  EXTI1_IRQn,
  EXTI2_IRQn,
  EXTI3_IRQn,
  EXTI4_IRQn,
  EXTI9_5_IRQn,
  EXTI9_5_IRQn,
  EXTI9_5_IRQn,
  EXTI9_5_IRQn,
  EXTI9_5_IRQn,
  EXTI15_10_IRQn,
  EXTI15_10_IRQn,
  EXTI15_10_IRQn,
  EXTI15_10_IRQn,
  EXTI15_10_IRQn,
  EXTI15_10_IRQn
};

uint32_t PortAddress[] = {
  GPIOA_BASE,
  GPIOB_BASE,
  GPIOC_BASE,
  GPIOD_BASE,
  GPIOE_BASE,
  GPIOF_BASE,
  GPIOG_BASE
};

/**
* ExtiInit:Exti Interrup INIT_AUTO_FAN_PIN
* para PortIndex:GPIOA-GPIOI
* para PinIndex:0-15
* para RisingFallingEdge:0-3,0:Falling, 1:Rising, 2:Rising and falling
*/
void ExtiInit(uint8_t PortIndex, uint8_t PinIndex, uint8_t RisingFallingEdge) 
{
  GPIO_TypeDef *Port;
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;

  Port = (GPIO_TypeDef*)PortAddress[PortIndex];
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Pin = 1 << PinIndex;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(Port, &GPIO_InitStruct);
  
  EXTI_InitStruct.EXTI_Trigger = Edge[RisingFallingEdge];
  EXTI_InitStruct.EXTI_Line = (1 << PinIndex);
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);

  GPIO_EXTILineConfig(PortIndex, PinIndex);

  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannel = EXTI_IRQN[PinIndex];
  NVIC_Init(&NVIC_InitStruct);
}


#endif // def __GD32F1__