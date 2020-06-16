#if defined(__GD32F1__)
// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "HAL_LaserPwm_STM32F1.h"
#include "std_library/inc/stm32f10x_gpio.h"
#include "std_library/inc/stm32f10x_tim.h"
#include "std_library/inc/stm32f10x_rcc.h"

/**
 *Tim1PwmInit:Initialize Tim1 for laser pwm
 */
void Tim1PwmInit() {
	TIM_DeInit(TIM1);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
  
	//100HZ  ，255  Level
  #if(0)
  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Only for Snapmaker1
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIOB->BRR = GPIO_Pin_10;
  #else

  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
  #endif

	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 1862;//959;//1862;//1344;
	TIM_TimeBaseInitStruct.TIM_Period = 255;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
	TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Repetitive);

	TIM_BDTRStructInit(&TIM_BDTRInitStruct);
	TIM_BDTRInitStruct.TIM_OSSRState = ENABLE;
	TIM_BDTRInitStruct.TIM_OSSIState = ENABLE;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStruct.TIM_DeadTime = 0;
	TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);

  #if(0)
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
  
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
  #else
  TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
  TIM_OC4Init(TIM1, &TIM_OCInitStruct);
  #endif
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);

  //Serial initialize
  //USART_InitTypeDef USART_InitStruct;
  //USART_InitStruct.
  //USART_Init(USART3, &USART_InitStruct);
}

void Tim1SetCCR1(uint16_t Value) {
  TIM1->CCR1 = Value;
}

void Tim1SetCCR2(uint16_t Value) {
  TIM1->CCR2 = Value;
}

void Tim1SetCCR3(uint16_t Value) {
  TIM1->CCR3 = Value;
}

void Tim1SetCCR4(uint16_t Value) {
  TIM1->CCR4 = Value;
}

uint16_t Tim1GetCCR4(void) {
	return TIM1->CCR4;
}

#endif
