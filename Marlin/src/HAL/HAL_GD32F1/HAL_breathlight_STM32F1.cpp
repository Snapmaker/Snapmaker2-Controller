#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "std_library/inc/stm32f10x_gpio.h"
#include "std_library/inc/stm32f10x_rcc.h"
#include "std_library/inc/stm32f10x_dac.h"
#include "std_library/inc/stm32f10x_dma.h"
#include "std_library/inc/stm32f10x_tim.h"
#include "std_library/inc/misc.h"
#include "Hal_breathlight_STM32F1.h"

static const uint8_t BreathLedTable[] = {
  50,84,104,119,130,139,147,153,159,164,169,173,177,181,184,188,191,193,196,199,201,203,206,208,210,212,213,215,217,219,220,222,223,225,226,228,229,230,232,233,234,235,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,251,252,253,254,255,255,255,254,253,252,251,251,250,249,248,247,246,245,244,243,242,241,240
  ,239,238,237,235,234,233,232,230,229,228,226,225,223,222,220,219,217,215,213,212,210,208,206,203,201,199,196,193,191,188,184,181,177,173,169,164,159,153,147,139,130,119,104,84,50
};
  
/**
 * BreathLightInit:
 */
void BreathLightInit() 
{
  DAC_InitTypeDef DAC_InitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  DMA_InitTypeDef DMA_InitStruct;

  TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 120000000 / 30000 - 1;
	TIM_TimeBaseInitStruct.TIM_Period = 999;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);
	TIM_SelectOnePulseMode(TIM7, TIM_OPMode_Repetitive);
	TIM_DMACmd(TIM7, TIM_DMA_Update, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC | RCC_APB1Periph_TIM7, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  DMA_InitStruct.DMA_BufferSize = sizeof(BreathLedTable) / sizeof(BreathLedTable[0]);
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)BreathLedTable;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(DAC_BASE | 0x1C);
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA2_Channel4, &DMA_InitStruct);
	DMA_Cmd(DMA2_Channel4, ENABLE);

  DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits7_0;
  DAC_InitStruct.DAC_OutputBuffer = ENABLE;
  DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_Init(DAC_Channel_2, &DAC_InitStruct);
  DAC_Cmd(DAC_Channel_2, ENABLE);
  TIM_Cmd(TIM7, ENABLE);
}


void BreathLightClose(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  DAC_Cmd(DAC_Channel_2, DISABLE);
  DMA_Cmd(DMA2_Channel4, DISABLE);
  TIM_Cmd(TIM7, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC | RCC_APB1Periph_TIM7, DISABLE);
  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

#endif // def __GD32F1__
