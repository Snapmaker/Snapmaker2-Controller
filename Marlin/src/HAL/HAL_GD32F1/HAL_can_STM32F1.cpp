#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "std_library/inc/stm32f10x_can.h"
#include "std_library/inc/stm32f10x_gpio.h"
#include "Hal_can_STM32F1.h"
#include <libmaple/systick.h>

/**
 * Returns time (in milliseconds) since the beginning of program
 * execution. On overflow, restarts at 0.
 * @see micros()
 */
static inline uint32_t millis(void) {
    return systick_uptime();
}

/**
 * CanInit:Initialize the can bus
 * para ID: The ID of the target
 * para PortNum: The can bus port number, 1 or 2
 */
void CanInit(uint8_t PortNum) {
  CAN_InitTypeDef CAN_InitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;
  CAN_TypeDef *Port;
  if(PortNum == 1)
    Port = CAN1;
  else if(PortNum == 2)
    Port = CAN2;
  else
    return;

  //Gpio Init
  GPIO_PinRemapConfig(GPIO_Remap_CAN2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  CAN_StructInit(&CAN_InitStruct);
  CAN_InitStruct.CAN_ABOM = DISABLE;
  CAN_InitStruct.CAN_AWUM = DISABLE;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_15tq;
  CAN_InitStruct.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStruct.CAN_NART = DISABLE;
  CAN_InitStruct.CAN_Prescaler = 5;
  CAN_InitStruct.CAN_RFLM = DISABLE;
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStruct.CAN_TTCM = DISABLE;
  CAN_InitStruct.CAN_TXFP = ENABLE;
  CAN_Init(Port, &CAN_InitStruct);
  CAN_ITConfig(Port, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(Port, CAN_IT_FMP1, ENABLE);
  //CAN_ITConfig(Port, CAN_IT_TME, ENABLE);
  CAN_ClearITPendingBit(Port, CAN_IT_FMP0);
  CAN_ClearITPendingBit(Port, CAN_IT_FMP1);
  //CAN_ClearITPendingBit(Port, CAN_IT_TME);

  CAN_FilterInitTypeDef CAN_FilterInitStruct;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStruct.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStruct.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterNumber = 14;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInit(&CAN_FilterInitStruct);
}

/**
 * CanSendShortPacked:Can bus send data
 * para ID: The ID of the target
 * para PortNum: The can bus port number, 1 or 2
 * para FrameType: The frame type,etc remote control or data
 * para pData: The pointer to the data
 * para DataLen: The count of the data to be send
 * return : true if success, or else false.  
 */
bool CanSendShortPacked(uint32_t ID, uint8_t PortNum, uint8_t FrameType, uint8_t DataLen, uint8_t *pData) {
  CanTxMsg TxMessage;
  uint8_t retry;
  uint32_t tmptick;
  uint32_t regtsr;

  if(DataLen > 8)
    return false;

  if(FrameType == FRAME_DATA) {
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = DataLen;
    for(int i=0;i<DataLen;i++)
      TxMessage.Data[i] = *pData++;
  }
  else if(FrameType == FRAME_REMOTE) {
    TxMessage.RTR = CAN_RTR_REMOTE;
    TxMessage.DLC = 0;
  }
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.StdId = ID;

  retry = 1;
  if(PortNum == 1) {
    while(retry--) {
      CAN_Transmit(CAN1, &TxMessage);
      tmptick = millis();
      //while pending
      while(!(regtsr = (CAN1->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0)))) {
        if((millis() - tmptick) > 200)
          break;
        //transsmit ok
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0))
          return true;
        //transsmit fail
        if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }
    }
  }
  else {
    while(retry--) {
      CAN_Transmit(CAN2, &TxMessage);
      tmptick = millis();
      //while pending
      while(!(regtsr = (CAN2->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0)))) {
        if((millis() - tmptick) > 200)
          break;
        //transsmit ok
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0))
          return true;
        //transsmit fail
        if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }
    }
  }
  return false;
}

/**
 * Canbus1ParseData:Canbus 1 parse the for ISR
 * para ID: The pointer to save the ID
 * para FrameType: The pointer to save the frame type,etc remote control or data
 * para pData: The pointer to the data
 * para Len: The pointer to the Len
 * return : The filter index
 */
uint8_t Canbus1ParseData(uint32_t *ID, uint8_t *FrameType, uint8_t *pData, uint8_t *Len) {
  uint8_t FMI;
  
  *FrameType = (uint8_t)0x02 & CAN1->sFIFOMailBox[CAN_FIFO0].RIR;
  /* Get the DLC */
  *Len = (uint8_t)0x0F & CAN1->sFIFOMailBox[CAN_FIFO0].RDTR;
  /* Get the FMI */
  FMI = (uint8_t)0xFF & (CAN1->sFIFOMailBox[CAN_FIFO0].RDTR >> 8);
  /* Get the data field */
  pData[0] = (uint8_t)0xFF & CAN1->sFIFOMailBox[CAN_FIFO0].RDLR;
  pData[1] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[CAN_FIFO0].RDLR >> 8);
  pData[2] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[CAN_FIFO0].RDLR >> 16);
  pData[3] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[CAN_FIFO0].RDLR >> 24);
  pData[4] = (uint8_t)0xFF & CAN1->sFIFOMailBox[CAN_FIFO0].RDHR;
  pData[5] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[CAN_FIFO0].RDHR >> 8);
  pData[6] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[CAN_FIFO0].RDHR >> 16);
  pData[7] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[CAN_FIFO0].RDHR >> 24);

  //only use FIFO0
  CAN1->RF0R |= CAN_RF0R_RFOM0;
  return FMI;
}

/**
 * Canbus1ParseData:Canbus 2 parse the for ISR
 * para ID: The pointer to save the ID
 * para FrameType: The pointer to save the frame type,etc remote control or data
 * para pData: The pointer to the data
 * para Len: The pointer to the Len
 * return : The filter index
 */
uint8_t Canbus2ParseData(uint32_t *ID, uint8_t *FrameType, uint8_t *pData, uint8_t *Len) {
  uint8_t FMI;
  
  *FrameType = (uint8_t)0x02 & CAN2->sFIFOMailBox[CAN_FIFO0].RIR;
  /* Get the DLC */
  *Len = (uint8_t)0x0F & CAN2->sFIFOMailBox[CAN_FIFO0].RDTR;
  /* Get the FMI */
  FMI = (uint8_t)0xFF & (CAN2->sFIFOMailBox[CAN_FIFO0].RDTR >> 8);
  /* Get the data field */
  pData[0] = (uint8_t)0xFF & CAN2->sFIFOMailBox[CAN_FIFO0].RDLR;
  pData[1] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[CAN_FIFO0].RDLR >> 8);
  pData[2] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[CAN_FIFO0].RDLR >> 16);
  pData[3] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[CAN_FIFO0].RDLR >> 24);
  pData[4] = (uint8_t)0xFF & CAN2->sFIFOMailBox[CAN_FIFO0].RDHR;
  pData[5] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[CAN_FIFO0].RDHR >> 8);
  pData[6] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[CAN_FIFO0].RDHR >> 16);
  pData[7] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[CAN_FIFO0].RDHR >> 24);

  //Only use FIFO0
  CAN2->RF0R |= CAN_RF0R_RFOM0;
  return FMI;
}


#endif // def __GD32F1__