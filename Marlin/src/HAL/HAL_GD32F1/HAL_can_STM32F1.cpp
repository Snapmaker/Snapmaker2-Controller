#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "std_library/inc/stm32f10x_can.h"
#include "std_library/inc/stm32f10x_gpio.h"
#include "std_library/inc/stm32f10x_spi.h"
#include "Hal_can_STM32F1.h"
#include <libmaple/systick.h>

/**
 * Returns time (in milliseconds) since the beginning of program
 * execution. On overflow, restarts at 0.
 * @see micros()
 */
static inline uint32_t millis() {
    return systick_uptime();
}

/**
 * CanInit:Initialize the can bus
 * para ID: The ID of the target
 * para PortNum: The can bus port number, 1 or 2
 */
void CanInit() {
  CAN_InitTypeDef CAN_InitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;

  //Gpio Init
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStruct);
  CAN_InitStruct.CAN_ABOM = DISABLE;
  CAN_InitStruct.CAN_AWUM = DISABLE;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_14tq;
  CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStruct.CAN_NART = DISABLE;
  CAN_InitStruct.CAN_Prescaler = 6;
  CAN_InitStruct.CAN_RFLM = DISABLE;
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStruct.CAN_TTCM = DISABLE;
  CAN_InitStruct.CAN_TXFP = ENABLE;

  CAN_SlaveStartBank(14);
  CAN_Init(CAN1, &CAN_InitStruct);
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
  CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
  CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);

  CAN_Init(CAN2, &CAN_InitStruct);
  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN2, CAN_IT_FMP1, ENABLE);
  CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
  CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);

  CAN_FilterInitTypeDef CAN_FilterInitStruct;
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_16bit;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_FIFO0;
  /*
  CAN_FilterInitStruct.CAN_FilterIdHigh = (uint16)(0x500 << 5);
  CAN_FilterInitStruct.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = (uint16)(0x700 << 5);
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
  */
  CAN_FilterInitStruct.CAN_FilterIdHigh = (uint16)(0x000 << 5);
  CAN_FilterInitStruct.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = (uint16)(0x000 << 5);
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
  //Can1
  CAN_FilterInitStruct.CAN_FilterNumber = 0;
  CAN_FilterInit(&CAN_FilterInitStruct);
  //Can2
  CAN_FilterInitStruct.CAN_FilterNumber = 14;
  CAN_FilterInit(&CAN_FilterInitStruct);

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_Init(&NVIC_InitStruct);
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

  retry = 100;
  if(PortNum == 1) {
    while(retry--) {
      CAN_Transmit(CAN1, &TxMessage);
      tmptick = millis();
      //while pending
      do {
        if((millis() - tmptick) > 200)
          break;
        regtsr = CAN1->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0))
          return true;
        else if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }while(true);
    }
  }
  else {
    while(retry--) {
      CAN_Transmit(CAN2, &TxMessage);
      tmptick = millis();
      //while pending
      do {
        if((millis() - tmptick) > 200)
          break;
        regtsr = CAN2->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0))
          return true;
        else if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }while(true);
    }
  }
  return false;
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
bool CanSendShortPacked2(uint32_t ID, uint8_t PortNum, uint8_t FrameType, uint8_t DataLen, uint8_t *pData, uint32_t *RegStatusValue) {
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
      do {
        if((millis() - tmptick) > 500)
          break;
        regtsr = CAN1->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0))
        {
          *RegStatusValue = regtsr;
          return true;
        }
        else if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }while(true);
    }
  }
  else {
    while(retry--) {
      CAN_Transmit(CAN2, &TxMessage);
      tmptick = millis();
      //while pending
      do {
        if((millis() - tmptick) > 500)
          break;
        regtsr = CAN2->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0))
        {
          *RegStatusValue = regtsr;
          return true;
        }
        else if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }while(true);
    }
  }
  regtsr = 0xaaa;
  *RegStatusValue = regtsr;
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
  uint32_t IDE;
  uint8_t FMI;

  IDE = (uint8_t)0x04 & CAN1->sFIFOMailBox[CAN_FIFO0].RIR;
  if (IDE == CAN_Id_Standard)
  {
    *ID = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[CAN_FIFO0].RIR >> 21);
  }
  else
  {
    *ID = (uint32_t)0x1FFFFFFF & (CAN1->sFIFOMailBox[CAN_FIFO0].RIR >> 3);
  }
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
  uint32_t IDE;
  uint8_t FMI;

  IDE = (uint8_t)0x04 & CAN2->sFIFOMailBox[CAN_FIFO0].RIR;
  if (IDE == CAN_Id_Standard)
  {
    *ID = (uint32_t)0x000007FF & (CAN2->sFIFOMailBox[CAN_FIFO0].RIR >> 21);
  }
  else
  {
    *ID = (uint32_t)0x1FFFFFFF & (CAN2->sFIFOMailBox[CAN_FIFO0].RIR >> 3);
  }
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