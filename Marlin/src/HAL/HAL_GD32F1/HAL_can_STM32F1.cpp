#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "std_library/inc/stm32f10x_can.h"
#include "Hal_can_STM32F1.h"

void CanInit(uint8_t PortNum) {
  CAN_InitTypeDef CAN_InitStruct;
  CAN_TypeDef *Port;
  if(PortNum == 1)
    Port = CAN1;
  else if(PortNum == 2)
    Port = CAN2;
  else
    return;
    
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

bool CanSendMessage(uint32_t ID, uint8_t PortNum, uint8_t FrameType, uint8_t DataLen, uint8_t *pData) {
  CanTxMsg TxMessage;

  if(DataLen > 8)
    return false;

  if(FrameType == FRAME_DATA)
  {
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = DataLen;
    for(int i=0;i<DataLen;i++)
      TxMessage.Data[i] = *pData++;
  }
  else if(FrameType == FRAME_REMOTE)
  {
    TxMessage.RTR = CAN_RTR_REMOTE;
    TxMessage.DLC = 0;
  }
  else
  {
    return false;
  }
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.StdId = ID;
  
  if(PortNum == 1)
    CAN_Transmit(CAN1, &TxMessage);
  else
    CAN_Transmit(CAN2, &TxMessage);

  return true;
}

#endif // def __GD32F1__