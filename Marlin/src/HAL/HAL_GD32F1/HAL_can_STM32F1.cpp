#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "std_library/inc/stm32f10x_conf.h"
#include "HAL_can_STM32F1.h"
#include <libmaple/systick.h>

#include "MapleFreeRTOS1030.h"

static SemaphoreHandle_t can_lock = NULL;

/**
 * Returns time (in milliseconds) since the beginning of program
 * execution. On overflow, restarts at 0.
 * @see micros()
 */
static inline uint32_t millis() {
    return systick_uptime();
}

/**
 * CanInitFilter:Initialize the can bus filter
 * para ID: The ID of the target
 * para PortNum: The can bus port number, 1 or 2
 */
void CanInitFilter() {
  uint32_t FilterValue;
  uint32_t FilterMask;
  uint32_t FilterID;
  CAN_FilterInitTypeDef CAN_FilterInitStruct;

  can_lock = xSemaphoreCreateMutex();
  configASSERT(can_lock);
  
  CAN_SlaveStartBank(24);

  //Extent and remote frame for collect modules
  //FilterID = (1 << 28);
  FilterID = 1;
  FilterValue = CAN_ID_EXT | CAN_RTR_REMOTE | (FilterID << 3);
  FilterMask = (1<<1) | (1<<2) | (1 << 3);
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_FIFO1;
  CAN_FilterInitStruct.CAN_FilterIdHigh = (uint16_t)(FilterValue >> 16);
  CAN_FilterInitStruct.CAN_FilterIdLow = (uint16_t)FilterValue;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = (uint16_t)(FilterMask >> 16);
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = (uint16_t)FilterMask;
  //Can2
  CAN_FilterInitStruct.CAN_FilterNumber = 24;
  CAN_FilterInit(&CAN_FilterInitStruct);
  //Can1
  CAN_FilterInitStruct.CAN_FilterNumber = 0;
  CAN_FilterInit(&CAN_FilterInitStruct);

  //Extent and data frame for module long pack
  FilterID = 1;
  FilterValue = CAN_ID_EXT | CAN_RTR_DATA | (FilterID << 3);
  FilterMask = (1<<1) | (1<<2) | (1 << 3);
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_FIFO1;
  CAN_FilterInitStruct.CAN_FilterIdHigh = (uint16_t)(FilterValue >> 16);
  CAN_FilterInitStruct.CAN_FilterIdLow = (uint16_t)FilterValue;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = (uint16_t)(FilterMask >> 16);
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = (uint16_t)FilterMask;
  //Can2
  CAN_FilterInitStruct.CAN_FilterNumber = 25;
  CAN_FilterInit(&CAN_FilterInitStruct);
  //Can1
  CAN_FilterInitStruct.CAN_FilterNumber = 1;
  CAN_FilterInit(&CAN_FilterInitStruct);

  //Stander and data frame for fix id, 0-31
  FilterID = 0x600;
  FilterValue = CAN_ID_STD | CAN_RTR_DATA | (FilterID << 21);
  FilterMask = (1<<1) | (1<<2) | (0x600 << 21);
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStruct.CAN_FilterIdHigh = (uint16_t)(FilterValue >> 16);
  CAN_FilterInitStruct.CAN_FilterIdLow = (uint16_t)FilterValue;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = (uint16_t)(FilterMask >> 16);
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = (uint16_t)FilterMask;
  //Can2
  CAN_FilterInitStruct.CAN_FilterNumber = 26;
  CAN_FilterInit(&CAN_FilterInitStruct);
  //Can1
  CAN_FilterInitStruct.CAN_FilterNumber = 2;
  CAN_FilterInit(&CAN_FilterInitStruct);
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
  CAN_InitStruct.CAN_NART = ENABLE;
  CAN_InitStruct.CAN_Prescaler = 6;
  CAN_InitStruct.CAN_RFLM = DISABLE;
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStruct.CAN_TTCM = DISABLE;
  CAN_InitStruct.CAN_TXFP = ENABLE;

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

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = CAN2_RX1_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_Init(&NVIC_InitStruct);

  CanInitFilter();
}


/**
 * CanSendPacked:Can bus send data
 * para ID: The ID of the target
 * para IDType: The ID Type, standar(IDTYPE_STD) or externsion(IDTYPE_EXT)
 * para PortNum: The can bus port number, 1 or 2
 * para FrameType: The frame type,etc remote control or data
 * para pData: The pointer to the data
 * para DataLen: The count of the data to be send
 * return : true if success, or else false.  
 */
bool CanSendPacked(uint32_t ID, uint8_t IDType, uint8_t PortNum, uint8_t FrameType, uint8_t DataLen, uint8_t *pData) {
  CanTxMsg TxMessage;
  uint8_t retry;
  uint32_t tmptick;
  uint32_t regtsr;

  BaseType_t ret = pdFAIL;

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
  if(IDTYPE_STDID == IDType)
  {
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.StdId = ID;
  }
  else
  {
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.ExtId = ID;
  }

  retry = 1;
  if(PortNum == 1) {
    while(retry--) {

      if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        ret = xSemaphoreTake(can_lock, portMAX_DELAY);

      CAN_Transmit(CAN1, &TxMessage);

      if (ret == pdPASS)
        xSemaphoreGive(can_lock);

      tmptick = millis();
      //while pending
      do {
        if((millis() - tmptick) > 500)
          break;
        regtsr = CAN1->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0)) {
          return true;
        }
        else if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }while(true);
    }
  }
  else {
    while(retry--) {
      if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        ret = xSemaphoreTake(can_lock, portMAX_DELAY);

      CAN_Transmit(CAN2, &TxMessage);

      if (ret == pdPASS)
        xSemaphoreGive(can_lock);

      tmptick = millis();
      //while pending
      do {
        if((millis() - tmptick) > 500)
          break;
        regtsr = CAN2->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0)) {
          return true;
        }
        else if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }while(true);
    }
  }
  return false;
}

/**
 * CanSendPacked:Can bus send data
 * para ID: The ID of the target
 * para PortNum: The can bus port number, 1 or 2
 * para FrameType: The frame type,etc remote control or data
 * para pData: The pointer to the data
 * para DataLen: The count of the data to be send
 * return : true if success, or else false.  
 */
bool CanSendPacked2(uint32_t ID, uint8_t PortNum, uint8_t FrameType, uint8_t DataLen, uint8_t *pData, uint32_t *RegStatusValue) {
  CanTxMsg TxMessage;
  uint8_t retry;
  uint32_t tmptick;
  uint32_t regtsr;

  BaseType_t ret = pdFAIL;

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
      /* must call this function at one task */
      if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        ret = xSemaphoreTake(can_lock, portMAX_DELAY);

      CAN_Transmit(CAN1, &TxMessage);

      if (ret == pdPASS)
        xSemaphoreGive(can_lock);

      tmptick = millis();
      //while pending
      do {
        if((millis() - tmptick) > 10)
          break;
        regtsr = CAN1->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
        if(regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0))
        {
          *RegStatusValue = regtsr;
          return true;
        }
        if(regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
          break;
      }while(true);
    }
  }
  else {
    while(retry--) {

      if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        ret = xSemaphoreTake(can_lock, portMAX_DELAY);

      CAN_Transmit(CAN2, &TxMessage);

      if (ret == pdPASS)
        xSemaphoreGive(can_lock);

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
uint8_t Canbus1ParseData(uint32_t *ID, uint8_t *IDType, uint8_t *FrameType, uint8_t *pData, uint8_t *Len, uint8_t FIFONum) {
  uint32_t IDE;
  uint8_t FMI;

  IDE = (uint8_t)0x04 & CAN1->sFIFOMailBox[FIFONum].RIR;
  if (IDE == CAN_Id_Standard)
  {
    *ID = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[FIFONum].RIR >> 21);
    *IDType = IDTYPE_STDID;
  }
  else
  {
    *ID = (uint32_t)0x1FFFFFFF & (CAN1->sFIFOMailBox[FIFONum].RIR >> 3);
    *IDType = IDTYPE_EXTID;
  }
  *FrameType = (uint8_t)0x02 & CAN1->sFIFOMailBox[FIFONum].RIR;
  /* Get the DLC */
  *Len = (uint8_t)0x0F & CAN1->sFIFOMailBox[FIFONum].RDTR;
  /* Get the FMI */
  FMI = (uint8_t)0xFF & (CAN1->sFIFOMailBox[FIFONum].RDTR >> 8);
  /* Get the data field */
  pData[0] = (uint8_t)0xFF & CAN1->sFIFOMailBox[FIFONum].RDLR;
  pData[1] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[FIFONum].RDLR >> 8);
  pData[2] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[FIFONum].RDLR >> 16);
  pData[3] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[FIFONum].RDLR >> 24);
  pData[4] = (uint8_t)0xFF & CAN1->sFIFOMailBox[FIFONum].RDHR;
  pData[5] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[FIFONum].RDHR >> 8);
  pData[6] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[FIFONum].RDHR >> 16);
  pData[7] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[FIFONum].RDHR >> 24);

  //only use FIFO0
  if(FIFONum == 0) CAN1->RF0R |= CAN_RF0R_RFOM0;
  else CAN1->RF1R |= CAN_RF1R_RFOM1;
  return FMI;
}

/**
 * Canbus1ParseData:Canbus 2 parse the for ISR
 * para ID: The pointer to save the ID
 * para FrameType: The pointer to save the frame type,etc remote control or data
 * para pData: The pointer to the data
 * para Len: The pointer to the Len
 * para FIFONum: The FIFO number
 * return : The filter index
 */
uint8_t Canbus2ParseData(uint32_t *ID, uint8_t *IDType, uint8_t *FrameType, uint8_t *pData, uint8_t *Len, uint8_t FIFONum) {
  uint32_t IDE;
  uint8_t FMI;

  IDE = (uint8_t)0x04 & CAN2->sFIFOMailBox[FIFONum].RIR;
  if (IDE == CAN_Id_Standard)
  {
    *ID = (uint32_t)0x000007FF & (CAN2->sFIFOMailBox[FIFONum].RIR >> 21);
    *IDType = IDTYPE_STDID;
  }
  else
  {
    *ID = (uint32_t)0x1FFFFFFF & (CAN2->sFIFOMailBox[FIFONum].RIR >> 3);
    *IDType = IDTYPE_EXTID;
  }
  *FrameType = (uint8_t)0x02 & CAN2->sFIFOMailBox[FIFONum].RIR;
  /* Get the DLC */
  *Len = (uint8_t)0x0F & CAN2->sFIFOMailBox[FIFONum].RDTR;
  /* Get the FMI */
  FMI = (uint8_t)0xFF & (CAN2->sFIFOMailBox[FIFONum].RDTR >> 8);
  /* Get the data field */
  pData[0] = (uint8_t)0xFF & CAN2->sFIFOMailBox[FIFONum].RDLR;
  pData[1] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[FIFONum].RDLR >> 8);
  pData[2] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[FIFONum].RDLR >> 16);
  pData[3] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[FIFONum].RDLR >> 24);
  pData[4] = (uint8_t)0xFF & CAN2->sFIFOMailBox[FIFONum].RDHR;
  pData[5] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[FIFONum].RDHR >> 8);
  pData[6] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[FIFONum].RDHR >> 16);
  pData[7] = (uint8_t)0xFF & (CAN2->sFIFOMailBox[FIFONum].RDHR >> 24);

  //
  if(FIFONum == 0) CAN2->RF0R |= CAN_RF0R_RFOM0;
  else CAN2->RF1R |= CAN_RF1R_RFOM1;
  return FMI;
}


#endif // def __GD32F1__