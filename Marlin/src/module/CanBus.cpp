#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)

#include "../../HAL/HAL_GD32F1/HAL_can_STM32F1.h"
#include "../Marlin.h"
#include "temperature.h"
#include "configuration_store.h"
#include "ExecuterManager.h"
#include "Periphdevice.h"
#include "CanBus.h"

CanBus CanBusControlor;

/**
 * Init:Initialize the Can controlor
 */
void CanBus::Init()
{
  CanInit();
}

/**
 * WaitReply:Wait reply from the specific ID
 * para CanNum:Can port number, 1 or 2
 * para ID:The specific ID
 * para pData:Datas to be send
 * para Len:How many datas to be send, max is 8
 * para timeout:How long to wait, in millisecond
 * return : true if reply has been received, or else false
 */
bool CanBus::WaitReply(uint8_t CanNum, uint32_t ID, uint8_t *pData, uint32_t Len, millis_t timeout)
{
  millis_t tmptick;
  RequestReplied = false;
  RequestReplyCAN = CanNum;
  RequestReplyID = ID;
  CanSendShortPacked(ID, CanNum, FRAME_DATA, Len, pData);
  tmptick = millis() + timeout;
  while(tmptick > millis())
  {
    if(RequestReplied == true)
      return true;
  }
  return false;
}

/**
 * SendData:Send data frame to the specific ID
 * para CanNum:Can port number, 1 or 2
 * para ID:The specific ID
 * para pData:Datas to be send
 * para Len:How many datas to be send, max is 8
 * return : true if success, or else false
 */
bool CanBus::SendData(uint8_t CanNum, uint32_t ID, uint8_t *pData, uint32_t Len)
{
  return CanSendShortPacked(ID, CanNum, FRAME_DATA, Len, pData);
}

/**
 * SendData:Send data frame to the specific ID
 * para CanNum:Can port number, 1 or 2
 * para ID:The specific ID
 * para pData:Datas to be send
 * para Len:How many datas to be send, max is 8
 * return : true if success, or else false
 */
bool CanBus::SendData(uint8_t CanNum, uint32_t ID, uint8_t *pData, uint32_t Len, uint32_t *Err)
{
  return CanSendShortPacked2(ID, CanNum, FRAME_DATA, Len, pData, Err);
}

/**
 * CheckReplay:Send data frame to the specific ID
 * para CanNum:Can port number, 1 or 2
 * para ID:The specific ID
 */
void CanBus::CheckReplay(uint8_t CanNum, uint32_t ID)
{
  RequestReplyCAN = CanNum;
  if(ID == RequestReplyID)
    RequestReplied = true;
  RequestReplyID = 0xffffffff;
}

extern "C"
{
void __irq_can1_tx(void)
{
}

void __irq_can1_rx0(void)
{
  strCanData tmpData;
  uint8_t Len;
  uint8_t FMI;
  FMI = Canbus1ParseData(&tmpData.ID, &tmpData.FrameType, tmpData.Data, &Len);

  if(FMI == 0)
  {
    switch(tmpData.ID)
    {
      case CAN_IDS_TEMP_CONTROL:
        ExecuterHead.temp_hotend[tmpData.Data[0]] = (tmpData.Data[1] << 8) | tmpData.Data[2];
        ExecuterHead.CanTempMeasReady = true;
      break;

      case CAN_IDS_SWTICH:
        Periph.IOLevel &= ~(1 << tmpData.Data[0]);
        Periph.IOLevel |= 1 << tmpData.Data[0];
      break;

      case CAN_IDS_LASER:
        //index,only one laser executer support currently
        tmpData.Data[0] = tmpData.Data[0];
        //opcode:0x01 for loading focus height
        if(tmpData.Data[1] == 0x03)
          ExecuterHead.Laser.FocusHeight = (float)((tmpData.Data[2] << 24) | (tmpData.Data[3] << 16) | (tmpData.Data[4] << 8) | (tmpData.Data[5])) / 1000.0f;
        CanBusControlor.CheckReplay(1, tmpData.ID);
      break;

      case CAN_IDS_DCMOTOR:
        //index, only on cnc executer support currently
        tmpData.Data[0] = tmpData.Data[0];
        ExecuterHead.CNC.RPM = (float)((tmpData.Data[1] << 8) | tmpData.Data[2]);
        CanBusControlor.CheckReplay(1, tmpData.ID);
      break;
    }
  }
  else
  {
  }
}

void __irq_can1_rx1(void)
{
}

void __irq_can1_sce(void)
{
}

void __irq_can2_tx(void)
{
}

void __irq_can2_rx0(void)
{
  strCanData tmpData;
  uint8_t Len;
  uint8_t FMI;
  FMI = Canbus2ParseData(&tmpData.ID, &tmpData.FrameType, tmpData.Data, &Len);  
  if(FMI == 0)
  {
    switch(tmpData.ID)
    {
      case CAN_IDS_BC:
        if(tmpData.Data[0] == 0x01)
        {
          ExecuterHead.MachineType = tmpData.Data[1];
        }
      break;

      case CAN_IDS_TEMP_CONTROL:
        //if(tmpData.Data[0] < 5)
        {
          ExecuterHead.temp_hotend[tmpData.Data[1]] = (uint16_t)((tmpData.Data[2] << 8) | tmpData.Data[3]);
          ExecuterHead.CanTempMeasReady = true;
        }
      break;
      
      case CAN_IDS_SWTICH:
        Periph.IOLevel |= (1 << tmpData.Data[1]);
        if(tmpData.Data[2] == 0)
          Periph.IOLevel &= ~(1 << tmpData.Data[1]);
      break;

      case CAN_IDS_LASER:
      break;
    }
  }
  else
  {
  }
}

void __irq_can2_rx1(void)
{
}

void __irq_can2_sce(void)
{
}

}

#endif // ENABLED CANBUS_SUPPORT