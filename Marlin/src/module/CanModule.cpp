#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)

#include "../../HAL/HAL_GD32F1/HAL_can_STM32F1.h"
#include "../Marlin.h"
#include "temperature.h"
#include "configuration_store.h"
#include "ExecuterManager.h"
#include "Periphdevice.h"
#include "CanModule.h"
#include <EEPROM.h>
#include "../SnapScreen/Screen.h"

CanModule CanModules;

#define CANID_BROCAST (1)

#define MODULE_MASK_BITS  0x1ff00000
#define MODULE_EXECUTER_PRINT 0
#define MODULE_EXECUTER_CNC 1
#define MODULE_EXECUTER_LASER 2
#define MODULE_LINEAR 3
#define MODULE_ROTATE 4
#define MODULE_ENCLOSER 5
#define MODULE_LIGHT 6
#define MODULE_AIRCONDITIONER 7


#define MAKE_ID(MID)  ((MID << 20) & MODULE_MASK_BITS)

#define FLASH_CAN_TABLE_ADDR  (0x8000000 + 32 * 1024)

/**
 *Init:Initialize module table
 */
void CanModule::Init(void) {
  char IDS[16];
  SERIAL_ECHOLN("Module enum");
  CollectPlugModules();
  UpdateProcess();
  PrepareLinearModules();
  PrepareExecuterModules();
  
  for(int i=0;i<CanBusControlor.ModuleCount;i++) {
    sprintf(IDS, "ID:%08X", (int)CanBusControlor.ModuleMacList[i]);
    SERIAL_ECHOLN(IDS);
  }
}

/**
 *CollectPlugModules:Collect the IDs of the pluged modules
 */
void CanModule::CollectPlugModules() {
  uint32_t tmptick;
  uint32_t ID;
  int i;
  uint32_t ExeIDs[3];

  while(1) {
    if(CanSendPacked(CANID_BROCAST, IDTYPE_EXTID, 2, FRAME_REMOTE, 0, 0) == true) {
      tmptick = millis() + 1000;
      while(tmptick > millis()) {
        ;
      }
      break;
    } else {
      SERIAL_ECHOLN("Send Error");
      break;
    }
  }

  LinearModuleCount = 0;
  for(i=0;i<CanBusControlor.ModuleCount;i++) {
    if((CanBusControlor.ModuleMacList[i] & MODULE_MASK_BITS) == MAKE_ID(MODULE_LINEAR))
      LinearModuleID[LinearModuleCount++] = CanBusControlor.ModuleMacList[i];
  }

  ExecuterCount = 0;
  ExeIDs[0] = MAKE_ID(MODULE_EXECUTER_PRINT);
  ExeIDs[1] = MAKE_ID(MODULE_EXECUTER_CNC);
  ExeIDs[2] = MAKE_ID(MODULE_EXECUTER_LASER);
  for(i=0;i<CanBusControlor.ModuleCount;i++) {
    ID = (CanBusControlor.ModuleMacList[i] & MODULE_MASK_BITS);
    if((ID == ExeIDs[0]) || (ID == ExeIDs[1]) || (ID == ExeIDs[2]))
      ExecuterID[ExecuterCount++] = CanBusControlor.ModuleMacList[i];
  }
}

/**
 *PrepareLinearModules:Prepare for LinearModule
 */
void CanModule::PrepareLinearModules(void) {
  uint32_t tmptick;
  uint32_t i;
  uint32_t j;
  uint16_t tmpFuncID[9];
  uint8_t LinearAxisMark[9];
  uint8_t CanNum = 2;
  bool prepared;
  uint8_t Buff[3] = {CMD_T_CONFIG, 0x00, 0x00};
  int Pins[3] = {X_DIR_PIN, Y_DIR_PIN, Z_DIR_PIN};
  
  WRITE(X_DIR_PIN, LOW);
  WRITE(Y_DIR_PIN, LOW);
  WRITE(Z_DIR_PIN, LOW);
  
  for(i=0;i<LinearModuleCount;i++) {
    prepared = false;
    for(j=0;j<3;j++) {
      Buff[1] = j;
      WRITE(Pins[j], HIGH);
      CanBusControlor.SendLongData(CanNum, LinearModuleID[i], Buff, 2);
      tmptick = millis() + 500;
      while(tmptick > millis()) {
        if(CanBusControlor.ProcessLongPacks(RecvBuff, 5) == 5) {
          if(RecvBuff[0] == CMD_R_CONFIG_REACK) {
            if(RecvBuff[1] == 1) {
              LinearAxisMark[i] = j;
              //Axis Endstop describe,etc 0-2:xyz 3:probe min 4-6:xyz max
              LinearModuleAxis[i] = j + 4;
              LinearModuleLength[i] = (RecvBuff[2] << 8) | RecvBuff[3];
              prepared = true;
              //Limit position
              //RecvBuff[4];
              SERIAL_ECHOLNPAIR("Length:", LinearModuleLength[i], " Axis:", LinearAxisMark[i]);
            }
          }
          break;
        }
      }
      WRITE(Pins[j], LOW);
      if(prepared == true)
        break;
    }
  }

  //Get Linear module function ID
  for(i=0;i<LinearModuleCount;i++) {
    SendBuff[0] = CMD_T_REQUEST_FUNCID;
    SendBuff[1] = 0;
    CanBusControlor.SendLongData(CanNum, LinearModuleID[i], SendBuff, 2);
    tmptick = millis() + 100;
    tmpFuncID[i] = 0xffff;
    while(tmptick > millis()) {
      if(CanBusControlor.ProcessLongPacks(RecvBuff, 4) == 4) {
        if(RecvBuff[0] == CMD_R_REPORT_FUNCID) {
          tmpFuncID[i] = (RecvBuff[2] << 8) | RecvBuff[3];
          SERIAL_ECHOLNPAIR("MacIC:", LinearModuleID[i], " FuncID:", tmpFuncID[i]);
        }
        break;
      }
    }
  }

  SERIAL_ECHOLN("Bind LinearModule");

  //Bind Function ID with the message ID
  uint8_t AxisLinearCount[3] = {0, 0, 0};
  SendBuff[0] = CMD_T_CONFIG_FUNCID;
  SendBuff[1] = 0x01;
  for(i=0;i<LinearModuleCount;i++) {
    //Check if the max count of each axis reach 3 and if the FuncID is 0x0000
    if((LinearAxisMark[i] < 3) && (AxisLinearCount[LinearAxisMark[i]] < 3) && (tmpFuncID[i] == FUNC_REPORT_LIMIT)) {
      SendBuff[2] = 0;
      //Temporary for max endstops, example:first x axis is 0+3+0, the second x axis is 0+3+6 
      SendBuff[3] = LinearAxisMark[i] + 3 + AxisLinearCount[LinearAxisMark[i]] * 6;
      SendBuff[4] = (uint8_t)(tmpFuncID[i] >> 8);
      SendBuff[5] = (uint8_t)(tmpFuncID[i]);
      MsgIDTable[LinearAxisMark[i] + 3 + AxisLinearCount[LinearAxisMark[i]] * 6] = tmpFuncID[i];
      //Axis count add 1
      AxisLinearCount[LinearAxisMark[i]]++;
      LinearModuleMsgID[i] = SendBuff[3];
      CanBusControlor.SendLongData(CanNum, LinearModuleID[i], SendBuff, 6);
      SERIAL_ECHOLN(LinearAxisMark[i]);
    }
  }
  //Reserved 20 for high response
  MsgIDCount = 20;
}

/**
 *PrepareExecuterModules:Prepare for Executer module
 */
void CanModule::PrepareExecuterModules(void) {
  millis_t tmptick;
  uint32_t i;
  uint32_t j;
  int m;
  int k;
  uint16_t FuncIDCount;
  uint8_t ExecuterMark[6];
  uint8_t Buff[3] = {CMD_T_CONFIG, 0x00, 0x00};
  uint8_t CanNum = 2;
  int Pins[] = {E0_DIR_PIN};
 
  WRITE(E0_DIR_PIN, LOW);

  for(i=0;i<ExecuterCount;i++) {
    for(j=0;j<sizeof(Pins) / sizeof(Pins[0]);j++) {
      WRITE(Pins[j], HIGH);
      CanBusControlor.SendLongData(CanNum, ExecuterID[i], Buff, 3);
      
      tmptick = millis() + 100;
      //Unsed in current hardware design
      ExecuterMark[i] = 0xff;
      while(tmptick > millis())
      {
        if(CanBusControlor.ProcessLongPacks(RecvBuff, 20) == 2)
        {
          if(RecvBuff[0] == CMD_R_CONFIG_REACK) ExecuterMark[i] = RecvBuff[1];
          //SERIAL_ECHOLNPAIR("Executer Mark:", ExecuterMark[i]);
          break;
        }
      }
      WRITE(Pins[j], LOW);
      if(ExecuterMark[i] != 0xff) break;
    }
  }

  //Get Executer module function ID
  #if(1)
  FuncIDCount = 0;
  for(i=0;i<ExecuterCount;i++) {
    SendBuff[0] = CMD_T_REQUEST_FUNCID;
    SendBuff[1] = 0;
    CanBusControlor.SendLongData(CanNum, ExecuterID[i], SendBuff, 2);
    tmptick = millis() + 500;
    while(tmptick > millis()) {
      if(CanBusControlor.ProcessLongPacks(RecvBuff, 10) >= 10) {
        SERIAL_ECHOLN("S2");
        if(RecvBuff[0] == CMD_R_REPORT_FUNCID) {
          m = 2;
          for(k=0;k<RecvBuff[1];k++) {
            FuncIDList[FuncIDCount] = (RecvBuff[m] << 8) | RecvBuff[m + 1];
            MacIDofFuncID[FuncIDCount] = ExecuterID[i];
            m = m + 2;
            FuncIDCount++;
          }
        }
        break;
      }
    }
  }
  #endif

  SERIAL_ECHOLN("Get FuncID of Executer End");
  for(i=0;i<FuncIDCount;i++) {
    SERIAL_ECHOLNPAIR("FuncID:", FuncIDList[i]);
  }

  tmptick = millis() + 200;
  while(tmptick > millis());

  //Get Priority from table
  for(i=0;i<FuncIDCount;i++) {
    FuncIDPriority[i] = 15;
    for(j=0;j<sizeof(PriorityTable) / sizeof(PriorityTable[0]);j++)
    if(PriorityTable[j][0] == FuncIDList[i]) {
      FuncIDPriority[i] = PriorityTable[j][1]; 
      break;
    }
  }

  //Sort
  uint32_t tmpswapvalue;
  if(FuncIDCount > 1)
  {
    for(i=0;i<(FuncIDCount - 1);i++) {
      for(j=(i + 1);j<FuncIDCount;j++) {
        if(FuncIDPriority[i] > FuncIDPriority[j]) {
          tmpswapvalue = FuncIDPriority[i];
          FuncIDPriority[i] = FuncIDPriority[j];
          FuncIDPriority[j] = tmpswapvalue;
          tmpswapvalue = FuncIDList[i];
          FuncIDList[i] = FuncIDList[j];
          FuncIDList[j] = tmpswapvalue;
          tmpswapvalue = MacIDofFuncID[i];
          MacIDofFuncID[i] = MacIDofFuncID[j];
          MacIDofFuncID[j] = tmpswapvalue;
        }
      }
    }
  }

  //Fill MsgID Table
  for(i=0;i<FuncIDCount;i++)
    MsgIDTable[MsgIDCount++] = FuncIDList[i];
  //
  //Bind Function ID with the message ID
  SendBuff[0] = CMD_T_CONFIG_FUNCID;
  for(i=0;i<ExecuterCount;i++) {
    SendBuff[1] = 0x00;
    k = 2;
    for(j=0;j<FuncIDCount;j++) {
      if(ExecuterID[i] == MacIDofFuncID[j]) {        
        SendBuff[k++] = (uint8_t)(j >> 8);
        SendBuff[k++] = (uint8_t)(j);
        SendBuff[k++] = (uint8_t)(FuncIDList[i] >> 8);
        SendBuff[k++] = (uint8_t)(FuncIDList[i]);
        SendBuff[1]++;
      }
    }    
    CanBusControlor.SendLongData(CanNum, MacIDofFuncID[i], SendBuff, k);
  }

  for(i=0;i<MsgIDCount;i++) {
    SERIAL_ECHOLNPAIR("FuncID:", MsgIDTable[i]);
  }

  if((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_PRINT)) ExecuterHead.MachineType = MACHINE_TYPE_3DPRINT;
  else if(((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_CNC)) && (ExecuterMark[0] != 0xff)) ExecuterHead.MachineType = MACHINE_TYPE_CNC;
  else if(((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_LASER)) && (ExecuterMark[0] != 0xff)) ExecuterHead.MachineType = MACHINE_TYPE_LASER;
}

/**
 *LoadUpdateData:Load update data from flash
 *para Packindex:
 *para pData:The point to the buff 
 */
void CanModule::EraseUpdatePack(void) {
  uint32_t Address;
  FLASH_Unlock();
  Address = FLASH_UPDATE_CONTENT_INFO;
  FLASH_ErasePage(Address);
  Address = FLASH_UPDATE_CONTENT;
  FLASH_ErasePage(Address);
  FLASH_Lock();
  SERIAL_ECHOLN("Erase Flash Complete!");
}

/**
 *LoadUpdateData:Load update data from flash
 *para Packindex:
 *para pData:The point to the buff 
 */
bool CanModule::LoadUpdatePack(uint16_t Packindex, uint8_t *pData) {
  uint32_t Address;
  uint32_t Size;
  int i;
  uint16_t Packs;
  uint16_t PackSize = 128;
  Address = FLASH_UPDATE_CONTENT + 40;
  Size = *((uint32_t*)Address);
  Packs = Size / PackSize;
  if(Size % PackSize) Packs++;
  if(Packindex >= Packs) return false;
  Address = FLASH_UPDATE_CONTENT + 2048 + Packindex * PackSize;
  for(i=0;i<PackSize;i++) *pData++ = *((uint8_t*)Address++);
  return true;
}

/**
 *LoadUpdateInfo:Load update data from flash
 *para Version:Update file version
 *para StartID:
 *prar EndID:how many id type can be update when use the 
 *
 */
bool CanModule::LoadUpdateInfo(char *Version, uint16_t *StartID, uint16_t *EndID, uint32_t *Flag) {
  uint32_t Address;
  uint32_t Size;
  uint8_t Buff[33];
  Address = FLASH_UPDATE_CONTENT;
  for(int i=0;i<5;i++)
    Buff[i] = *((uint8_t*)Address++);
  if(Buff[0] != 1) return false;
  *StartID = (uint16_t)((Buff[1] << 8) | Buff[2]);
  *EndID = (uint16_t)((Buff[3] << 8) | Buff[4]);
  for(int i=0;i<32;i++)
    Version[i] = *((uint8_t*)Address++);
  Address = FLASH_UPDATE_CONTENT + 40;
  Size = *((uint32_t*)Address);
  SERIAL_ECHOLNPAIR("Size:", Size);
  Address = FLASH_UPDATE_CONTENT + 48;
  *Flag = *((uint32_t*)Address);
  SERIAL_ECHOLNPAIR("Flag:", *Flag);
  return true;
}

/**
 *Update:Send Start update process
 *para CanBum:Can port number
 *para ID:Module ID
 *para Version:Update file version
 *return :true if update success, or else false
 */
bool CanModule::UpdateModule(uint8_t CanNum, uint32_t ID, char *Version, uint32_t Flag) {
  uint32_t tmptick;
  uint16_t PackIndex;
  int i;
  int j;
  int err;

  SERIAL_ECHOLNPAIR("Module Start Update:" , ID);
  tmptick = millis() + 50;
  while(tmptick > millis());

  //Step1:send update version
  i = 0;
  err = 1;
  SendBuff[i++] = CMD_T_UPDATE_REQUEST;
  SendBuff[i++] = (Flag & 1)?1:0;
  for(j=0;(j<64) && (Version[j]!=0);j++) SendBuff[i++] = Version[j];
  CanBusControlor.SendLongData(CanNum, ID, SendBuff, i);
  tmptick = millis() + 6000;
  while(tmptick > millis()) {
    if(CanBusControlor.ProcessLongPacks(RecvBuff, 2) == 2) {
      if(RecvBuff[0] == CMD_R_UPDATE_REQUEST_REACK)
      {
        if(RecvBuff[1] == 0x00) {
          SERIAL_ECHOLN("Update Reject");
          break;
        }
        else if(RecvBuff[1] == 0x01) {
          SERIAL_ECHOLN("Requested");
          err = 0;
          break;
        }   
      }
    }
  }
  if(err) {
    SERIAL_ECHOLNPAIR("Module Update Fail:" , ID);
    return false;
  }

  //Step2:send update content
  tmptick = millis() + 2000;
  while(1) {
    if(CanBusControlor.ProcessLongPacks(RecvBuff, 4) == 4) {
      if(RecvBuff[0] == CMD_R_UPDATE_PACK_REQUEST) {
        PackIndex = (uint16_t)((RecvBuff[2] << 8) | RecvBuff[3]);
        if(LoadUpdatePack(PackIndex, &SendBuff[2]) == true) {
          SendBuff[0] = CMD_T_UPDATE_PACKDATA;
          SendBuff[1] = 0;
          CanBusControlor.SendLongData(CanNum, ID, SendBuff, 128 + 2);
        } else {
          SendBuff[0] = CMD_T_UPDATE_END;
          SendBuff[1] = 0;
          CanBusControlor.SendLongData(CanNum, ID, SendBuff, 2);
          break;
        }
        //delay = millis() + 500;
        //while(delay > millis());
        tmptick = millis() + 200;
      }
    }
    if(millis() > tmptick) {
      SERIAL_ECHOLNPAIR("Module Update Fail:" , ID);
      return false;
    }
  }
  tmptick = millis() + 20;
  while(tmptick > millis());
  SERIAL_ECHOLNPAIR("Module Update Complete:" , ID);

  return true;
}

/**
 *UpdateProcess:
 */
void CanModule::UpdateProcess(void)
{
  int i;
  millis_t tmptick;
  uint32_t UpdateFlag;
  uint16_t StartID, EndID, CurTypeID;
  uint8_t CanNum;

  CanNum = 0;
  char Version[64];
  //Load Update infomation and check if it is the module update file
  if(LoadUpdateInfo(Version, &StartID, &EndID, &UpdateFlag) == true) {
    SERIAL_ECHOLNPAIR("Version:", Version, "  StartID", StartID, "  EndID:", EndID);
    //Ergodic all modules which are suitable for updating
    for(i=0;i<CanBusControlor.ModuleCount;i++) {
      CurTypeID = CanBusControlor.ModuleMacList[i] & MODULE_MASK_BITS;
      if((CurTypeID >= StartID) && (CurTypeID <= EndID)) {
        UpdateModule(CanNum, CanBusControlor.ModuleMacList[i], Version, UpdateFlag);
      } else {
        
      }
    }
    EraseUpdatePack();
    HMI.SendUpdateComplete(1);
    tmptick = millis() + 2000;
    while(tmptick > millis());
  }
}

/**
 *UpdateEndstops:Update endstop from can
 */
int CanModule::UpdateEndstops(uint8_t *pBuff) {
  uint16_t MsgID;
  uint16_t index;
  
  MsgID = (uint16_t)((pBuff[0] << 8) | pBuff[1]);
  for(int i=0;i<sizeof(LinearModuleMsgID) / sizeof(LinearModuleMsgID[0]);i++) {
    if(LinearModuleMsgID[i] == MsgID) {
      index = LinearModuleAxis[i];
      Endstop |= (1 << index);      
      if(pBuff[2] == 0) Endstop &= ~(1 << index);
      //SERIAL_ECHOLNPAIR("Value:", Endstop);
      break;
    }
  }
  return 0;
}

/**
 *UpdateTemperature:Update temperature from can
 */
int CanModule::UpdateTemperature(uint8_t *pBuff) {
  ExecuterHead.temp_hotend[0] = (pBuff[0] << 8) | pBuff[1];
  ExecuterHead.CanTempMeasReady = true;
  return 0;
}

/**
 *UpdateCNCRPM:Update CNC RPM from can
 */
int CanModule::UpdateCNCRPM(uint8_t *pBuff) {
  ExecuterHead.CNC.RPM = (pBuff[0] << 8) | pBuff[1];
  return 0;
}

/**
 *SetExecuterFan1:Set executer fan1
 */
int CanModule::SetExecuterFan1(uint8_t Speed) {
  return 0;
}

/**
 *SetExecuterFan1:Set executer fan1
 */
int CanModule::SetExecuterFan2(uint8_t Speed) {
  return 0;
}

/**
 *SetExecuterFan1:Set executer fan1
 */
int CanModule::SetExecuterTemperature(uint16_t TargetTemperature) {
  return 0;
}


#endif // ENABLED CANBUS_SUPPORT
