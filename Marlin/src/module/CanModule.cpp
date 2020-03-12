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
#include "../libs/GenerialFunctions.h"
#include "../SnapScreen/Screen.h"
#include "motion.h"
#include "StatusControl.h"

CanModule CanModules;

uint8_t CanModule::machine_size_type = MACHINE_SIZE_UNKNOW;


#define CANID_BROCAST (1)

#define FLASH_CAN_TABLE_ADDR  (0x8000000 + 32 * 1024)

/**
 *Init:Initialize module table
 */
void CanModule::Init(void) {
  CanBusControlor.Init();
  millis_t tmptick;

  disable_power_domain(POWER_DOMAIN_1 | POWER_DOMAIN_2);
  tmptick = millis() + 500;
  while(tmptick > millis());
  enable_power_domain(POWER_DOMAIN_1 | POWER_DOMAIN_2);
  tmptick = millis() + 500;
  while(tmptick > millis());

  SERIAL_ECHOLN("Module enum");
  CollectPlugModules();
  UpdateProcess();
  PrepareLinearModules();
  PrepareRestModules();

  // query state of filament sensor
  SetFunctionValue(BASIC_CAN_NUM, FUNC_REPORT_CUT, NULL, 0);
  //PrepareExecuterModules();
  //PrepareExtendModules();
  for(int i=0;i<CanBusControlor.ModuleCount;i++) {
    SERIAL_ECHOLNPAIR("Basic ID: 0x", Value32BitToString(CanBusControlor.ModuleMacList[i]));
  }

  for(int i=0;i<CanBusControlor.ExtendModuleCount;i++) {
    SERIAL_ECHOLNPAIR("Extend ID: 0x", Value32BitToString(CanBusControlor.ExtendModuleMacList[i]));
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

  //Collect module for can1
  while(1) {
    // tell all modules to return its ID, which connect to CAN2
    if(CanSendPacked(CANID_BROCAST, IDTYPE_EXTID, BASIC_CAN_NUM, FRAME_REMOTE, 0, 0) == true) {
      // wait 1 second, then CAN RX ISR will save all connected modules
      tmptick = millis() + 1000;
      while(tmptick > millis()) {
        ;
      }
      break;
    } else {
      SERIAL_ECHOLN("Send2 Error");
      break;
    }
  }

  // CanBusControlor.ModuleCount indicates all plugged modules
  // and their ID were saved in array CanBusControlor.ModuleMacList[]
  LinearModuleCount = 0;
  for(i=0;i<CanBusControlor.ModuleCount;i++) {
    // checkout linear modules ID to LinearModuleID[] firstly
    if((CanBusControlor.ModuleMacList[i] & MODULE_MASK_BITS) == MAKE_ID(MODULE_LINEAR))
      LinearModuleID[LinearModuleCount++] = CanBusControlor.ModuleMacList[i];
  }

  ExecuterCount = 0;
  ExeIDs[0] = MAKE_ID(MODULE_EXECUTER_PRINT);
  ExeIDs[1] = MAKE_ID(MODULE_EXECUTER_CNC);
  ExeIDs[2] = MAKE_ID(MODULE_EXECUTER_LASER);
  for(i=0;i<CanBusControlor.ModuleCount;i++) {
    // checkout executors ID to ExecuterID[] secondly
    ID = (CanBusControlor.ModuleMacList[i] & MODULE_MASK_BITS);
    if((ID == ExeIDs[0]) || (ID == ExeIDs[1]) || (ID == ExeIDs[2]))
      ExecuterID[ExecuterCount++] = CanBusControlor.ModuleMacList[i];
  }

  //Collect module for can2
  while(1) {
    // tell all extended modules to return its ID, which connect to CAN1
    // CanBusControlor.ExtendModuleCount indicates total of modules
    // their ID were saved in array CanBusControlor.ExtendModuleMacList[]
    if(CanSendPacked(CANID_BROCAST, IDTYPE_EXTID, EXTEND_CAN_NUM, FRAME_REMOTE, 0, 0) == true) {
      tmptick = millis() + 1000;
      while(tmptick > millis()) {
        ;
      }
      break;
    } else {
      SERIAL_ECHOLN("Send1 Error");
      break;
    }
  }

  //Update Endstops
  //for(int i=0;i<20;i++)
  //  CanSendPacked(i, IDTYPE_STDID, BASIC_CAN_NUM, FRAME_DATA, 0, 0);
}

/**
 *PrepareLinearModules:Prepare for LinearModule
 */
void CanModule::PrepareLinearModules(void) {
  uint32_t tmptick;
  uint32_t i;
  uint32_t j;
  uint16_t tmpFuncID[9];
  uint8_t endstop_init_status[9];

  bool prepared;
  uint8_t Buff[3] = {CMD_M_CONFIG , 0x00, 0x00};
  int Pins[3] = {X_DIR_PIN, Y_DIR_PIN, Z_DIR_PIN};

  WRITE(X_DIR_PIN, LOW);
  WRITE(Y_DIR_PIN, LOW);
  WRITE(Z_DIR_PIN, LOW);
  tmpEndstopBits = 0xffffffff;

  for(i=0;i<MAX_CAN_AXES;i++) {
    LinearModuleLength[i] = 0xffff;
    LinearModuleMark[i] = 0xff;
  }

  for(i=0;i<LinearModuleCount;i++) {
    prepared = false;
    for(j=0;j<=Z_AXIS;j++) {
      Buff[1] = j;
      WRITE(Pins[j], HIGH);
      CanBusControlor.SendLongData(BASIC_CAN_NUM, LinearModuleID[i], Buff, 2);
      tmptick = millis() + 500;
      while(tmptick > millis()) {
        if(CanBusControlor.ProcessLongPacks(RecvBuff, 8) > 4) {
          if(RecvBuff[0] == CMD_S_CONFIG_REACK) {
            if(RecvBuff[1] == 1) {
              // Mark the xyz for current module
              LinearModuleMark[i] = j;
              LinearModuleT[i] = (RecvBuff[6] << 8) | RecvBuff[7];
              prepared = true;
              endstop_init_status[i] = RecvBuff[5];
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

  // Get length of all axes
  GetAxesLength();

  // Get lead of all axes
  GetAxesLead();

  X_MAX_POS = Y_MAX_POS = Z_MAX_POS = 65535;

  for(i=0;i<LinearModuleCount;i++) {
    if(LinearModuleMark[i] == X_AXIS) {
      if(LinearModuleLength[i] < 200) {
        if(endstop_init_status[i] == 0) tmpEndstopBits &= ~(1 << 4);
      }
      else if(LinearModuleLength[i] < 300) {
        if(endstop_init_status[i] == 0) tmpEndstopBits &= ~(1 << 0);
      }
      else if(LinearModuleLength[i] < 400) {
        if(endstop_init_status[i] == 0) tmpEndstopBits &= ~(1 << 0);
      }
      if(LinearModuleLength[i] < X_MAX_POS)
        X_MAX_POS = LinearModuleLength[i];
    }
    if(LinearModuleMark[i] == Y_AXIS) {
      if(LinearModuleLength[i] < Y_MAX_POS)
        Y_MAX_POS = LinearModuleLength[i];
      if(endstop_init_status[i] == 0) tmpEndstopBits &= ~(1 << 5);
    }
    if(LinearModuleMark[i] == Z_AXIS) {
      if(LinearModuleLength[i] < Z_MAX_POS)
        Z_MAX_POS = LinearModuleLength[i];
      if(endstop_init_status[i] == 0) tmpEndstopBits &= ~(1 << 6);
    }
    SERIAL_ECHOLNPAIR("Length:", LinearModuleLength[i], " Axis:", LinearModuleMark[i]);
  }

  if(X_MAX_POS == 65535)
    X_MAX_POS = 0;

  if(Y_MAX_POS == 65535)
    Y_MAX_POS = 0;

  if(Z_MAX_POS == 65535)
    Z_MAX_POS = 0;

  // 3 Axes length are the same
  if((X_MAX_POS == Y_MAX_POS) && (X_MAX_POS == Z_MAX_POS) && (X_MAX_POS > 0)) {
    if(X_MAX_POS < 200) {
      X_MAX_POS = 167;
      Y_MAX_POS = 169;
      Z_MAX_POS = 150;
      X_HOME_DIR = 1;
      X_DIR = false;
      Y_HOME_DIR = 1;
      Y_DIR = false;
      Z_HOME_DIR = 1;
      Z_DIR = false;

      LOOP_XYZ(i) home_offset[i] = s_home_offset[i];

      X_DEF_SIZE = 160;
      Y_DEF_SIZE = 160;
      Z_DEF_SIZE = 145;

      MAGNET_X_SPAN = 114;
      MAGNET_Y_SPAN = 114;

      machine_size_type = MACHINE_SIZE_S;
      SystemStatus.ClearException(EHOST_MC, ETYPE_NO_HOST);
    }
    else if(Y_MAX_POS < 300) {
      X_MAX_POS = 252;
      Y_MAX_POS = 260;
      Z_MAX_POS = 235;
      X_HOME_DIR = -1;
      X_DIR = true;
      Y_HOME_DIR = 1;
      Y_DIR = false;
      Z_HOME_DIR = 1;
      Z_DIR = false;

      LOOP_XYZ(i) home_offset[i] = m_home_offset[i];

      X_DEF_SIZE = 230;
      Y_DEF_SIZE = 250;
      Z_DEF_SIZE = 235;

      MAGNET_X_SPAN = 184;
      MAGNET_Y_SPAN = 204;

      machine_size_type = MACHINE_SIZE_M;
      SystemStatus.ClearException(EHOST_MC, ETYPE_NO_HOST);
    }
    else if(Z_MAX_POS < 400) {
      X_MAX_POS = 345;
      Y_MAX_POS = 360;
      Z_MAX_POS = 334;
      X_HOME_DIR = -1;
      X_DIR = true;
      Y_HOME_DIR = 1;
      Y_DIR = false;
      Z_HOME_DIR = 1;
      Z_DIR = false;

      LOOP_XYZ(i) home_offset[i] = l_home_offset[i];

      X_DEF_SIZE = 320;
      Y_DEF_SIZE = 350;
      Z_DEF_SIZE = 330; // unused & spec is lager than actual size.  334 - 6 = 328?

      MAGNET_X_SPAN = 274;
      MAGNET_Y_SPAN = 304;

      machine_size_type = MACHINE_SIZE_L;
      SystemStatus.ClearException(EHOST_MC, ETYPE_NO_HOST);
    }
    else {
      SystemStatus.ThrowException(EHOST_MC, ETYPE_NO_HOST);
    }
  }
  else {
    SystemStatus.ThrowException(EHOST_MC, ETYPE_NO_HOST);
  }

  SERIAL_ECHO("Machine Size:\r\n");
  SERIAL_ECHOLNPAIR("   X:", X_MIN_POS, " - ", X_MAX_POS);
  SERIAL_ECHOLNPAIR("   Y:", Y_MIN_POS, " - ", Y_MAX_POS);
  SERIAL_ECHOLNPAIR("   Z:", Z_MIN_POS, " - ", Z_MAX_POS);
  SERIAL_ECHOPAIR("Directions:\r\n");
  SERIAL_ECHOLNPAIR("   X:", X_DIR, " Y:", Y_DIR, " Z:", Z_DIR, " E:", E_DIR);
  SERIAL_ECHOPAIR("Home Directions:\r\n");
  SERIAL_ECHOLNPAIR("   X:", X_HOME_DIR, " Y:", Y_HOME_DIR, " Z:", Z_HOME_DIR);
  SERIAL_ECHOPAIR("Home offset:\r\n");
  SERIAL_ECHOLNPAIR("   X:", home_offset[X_AXIS], " Y:", home_offset[Y_AXIS], " Z:", home_offset[Z_AXIS]);

  //Get Linear module function ID
  for(i=0;i<LinearModuleCount;i++) {
    SendBuff[0] = CMD_M_REQUEST_FUNCID;
    SendBuff[1] = 0;
    CanBusControlor.SendLongData(BASIC_CAN_NUM, LinearModuleID[i], SendBuff, 2);
    tmptick = millis() + 100;
    tmpFuncID[i] = 0xffff;
    while(tmptick > millis()) {
      if(CanBusControlor.ProcessLongPacks(RecvBuff, 4) == 4) {
        if(RecvBuff[0] == CMD_S_REPORT_FUNCID) {
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
  SendBuff[0] = CMD_M_CONFIG_FUNCID;
  SendBuff[1] = 0x01;
  for(i=0;i<LinearModuleCount;i++) {
    //Check if the max count of each axis reach 3 and if the FuncID is 0x0000
    if((LinearModuleMark[i] < 3) && (AxisLinearCount[LinearModuleMark[i]] < 3) && (tmpFuncID[i] == FUNC_REPORT_LIMIT)) {
      SendBuff[2] = 0;
      //Temporary for max endstops, example:first x axis is 0+4+0, the second x axis is 0+4+7
      if(LinearModuleMark[i] == 0) {
        if(X_HOME_DIR > 0)
          SendBuff[3] = LinearModuleMark[i] + 4 + AxisLinearCount[LinearModuleMark[i]] * 7;
        else
          SendBuff[3] = LinearModuleMark[i] + AxisLinearCount[LinearModuleMark[i]] * 7;
      }
      if(LinearModuleMark[i] == 1) {
        if(Y_HOME_DIR > 0)
          SendBuff[3] = LinearModuleMark[i] + 4 + AxisLinearCount[LinearModuleMark[i]] * 7;
        else
          SendBuff[3] = LinearModuleMark[i] + AxisLinearCount[LinearModuleMark[i]] * 7;
      }
      if(LinearModuleMark[i] == 2) {
        if(Z_HOME_DIR > 0)
          SendBuff[3] = LinearModuleMark[i] + 4 + AxisLinearCount[LinearModuleMark[i]] * 7;
        else
          SendBuff[3] = LinearModuleMark[i] + AxisLinearCount[LinearModuleMark[i]] * 7;
      }
      SendBuff[4] = (uint8_t)(tmpFuncID[i] >> 8);
      SendBuff[5] = (uint8_t)(tmpFuncID[i]);
      MsgIDTable[SendBuff[3]] = tmpFuncID[i];
      //Axis count add 1
      AxisLinearCount[LinearModuleMark[i]]++;
      CanBusControlor.SendLongData(BASIC_CAN_NUM, LinearModuleID[i], SendBuff, 6);
      SERIAL_ECHOLNPAIR("Linear Module:", LinearModuleID[i], " MsgID:", SendBuff[3]);
    }
  }
  //Reserved 20 for high response
  MsgIDCount = 20;

  tmpEndstopBits = 0xffffffff;
  Endstop = 0xffffffff;
  // Update the endstops
  for (i=0;i<20;i++) {
    if (MsgIDTable[i] == FUNC_REPORT_LIMIT) {
      CanSendPacked(i, IDTYPE_STDID, BASIC_CAN_NUM, FRAME_DATA, 0, 0);
    }
  }
}

void CanModule::UpdateEndstops() {
  for (int i=0;i<20;i++) {
    if (MsgIDTable[i] == FUNC_REPORT_LIMIT) {
      CanSendPacked(i, IDTYPE_STDID, BASIC_CAN_NUM, FRAME_DATA, 0, 0);
    }
  }
}

/**
 *PrepareRestModules:Prepare for Rest module
 */
void CanModule::PrepareRestModules(void) {
  millis_t tmptick;
  uint32_t i;
  uint32_t j;
  int m;
  int k;
  int n;
  uint16_t LastFuncID;
  uint16_t FuncIDCount;
  uint8_t ExecuterMark[6];
  uint8_t Buff[3] = {CMD_M_CONFIG , 0x00, 0x00};
  int Pins[] = {E0_DIR_PIN};

  WRITE(E0_DIR_PIN, LOW);

  for (i = 0; i < ExecuterCount; i++) {
    for (j = 0; j < sizeof(Pins) / sizeof(Pins[0]); j++) {
      WRITE(Pins[j], HIGH);
      CanBusControlor.SendLongData(BASIC_CAN_NUM, ExecuterID[i], Buff, 3);

      tmptick = millis() + 100;
      //Unsed in current hardware design
      ExecuterMark[i] = 0xff;
      while (tmptick > millis())
      {
        if (CanBusControlor.ProcessLongPacks(RecvBuff, 20) == 2)
        {
          if (RecvBuff[0] == CMD_S_CONFIG_REACK) ExecuterMark[i] = RecvBuff[1];
            break;
        }
      }
      WRITE(Pins[j], LOW);
      if (ExecuterMark[i] != 0xff) break;
    }
  }

  /* Get Executer module function ID
   * all function ID of all Executor will be saved in arrray FuncIDList[].
   * for MacIDofFuncID[], its element is MAC ID, its index to index FuncIDList[] is function ID
   */
  FuncIDCount = 0;
  for (i = 0; i < ExecuterCount; i++) {
    SendBuff[0] = CMD_M_REQUEST_FUNCID;
    SendBuff[1] = 0;
    CanBusControlor.SendLongData(BASIC_CAN_NUM, ExecuterID[i], SendBuff, 2);
    tmptick = millis() + 2500;
    while (tmptick > millis()) {
      if (CanBusControlor.ProcessLongPacks(RecvBuff, 128) >= 4) {
        if (RecvBuff[0] == CMD_S_REPORT_FUNCID) {
          m = 2;
          for (k = 0; k < RecvBuff[1]; k++) {
            FuncIDList[FuncIDCount] = (RecvBuff[m] << 8) | RecvBuff[m + 1];
            MacIDofFuncID[FuncIDCount] = ExecuterID[i];
            SERIAL_ECHOLNPAIR("Basic MacID:", MacIDofFuncID[FuncIDCount], " FuncID:", FuncIDList[FuncIDCount]);
            m = m + 2;
            FuncIDCount++;
          }
        }
        break;
      }
    }
  }

  //Get Extend module function ID
  for (i = 0; i < CanBusControlor.ExtendModuleCount; i++) {
    SendBuff[0] = CMD_M_REQUEST_FUNCID;
    SendBuff[1] = 0;
    CanBusControlor.SendLongData(EXTEND_CAN_NUM, CanBusControlor.ExtendModuleMacList[i], SendBuff, 2);
    tmptick = millis() + 2500;
    while (tmptick > millis()) {
      if (CanBusControlor.ProcessLongPacks(RecvBuff, 128) >= 4) {
        if (RecvBuff[0] == CMD_S_REPORT_FUNCID) {
          m = 2;
          for (k = 0; k < RecvBuff[1]; k++) {
            FuncIDList[FuncIDCount] = (RecvBuff[m] << 8) | RecvBuff[m + 1];
            MacIDofFuncID[FuncIDCount] = CanBusControlor.ExtendModuleMacList[i];
            SERIAL_ECHOLNPAIR("Extend MacID:", MacIDofFuncID[FuncIDCount], " FuncID:", FuncIDList[FuncIDCount]);
            m = m + 2;
            FuncIDCount++;
          }
        }
        break;
      }
    }
  }

  tmptick = millis() + 200;
  while (tmptick > millis());

  /* Get Priority from table
   * PriorityTable[x][0] save Function ID, PriorityTable[x][1] save its priority.
   * FuncIDPriority[x] save the prority which corresponding to Function IDs in FuncIDList[x]
   */
  for (i = 0; i < FuncIDCount; i++) {
    FuncIDPriority[i] = 15;
    for (j = 0; j < sizeof(PriorityTable) / sizeof(PriorityTable[0]); j++)
    if (PriorityTable[j][0] == FuncIDList[i]) {
      FuncIDPriority[i] = PriorityTable[j][1];
      break;
    }
  }

  // Sort the function ID in FuncIDList[] per the priority in FuncIDPriority[]
  // After sorting, FuncIDPriority[], FuncIDList[], MacIDofFuncID[] are all sorted by priority
  uint32_t tmpswapvalue;
  if (FuncIDCount > 1)
  {
    for (i = 0; i < (FuncIDCount - 1); i++) {
      for (j = (i + 1); j < FuncIDCount; j++) {
        if(FuncIDPriority[i] > FuncIDPriority[j]) {
          // swap priority
          tmpswapvalue = FuncIDPriority[i];
          FuncIDPriority[i] = FuncIDPriority[j];
          FuncIDPriority[j] = tmpswapvalue;
          // swap function ID
          tmpswapvalue = FuncIDList[i];
          FuncIDList[i] = FuncIDList[j];
          FuncIDList[j] = tmpswapvalue;
          // swap MAC ID
          tmpswapvalue = MacIDofFuncID[i];
          MacIDofFuncID[i] = MacIDofFuncID[j];
          MacIDofFuncID[j] = tmpswapvalue;
        }
      }
    }
  }

  //Mark the start MsgID
  m = MsgIDCount;
  //Fill MsgID Table
  //Save the first Funcid to the MsgID list
  LastFuncID = ~FuncIDList[0];
  for (i = 0; i < FuncIDCount; i++) {
    if (LastFuncID != FuncIDList[i])
      MsgIDTable[MsgIDCount++] = FuncIDList[i];
    LastFuncID = FuncIDList[i];
  }

  // Bind Function ID with the message ID
  // for MsgIDTable[], index is message ID, and its element is function ID
  SendBuff[0] = CMD_M_CONFIG_FUNCID;
  for (i = 0; i < ExecuterCount; i++) {
    SendBuff[1] = 0x00;
    k = 2;
    for (j = 0; j < FuncIDCount; j++) {
      if (ExecuterID[i] == MacIDofFuncID[j]) {
        for (n = 0; n < MsgIDCount; n++) {
          if (MsgIDTable[n] == FuncIDList[j]) {
            // fill Msg ID
            SendBuff[k++] = (uint8_t)(n >> 8);
            SendBuff[k++] = (uint8_t)(n);
            // fill Func ID
            SendBuff[k++] = (uint8_t)(FuncIDList[j] >> 8);
            SendBuff[k++] = (uint8_t)(FuncIDList[j]);
            SERIAL_ECHOLNPAIR("MsgID:", n, "-FuncID:", FuncIDList[j]);
            SendBuff[1]++;
            m++;
            break;
          }
        }
      }
    }

    // k > 2 indicate that module has function ID, need to configure it
    if (k > 2) {
      SERIAL_ECHOLNPAIR("Configuring Module, MAC ID: 0x", Value32BitToString(ExecuterID[i]));
      CanBusControlor.SendLongData(BASIC_CAN_NUM, ExecuterID[i], SendBuff, k);
    }
  }

  SendBuff[0] = CMD_M_CONFIG_FUNCID;
  for (i = 0; i < CanBusControlor.ExtendModuleCount; i++) {
    SendBuff[1] = 0x00;
    k = 2;
    for (j = 0; j < FuncIDCount; j++) {
      // checkout the index in MacIDofFuncID[], use this index to find function id in FuncIDList[]
      // then use the function ID to find message ID in MsgIDTable[]
      if (CanBusControlor.ExtendModuleMacList[i] == MacIDofFuncID[j]) {
        // checkout message ID
        for (n = 0; n < MsgIDCount; n++) {
          if (MsgIDTable[n] == FuncIDList[j]) {
            // fill Msg ID
            SendBuff[k++] = (uint8_t)(n >> 8);
            SendBuff[k++] = (uint8_t)(n);
            // fill Func ID
            SendBuff[k++] = (uint8_t)(FuncIDList[j] >> 8);
            SendBuff[k++] = (uint8_t)(FuncIDList[j]);
            SERIAL_ECHOLNPAIR("MsgID:", n, "-FuncID:", FuncIDList[j]);
            SendBuff[1]++;
            m++;
            break;
          }
        }
      }
    }

    // k > 2 indicate that module has function ID, need to configure it
    if (k > 2) {
      SERIAL_ECHOLNPAIR("Configuring Module, MAC ID: 0x",
        Value32BitToString(CanBusControlor.ExtendModuleMacList[i]));
      CanBusControlor.SendLongData(EXTEND_CAN_NUM, CanBusControlor.ExtendModuleMacList[i], SendBuff, k);
    }
  }

  SERIAL_ECHOLNPAIR("Executer Count: ", ExecuterCount);
  if (ExecuterCount > 0) {
    if ((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_PRINT))
      ExecuterHead.MachineType = MACHINE_TYPE_3DPRINT;
    else if (((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_CNC))
            && (ExecuterMark[0] != 0xff))
      ExecuterHead.MachineType = MACHINE_TYPE_CNC;
    else if(((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_LASER))
            && (ExecuterMark[0] != 0xff))
      ExecuterHead.MachineType = MACHINE_TYPE_LASER;
  }

  // check chamber door
  if (CanBusControlor.ExtendModuleCount > 0)
    SetFunctionValue(EXTEND_CAN_NUM, FUNC_REPORT_ENCLOSURE, NULL, 0);
}


#if 0
/**
 *PrepareExecuterModules:Prepare for Executer module
 */
void CanModule::PrepareExecuterModules(void) {
  millis_t tmptick;
  uint32_t i;
  uint32_t j;
  int m;
  int k;
  int n;
  uint16_t LastFuncID;
  uint16_t FuncIDCount;
  uint8_t ExecuterMark[6];
  uint8_t Buff[3] = {CMD_M_CONFIG , 0x00, 0x00};
  int Pins[] = {E0_DIR_PIN};

  WRITE(E0_DIR_PIN, LOW);

  for(i=0;i<ExecuterCount;i++) {
    for(j=0;j<sizeof(Pins) / sizeof(Pins[0]);j++) {
      WRITE(Pins[j], HIGH);
      CanBusControlor.SendLongData(BASIC_CAN_NUM, ExecuterID[i], Buff, 3);

      tmptick = millis() + 100;
      //Unsed in current hardware design
      ExecuterMark[i] = 0xff;
      while(tmptick > millis())
      {
        if(CanBusControlor.ProcessLongPacks(RecvBuff, 20) == 2)
        {
          if(RecvBuff[0] == CMD_S_CONFIG_REACK) ExecuterMark[i] = RecvBuff[1];
          //SERIAL_ECHOLNPAIR("Executer Mark:", ExecuterMark[i]);
          break;
        }
      }
      WRITE(Pins[j], LOW);
      if(ExecuterMark[i] != 0xff) break;
    }
  }

  //Get Executer module function ID
  FuncIDCount = 0;
  for(i=0;i<ExecuterCount;i++) {
    SendBuff[0] = CMD_M_REQUEST_FUNCID;
    SendBuff[1] = 0;
    CanBusControlor.SendLongData(BASIC_CAN_NUM, ExecuterID[i], SendBuff, 2);
    tmptick = millis() + 2500;
    while(tmptick > millis()) {
      if(CanBusControlor.ProcessLongPacks(RecvBuff, 128) >= 4) {
        if(RecvBuff[0] == CMD_S_REPORT_FUNCID) {
          m = 2;
          for(k=0;k<RecvBuff[1];k++) {
            FuncIDList_CAN2[FuncIDCount] = (RecvBuff[m] << 8) | RecvBuff[m + 1];
            MacIDofFuncID_CAN2[FuncIDCount] = ExecuterID[i];
            SERIAL_ECHOLNPAIR("MacID:", MacIDofFuncID_CAN2[FuncIDCount], " FuncID:", FuncIDList_CAN2[FuncIDCount]);
            m = m + 2;
            FuncIDCount++;
          }
        }
        break;
      }
    }
  }

  tmptick = millis() + 200;
  while(tmptick > millis());

  //Get Priority from table
  for(i=0;i<FuncIDCount;i++) {
    FuncIDPriority_CAN2[i] = 15;
    for(j=0;j<sizeof(PriorityTable) / sizeof(PriorityTable[0]);j++)
    if(PriorityTable[j][0] == FuncIDList_CAN2[i]) {
      FuncIDPriority_CAN2[i] = PriorityTable[j][1];
      break;
    }
  }

  //Sort
  uint32_t tmpswapvalue;
  if(FuncIDCount > 1)
  {
    for(i=0;i<(FuncIDCount - 1);i++) {
      for(j=(i + 1);j<FuncIDCount;j++) {
        if(FuncIDPriority_CAN2[i] > FuncIDPriority_CAN2[j]) {
          tmpswapvalue = FuncIDPriority_CAN2[i];
          FuncIDPriority_CAN2[i] = FuncIDPriority_CAN2[j];
          FuncIDPriority_CAN2[j] = tmpswapvalue;
          tmpswapvalue = FuncIDList_CAN2[i];
          FuncIDList_CAN2[i] = FuncIDList_CAN2[j];
          FuncIDList_CAN2[j] = tmpswapvalue;
          tmpswapvalue = MacIDofFuncID_CAN2[i];
          MacIDofFuncID_CAN2[i] = MacIDofFuncID_CAN2[j];
          MacIDofFuncID_CAN2[j] = tmpswapvalue;
        }
      }
    }
  }

  //Mark the start MsgID
  m = MsgIDCount_CAN2;
  //Fill MsgID Table
  //Save the first Funcid to the MsgID list
  LastFuncID = ~FuncIDList_CAN2[0];
  for(i=0;i<FuncIDCount;i++) {
    if(LastFuncID != FuncIDList_CAN2[i])
      MsgIDTable[MsgIDCount_CAN2++] = FuncIDList_CAN2[i];
    LastFuncID = FuncIDList_CAN2[i];
  }

  //
  //Bind Function ID with the message ID
  SendBuff[0] = CMD_M_CONFIG_FUNCID;
  for(i=0;i<ExecuterCount;i++) {
    SendBuff[1] = 0x00;
    k = 2;
    for(j=0;j<FuncIDCount;j++) {
      if(ExecuterID[i] == MacIDofFuncID_CAN2[j]) {
        for(n=0;n<MsgIDCount_CAN2;n++) {
          if(MsgIDTable[n] == FuncIDList_CAN2[j]) {
            SendBuff[k++] = (uint8_t)(n >> 8);
            SendBuff[k++] = (uint8_t)(n);
            SendBuff[k++] = (uint8_t)(FuncIDList_CAN2[j] >> 8);
            SendBuff[k++] = (uint8_t)(FuncIDList_CAN2[j]);
            SERIAL_ECHOLNPAIR("MsgID:", n, "-FuncID:", FuncIDList_CAN2[j]);
            SendBuff[1]++;
            m++;
            break;
          }
        }
      }
    }
    CanBusControlor.SendLongData(BASIC_CAN_NUM, MacIDofFuncID_CAN2[i], SendBuff, k);
  }

  if((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_PRINT)) ExecuterHead.MachineType = MACHINE_TYPE_3DPRINT;
  else if(((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_CNC)) && (ExecuterMark[0] != 0xff)) ExecuterHead.MachineType = MACHINE_TYPE_CNC;
  else if(((ExecuterID[0] & MODULE_MASK_BITS) == MAKE_ID(MODULE_EXECUTER_LASER)) && (ExecuterMark[0] != 0xff)) ExecuterHead.MachineType = MACHINE_TYPE_LASER;
}


/**
 *PrepareExtendModules:Prepare for extend module
 */
void CanModule::PrepareExtendModules(void) {
  millis_t tmptick;
  uint32_t i;
  uint32_t j;
  int m;
  int k;
  int n;
  uint16_t LastFuncID;
  uint16_t FuncIDCount;

  SERIAL_ECHOLN("Prepare for extend module");

  //Get Extend module function ID
  FuncIDCount = 0;
  for(i=0;i<CanBusControlor.ExtendModuleCount;i++) {
    SendBuff[0] = CMD_M_REQUEST_FUNCID;
    SendBuff[1] = 0;
    CanBusControlor.SendLongData(EXTEND_CAN_NUM, CanBusControlor.ExtendModuleMacList[i], SendBuff, 2);
    tmptick = millis() + 2500;
    while(tmptick > millis()) {
      if(CanBusControlor.ProcessLongPacks(RecvBuff, 128) >= 4) {
        if(RecvBuff[0] == CMD_S_REPORT_FUNCID) {
          m = 2;
          for(k=0;k<RecvBuff[1];k++) {
            FuncIDList_CAN1[FuncIDCount] = (RecvBuff[m] << 8) | RecvBuff[m + 1];
            MacIDofFuncID_CAN1[FuncIDCount] = CanBusControlor.ExtendModuleMacList[i];
            SERIAL_ECHOLNPAIR("MacID:", MacIDofFuncID_CAN1[FuncIDCount], " FuncID:", FuncIDList_CAN1[FuncIDCount]);
            m = m + 2;
            FuncIDCount++;
          }
        }
        break;
      }
    }
  }

  tmptick = millis() + 200;
  while(tmptick > millis());

  //Get Priority from table
  for(i=0;i<FuncIDCount;i++) {
    FuncIDPriority_CAN1[i] = 15;
    for(j=0;j<sizeof(PriorityTable) / sizeof(PriorityTable[0]);j++)
    if(PriorityTable[j][0] == FuncIDList_CAN1[i]) {
      FuncIDPriority_CAN1[i] = PriorityTable[j][1];
      break;
    }
  }

  //Sort
  uint32_t tmpswapvalue;
  if(FuncIDCount > 1)
  {
    for(i=0;i<(FuncIDCount - 1);i++) {
      for(j=(i + 1);j<FuncIDCount;j++) {
        if(FuncIDPriority_CAN1[i] > FuncIDPriority_CAN1[j]) {
          tmpswapvalue = FuncIDPriority_CAN1[i];
          FuncIDPriority_CAN1[i] = FuncIDPriority_CAN1[j];
          FuncIDPriority_CAN1[j] = tmpswapvalue;
          tmpswapvalue = FuncIDList_CAN1[i];
          FuncIDList_CAN1[i] = FuncIDList_CAN1[j];
          FuncIDList_CAN1[j] = tmpswapvalue;
          tmpswapvalue = MacIDofFuncID_CAN1[i];
          MacIDofFuncID_CAN1[i] = MacIDofFuncID_CAN1[j];
          MacIDofFuncID_CAN1[j] = tmpswapvalue;
        }
      }
    }
  }

  //Mark the start MsgID
  m = MsgIDCount_CAN1;
  //Fill MsgID Table
  //Save the first Funcid to the MsgID list
  LastFuncID = ~FuncIDList_CAN1[0];
  for(i=0;i<FuncIDCount;i++) {
    if(LastFuncID != FuncIDList_CAN1[i])
      MsgIDTable[MsgIDCount_CAN1++] = FuncIDList_CAN1[i];
    LastFuncID = FuncIDList_CAN1[i];
  }

  //
  //Bind Function ID with the message ID
  SendBuff[0] = CMD_M_CONFIG_FUNCID;
  for(i=0;i<CanBusControlor.ExtendModuleCount;i++) {
    SendBuff[1] = 0x00;
    k = 2;
    for(j=0;j<FuncIDCount;j++) {
      if(CanBusControlor.ExtendModuleMacList[i] == MacIDofFuncID_CAN1[j]) {
        for(n=0;n<MsgIDCount_CAN1;n++) {
          if(MsgIDTable[n] == FuncIDList_CAN1[j]) {
            SendBuff[k++] = (uint8_t)(n >> 8);
            SendBuff[k++] = (uint8_t)(n);
            SendBuff[k++] = (uint8_t)(FuncIDList_CAN1[j] >> 8);
            SendBuff[k++] = (uint8_t)(FuncIDList_CAN1[j]);
            SERIAL_ECHOLNPAIR("MsgID:", n, "-FuncID:", FuncIDList_CAN1[j]);
            SendBuff[1]++;
            m++;
            break;
          }
        }
      }
    }
    CanBusControlor.SendLongData(EXTEND_CAN_NUM, MacIDofFuncID_CAN1[i], SendBuff, k);
  }
}
#endif

/**
 * GetFirmwareVersion:Get the firmware version of the specific module
 * para CanNum:The Can port of the Module
 * para MacID:The MacID of the Module
 * para pVersion:The pointer to save the version
 * return:True for success, or else false
 */
bool CanModule::GetFirmwareVersion(uint8_t CanNum, uint32 MacID, char* pVersion) {
  int i;
  uint16_t datalen;
  millis_t tmptick;
  SendBuff[0] = CMD_M_VERSIONS_REQUEST;
  SendBuff[1] = 0;
  CanBusControlor.SendLongData(BASIC_CAN_NUM, MacID, SendBuff, 2);
  tmptick = millis() + 1500;
  while(tmptick > millis()) {
    datalen = CanBusControlor.ProcessLongPacks(RecvBuff, 128);
    if(datalen >= 4) {
      if(RecvBuff[0] == CMD_S_VERSIONS_REACK) {
        for(i=0;(i<32) && (i<(datalen-2));i++) {
          pVersion[i] = RecvBuff[2 + i];
          if(pVersion[i] == 0)
            break;
        }
        pVersion[i] = 0;
        return true;
      }
    }
  }
  return false;
}

/**
 * EnumFirmwareVersion:Enum the firmware version of the modules on the CAN bus
 * para ReportToScreen:True for reporting to the screen
 * para ReportToPC:True for reporting to the PC
 */
void CanModule::EnumFirmwareVersion(bool ReportToScreen, bool ReportToPC) {
  char Version[32];
  uint32_t ID;
  if((ReportToPC == false) && (ReportToScreen == false))
    return;

  for(int i=0;i<CanBusControlor.ModuleCount;i++) {
    ID = CanBusControlor.ModuleMacList[i];
    if(GetFirmwareVersion(BASIC_CAN_NUM, ID, Version) == true) {
      if(ReportToPC == true)
        SERIAL_ECHOPAIR("MAC:", Value32BitToString(ID), " Version", Version);
      if(ReportToScreen == true)
        HMI.SendModuleVersion(ID, Version);
    }
  }
}

/**
 * SetAxesLength:Set the length of the specific linear module
 * para Lead:The length value set to the linear module
 * return:True for success, or else false
 */
bool CanModule::SetAxesLength(uint32_t ID, uint16_t Length) {
  int i;
  uint32_t intlen;
  uint16_t datalen;
  millis_t tmptick;
  intlen = Length * 1000.0f;
  SendBuff[0] = CMD_M_SET_LINEAR_LEN;
  SendBuff[1] = 1;
  SendBuff[2] = (uint8_t)(intlen >> 24);
  SendBuff[3] = (uint8_t)(intlen >> 16);
  SendBuff[4] = (uint8_t)(intlen >> 8);
  SendBuff[5] = (uint8_t)(intlen);
  ID |= 1;

  for(i=0;i<CanModules.LinearModuleCount;i++) {
    if(CanModules.LinearModuleID[i] == ID) {
      if(CanBusControlor.SendLongData(BASIC_CAN_NUM, CanModules.LinearModuleID[i], SendBuff, 6) == true) {
        tmptick = millis() + 1500;
        while(tmptick > millis()) {
          datalen = CanBusControlor.ProcessLongPacks(RecvBuff, 128);
          if(datalen >= 6) {
            if(RecvBuff[0] == CMD_S_SET_LINEAR_LEAD_REACK) {
              intlen = (uint32_t)((RecvBuff[2] << 24) | (RecvBuff[3] << 16) | (RecvBuff[4] << 8) | (RecvBuff[5]));
              LinearModuleLength[i] = (float)intlen / 1000.0f;
              return true;
            }
          }
        }
      }
      break;
    }
  }
  return false;
}

/**
 * GetAxesLength:Get the length of the specific linear module
 * return:True for success, or else false
 */
void CanModule::GetAxesLength() {
  int i;
  uint32_t intlen;
  uint16_t datalen;
  millis_t tmptick;
  SendBuff[0] = CMD_M_SET_LINEAR_LEN;
  SendBuff[1] = 0;
  for(i=0;i<CanModules.LinearModuleCount;i++) {
    CanBusControlor.SendLongData(BASIC_CAN_NUM, CanModules.LinearModuleID[i], SendBuff, 2);
    tmptick = millis() + 1500;
    while(tmptick > millis()) {
      datalen = CanBusControlor.ProcessLongPacks(RecvBuff, 128);
      if(datalen >= 6) {
        if(RecvBuff[0] == CMD_S_SET_LINEAR_LEN_REACK) {
          intlen = (uint32_t)((RecvBuff[2] << 24) | (RecvBuff[3] << 16) | (RecvBuff[4] << 8) | (RecvBuff[5]));
          LinearModuleLength[i] = (float)intlen / 1000.0f;
          SERIAL_ECHOLNPAIR("LEN:", LinearModuleLength[i]);
          break;
        }
      }
    }
  }
}

/**
 * SetAxesLead:Set the lead of the specific linear module
 * para Lead:The lead value set to the linear module
 * return:True for success, or else false
 */
bool CanModule::SetAxesLead(uint32_t ID, float Lead) {
  int i;
  uint32_t intLead;
  uint16_t datalen;
  millis_t tmptick;
  intLead = Lead * 1000.0f;
  SendBuff[0] = CMD_M_SET_LINEAR_LEAD;
  SendBuff[1] = 1;
  SendBuff[2] = (uint8_t)(intLead >> 24);
  SendBuff[3] = (uint8_t)(intLead >> 16);
  SendBuff[4] = (uint8_t)(intLead >> 8);
  SendBuff[5] = (uint8_t)(intLead);
  ID |= 1;

  for(i=0;i<CanModules.LinearModuleCount;i++) {
    if(CanModules.LinearModuleID[i] == ID) {
      if(CanBusControlor.SendLongData(BASIC_CAN_NUM, CanModules.LinearModuleID[i], SendBuff, 6) == true) {
        tmptick = millis() + 1500;
        while(tmptick > millis()) {
          datalen = CanBusControlor.ProcessLongPacks(RecvBuff, 128);
          if(datalen >= 6) {
            if(RecvBuff[0] == CMD_S_SET_LINEAR_LEAD_REACK) {
              intLead = (uint32_t)((RecvBuff[2] << 24) | (RecvBuff[3] << 16) | (RecvBuff[4] << 8) | (RecvBuff[5]));
              LinearModuleT[i] = (float)intLead / 1000.0f;
              return true;
            }
          }
        }
      }
      break;
    }
  }
  return false;
}

/**
 * GetAxesLead:Get the lead of the specific linear module
 * return:True for success, or else false
 */
void CanModule::GetAxesLead() {
  int i;
  uint32_t intLen;
  uint16_t datalen;
  millis_t tmptick;
  SendBuff[0] = CMD_M_SET_LINEAR_LEAD;
  SendBuff[1] = 0;
  for(i=0;i<CanModules.LinearModuleCount;i++) {
    CanBusControlor.SendLongData(BASIC_CAN_NUM, CanModules.LinearModuleID[i], SendBuff, 2);
    tmptick = millis() + 1500;
    while(tmptick > millis()) {
      datalen = CanBusControlor.ProcessLongPacks(RecvBuff, 128);
      if(datalen >= 6) {
        if(RecvBuff[0] == CMD_S_SET_LINEAR_LEAD_REACK) {
          intLen = (uint32_t)((RecvBuff[2] << 24) | (RecvBuff[3] << 16) | (RecvBuff[4] << 8) | (RecvBuff[5]));
          LinearModuleT[i] = (float)intLen / 1000.0f;
          break;
        }
      }
    }
  }
}

/**
 * SetMacID:Set the MacID of the specific module
 * OldMacID:The old MacID of the module
 * NewMacID:The new MacID set to the module
 * return:True for success, or else false etc unexisting
 */
bool CanModule::SetMacID(uint32_t OldMacID, uint32_t NewMacID) {
  int i;
  int j;
  uint32_t intLen;
  uint16_t datalen;
  millis_t tmptick;
  uint32_t tmp_new_macid;

  OldMacID = (OldMacID << 1) | 1;
  tmp_new_macid = (NewMacID << 1) | 1;
  NewMacID = (OldMacID & (uint32_t)~0x000fffff) | (tmp_new_macid & 0xfffff);
  NewMacID = (NewMacID >> 1) & 0x7ffff;

  SendBuff[0] = CMD_M_SET_RANDOM;
  SendBuff[1] = 1;
  SendBuff[2] = (uint8_t)(NewMacID >> 24);
  SendBuff[3] = (uint8_t)(NewMacID >> 16);
  SendBuff[4] = (uint8_t)(NewMacID >> 8);
  SendBuff[5] = (uint8_t)(NewMacID);
  for(i=0;i<CanBusControlor.ModuleCount;i++) {
    if(CanBusControlor.ModuleMacList[i] == OldMacID) {
      CanBusControlor.SendLongData(BASIC_CAN_NUM, CanBusControlor.ModuleMacList[i], SendBuff, 6);
      tmptick = millis() + 1500;
      while(tmptick > millis()) {
        datalen = CanBusControlor.ProcessLongPacks(RecvBuff, 128);
        if(datalen >= 6) {
          if(RecvBuff[0] == CMD_S_SET_RANDOM_REACK) {
            intLen = (uint32_t)((RecvBuff[2] << 24) | (RecvBuff[3] << 16) | (RecvBuff[4] << 8) | (RecvBuff[5]));
            for(j=0;j<CanModules.LinearModuleCount;j++) {
              if(CanModules.LinearModuleID[j] == OldMacID) {
                CanModules.LinearModuleID[j] = tmp_new_macid;
                break;
              }
            }
            CanBusControlor.ModuleMacList[i] = intLen;
            return true;
          }
        }
      }
      break;
    }
  }
  return false;
}

/**
 *LoadUpdateData:Load update data from flash
 *para Packindex:
 *para pData:The point to the buff
 */
uint16_t CanModule::SearchModule(uint16_t ModuleTypeID) {
  int i;
  uint16_t Count;
  Count = 0;
  for(i=0;i<CanBusControlor.ModuleCount;i++) {
    if(MAKE_ID(ModuleTypeID) == (CanBusControlor.ModuleMacList[i] & MODULE_MASK_BITS)) {
      Count++;
    }
  }
  for(i=0;i<CanBusControlor.ExtendModuleCount;i++) {
    if(MAKE_ID(ModuleTypeID) == (CanBusControlor.ExtendModuleMacList[i] & MODULE_MASK_BITS)) {
      Count++;
    }
  }
  return Count;
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
  uint32_t * UpdateFlagAddr = (uint32_t *)FLASH_UPDATE_CONTENT_INFO;
 
  if (*UpdateFlagAddr != 0xaa55ee11) {
    return false;
  }
  
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
  uint32_t resend_tick;
  uint16_t PackIndex;
  int i;
  int j;
  int err;

  SERIAL_ECHOLNPAIR("Module Start Update:" , ID);
  tmptick = millis() + 50;
  while(tmptick > millis());

  // Step1:send update version
  i = 0;
  err = 1;
  SendBuff[i++] = CMD_M_UPDATE_REQUEST;
  SendBuff[i++] = (Flag & 1)?1:0;
  for(j=0;(j<64) && (Version[j]!=0);j++) SendBuff[i++] = Version[j];
  CanBusControlor.SendLongData(CanNum, ID, SendBuff, i);
  tmptick = millis() + 6000;
  while(tmptick > millis()) {
    if(CanBusControlor.ProcessLongPacks(RecvBuff, 2) == 2) {
      if(RecvBuff[0] == CMD_S_UPDATE_REQUEST_REACK)
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

  // Step2:querry update status
  tmptick = millis() + 8000;
  resend_tick = millis();
  SendBuff[0] = CMD_M_UPDATE_STATUS_REQUEST;
  err = 1;
  while(1) {
    if(millis() > tmptick) {
      break;
    }
    else if(millis() > resend_tick) {
      resend_tick = millis() + 500;
      CanBusControlor.SendLongData(CanNum, ID, SendBuff, 1);
    }
    if(CanBusControlor.ProcessLongPacks(RecvBuff, 1) >= 1) {
      if(RecvBuff[0] == CMD_S_UPDATE_STATUS_REACK) {
        err = 0;
        break;
      }
    }
  }

  if(err != 0)
    return false;

  // Step3: send start update
  SendBuff[0] = CMD_M_UPDATE_START;
  CanBusControlor.SendLongData(CanNum, ID, SendBuff, 1);

  // Step4:send update content
  tmptick = millis() + 10000;
  while(1) {
    if(CanBusControlor.ProcessLongPacks(RecvBuff, 4) == 4) {
      if(RecvBuff[0] == CMD_S_UPDATE_PACK_REQUEST) {
        PackIndex = (uint16_t)((RecvBuff[2] << 8) | RecvBuff[3]);
        if(LoadUpdatePack(PackIndex, &SendBuff[2]) == true) {
          SendBuff[0] = CMD_M_UPDATE_PACKDATA;
          SendBuff[1] = 0;
          CanBusControlor.SendLongData(CanNum, ID, SendBuff, 128 + 2);
        } else {
          SendBuff[0] = CMD_M_UPDATE_END;
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

  char Version[64];
  //Load Update infomation and check if it is the module update file
  if(LoadUpdateInfo(Version, &StartID, &EndID, &UpdateFlag) == true) {
    SERIAL_ECHOLNPAIR("Version:", Version, "  StartID", StartID, "  EndID:", EndID);
    //Ergodic all modules which are suitable for updating
    for(i=0;i<CanBusControlor.ModuleCount;i++) {
      CurTypeID = CanBusControlor.ModuleMacList[i] & MODULE_MASK_BITS;
      if((CurTypeID >= StartID) && (CurTypeID <= EndID)) {
        UpdateModule(BASIC_CAN_NUM, CanBusControlor.ModuleMacList[i], Version, UpdateFlag);
      } else {

      }
    }

    for(i=0;i<CanBusControlor.ExtendModuleCount;i++) {
      CurTypeID = CanBusControlor.ExtendModuleMacList[i] & MODULE_MASK_BITS;
      if((CurTypeID >= StartID) && (CurTypeID <= EndID)) {
        UpdateModule(EXTEND_CAN_NUM, CanBusControlor.ExtendModuleMacList[i], Version, UpdateFlag);
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
  index = MsgID;
  tmpEndstopBits |= (1 << index);
  if(pBuff[2] == 0) tmpEndstopBits &= ~(1 << index);

  Endstop &= ~0x7f; // clear bit
  Endstop |= 0x7f & tmpEndstopBits & (tmpEndstopBits >> 7) & (tmpEndstopBits >> 14);
  return 0;
}


/**
 * SetFunctionValue:Post Value to the specific Function ID Modules
 * Para CanNum:The Can port number
 * Para FuncID:
 * Para pBuff:The Pointer of the data to be sent
 * Para Len:The length of the data,nomore than 8
 */
int CanModule::SetFunctionValue(uint8_t CanNum, uint16_t FuncID, uint8_t *pBuff, uint8_t Len) {
  int i;
  for(i=0;i<MsgIDCount;i++) {
    if(MsgIDTable[i] == FuncID) {
      CanSendPacked(i, IDTYPE_STDID, CanNum, FRAME_DATA, Len, pBuff);
      break;
    }
  }
  return 0;
}


#endif // ENABLED CANBUS_SUPPORT
