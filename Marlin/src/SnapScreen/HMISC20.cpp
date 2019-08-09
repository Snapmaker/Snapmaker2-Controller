
#include "../inc/MarlinConfig.h"

#if ENABLED(HMI_SC20W)
#include "../module/stepper.h"
#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/printcounter.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../gcode/gcode.h"
#include "../gcode/parser.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../module/configuration_store.h"
#include "../sd/cardreader.h"
#include "../module/ExecuterManager.h"
#include "../module/PowerPanic.h"
#include "../module/StatusControl.h"
#include "../module/LaserExecuter.h"
#include "../module/CNCexecuter.h"
#include "../module/PeriphDevice.h"
#include "../module/Probe.h"
#include "../module/endstops.h"
#include <EEPROM.h>
#include "HMISC20.h"
#include "../../HAL/HAL_GD32F1/HAL_watchdog_STM32F1.h"
#include "../snap_module/lightbar.h"
#include "../feature/runout.h"
#include "../snap_module/snap_dbg.h"

extern long pCounter_X, pCounter_Y, pCounter_Z, pCounter_E;
char tmpBuff[1024];

#define BYTES_TO_32BITS(buff, index) (((uint8_t)buff[index] << 24) | ((uint8_t)buff[index + 1] << 16) | ((uint8_t)buff[index + 2] << 8) | ((uint8_t)buff[index + 3]))
#define BYTES_TO_32BITS_WITH_INDEXMOVE(result, buff, N) do{result = BYTES_TO_32BITS(buff, N); N = N + 4; }while(0)
#define AXIS_MAX_POS(AXIS, BUFF, N) do{ AXIS##_MAX_POS = BYTES_TO_32BITS(BUFF, N) / 1000.0f; N = N + 4; }while(0)
#define AXIS_DIR(AXIS, BUFF, N) do{ AXIS##_DIR = (int32_t)BYTES_TO_32BITS(BUFF, N)==1?true:false; N = N + 4; }while(0)
#define AXIS_HOME_DIR(AXIS, BUFF, N) do{ AXIS##_HOME_DIR = (int32_t)BYTES_TO_32BITS(BUFF, N)>0?1:-1; N = N + 4; }while(0)
#define AXIS_HOME_OFFSET(AXIS, BUFF, N) do{home_offset[AXIS] = BYTES_TO_32BITS(BUFF, N) / 1000.0f; N = N + 4; }while(0)
#define BITS32_TO_BYTES(u32bit, buff, index) do { \
    buff[index++] = (uint8_t)(u32bit >> 24); \
    buff[index++] = (uint8_t)(u32bit >> 16); \
    buff[index++] = (uint8_t)(u32bit >> 8); \
    buff[index++] = (uint8_t)(u32bit); \
    }while(0)

//指令暂存缓冲，正确解析之后，指令存放在这里
static char SendBuff[1024];

//检测指令缓冲是否为空
#define CMD_BUFF_EMPTY() (commands_in_queue>0?false:true)

//U  盘枚举成功
#if ENABLED(SDSUPPORT)
#define IS_UDISK_INSERTED IS_SD_INSERTED()

#else

#define IS_UDISK_INSERTED false
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define CHECK_RUNOUT_SENSOR     runout.sensor_state()
#else
  #define CHECK_RUNOUT_SENSOR     (0)
#endif

/**
 *PackedProtocal:Pack up the data in protocal
 */
void HMI_SC20::PackedProtocal(char * pData, uint16_t len)
{
  uint16_t i;
  uint16_t j;
  uint32_t checksum;

  i = 0;

  //包头
  SendBuff[i++] = 0xAA;
  SendBuff[i++] = 0x55;

  //包长
  SendBuff[i++] = 0x00;
  SendBuff[i++] = 0x00;

  //协议版本
  SendBuff[i++] = 0x00;

  //包长效验
  SendBuff[i++] = 0x00;

  //校验
  SendBuff[i++] = 0x00;
  SendBuff[i++] = 0x00;
  while (len--) SendBuff[i++] = *pData++;

  //重填包长
  SendBuff[2] = (uint8_t) ((i - 8) >> 8);
  SendBuff[3] = (uint8_t) (i - 8);
  SendBuff[5] = SendBuff[2] ^SendBuff[3];

  //校验
  checksum = 0;
  for (j = 8; j < (i - 1); j = j + 2) checksum += (uint32_t) (((uint8_t) SendBuff[j] << 8) | (uint8_t) SendBuff[j + 1]);
  if ((i - 8) % 2) checksum += (uint8_t)SendBuff[i - 1];
  while (checksum > 0xffff) checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
  checksum = ~checksum;
  SendBuff[6] = checksum >> 8;
  SendBuff[7] = checksum;
  HmiWriteData(SendBuff, i);
}

/**
 *HmiWriteData:Write datas to the HMI serial port
 *para pData:the pointer to the datas
 *para len:number of the datas to be written
 */
void HMI_SC20::HmiWriteData(char * pData, uint16_t len)
{
  uint8_t c;

  while (len--) {
    c = *pData++;
    HMISERIAL.write(c);
  }
}

/**
 *SC20屏幕获取指令
 */
short HMI_SC20::GetCommand(unsigned char * pBuff)
{
  int c;
  uint16_t tmplen;
  uint16_t tmphead;
  uint16_t nexthead;
  uint16_t tmptail;
  uint16_t i;
  uint32_t checksum;

  tmphead = ReadHead;
  tmptail = ReadTail;
  while (1) {
    nexthead = (tmphead + 1) % sizeof(ReadBuff);
    if (nexthead == tmptail) break;
    c = HMISERIAL.read();
    if (c == -1) break;
    ReadBuff[tmphead] = (uint8_t)
    c;
    tmphead = nexthead;
  }
  ReadHead = tmphead;

  //没数据
  if (tmphead == tmptail) {
    return (short) - 1;
  }
  ReadHead = tmphead;
  tmplen = (uint16_t) ((tmphead + sizeof(ReadBuff) -tmptail) % sizeof(ReadBuff));

  //数据长度足够
  while (tmplen > 9) {
    if (ReadBuff[tmptail] != 0xAA) {
      tmptail = (tmptail + 1) % sizeof(ReadBuff);
      tmplen--;

      //更新读指针
      ReadTail = tmptail;
      continue;
    }
    if (ReadBuff[(tmptail + 1) % sizeof(ReadBuff)] != 0x55) {
      tmptail = (tmptail + 2) % sizeof(ReadBuff);
      tmplen = tmplen - 2;

      //更新读指针
      ReadTail = tmptail;
      continue;
    }

    //读取包长
    uint8_t cmdLen0 = ReadBuff[(tmptail + 2) % sizeof(ReadBuff)];
    uint8_t cmdLen1 = ReadBuff[(tmptail + 3) % sizeof(ReadBuff)];
    uint16_t commandLen = (uint16_t) ((cmdLen0 << 8) | cmdLen1);

    //包长效验错误
    if ((((commandLen >> 8) & 0xff) ^ (commandLen & 0xff)) != ReadBuff[(tmptail + 5) % sizeof(ReadBuff)]) {
      tmptail = (tmptail + 2) % sizeof(ReadBuff);
      tmplen = tmplen - 2;

      //更新读指针
      ReadTail = tmptail;
      continue;
    }

    //缓冲数据足够
    if (commandLen <= (tmplen - 8)) {
      //复制数据
      for (i = 0; i < (commandLen + 8); i++) {
        pBuff[i] = ReadBuff[tmptail];
        tmptail = (tmptail + 1) % sizeof(ReadBuff);
      }

      //更新读指针
      ReadTail = tmptail;

      //校验
      checksum = 0;
      for (i = 0; i < (commandLen - 1); i = i + 2) checksum += (pBuff[i + 8] << 8) | pBuff[i + 9];

      //奇偶判断
      if (commandLen % 2) checksum += pBuff[commandLen + 8 - 1];
      while (checksum > 0xffff) checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
      checksum = ~checksum;
      if ((uint16_t)checksum != (uint16_t) ((pBuff[6] << 8) | pBuff[7])) {
        CmdChecksumError(true);
        return (short) - 1;
      }

      //_rx_buffer->tail = _rx_buffer->head = 0;
      return commandLen + 8;
    }

    //数据长度不足
    else {
      return (short) - 1;
    }
  }
  return (short) - 1;
}

/********************************************************
请求固件版本信息
*********************************************************/
void HMI_SC20::RequestFirmwareVersion()
{
  uint32_t Address;
  uint16_t i;
  char Version[33];

  //Request controller's firmware version
  Address = FLASH_BOOT_PARA + 2048;
  for(i=0;i<32;i++)
    Version[i] = *((char*)Address++);
  Version[31] = 0;

  i = 0;

  tmpBuff[i++] = 0xAA;
  tmpBuff[i++] = 3;
  for(int j=0;j<32;j++) {
    tmpBuff[i++] = Version[j];
    if(Version[j] == 0) break;
  }
  PackedProtocal(tmpBuff, i);
}

/********************************************************
固件版本对比，并上报需不需要升级,Result为1表示需要升级
*********************************************************/
void HMI_SC20::CheckFirmwareVersion(char *pNewVersion)
{
  uint32_t Address;
  uint16_t i;
  uint8_t Result;
  char Version[33];

  Result = 0;
  Address = FLASH_BOOT_PARA + 2048;
  for(i=0;i<32;i++)
    Version[i] = *((char*)Address++);
  Version[32] = 0;
  for(i=0;i<32;i++) {
    if(Version[i] != pNewVersion[i]) {
      Result = 1;
      break;
    }
    if((Version[i] == pNewVersion[i]) && (Version[i]) == 0) break;
  }

  i = 0;
  tmpBuff[i++] = 0xAA;
  tmpBuff[i++] = 4;
  tmpBuff[i++] = Result;
  PackedProtocal(tmpBuff, i);
}


/********************************************************
启动升级
*********************************************************/
void HMI_SC20::StartUpdate(void)
{
  uint32_t Address;
  uint8_t Pages;

  //擦除FLASH
  FLASH_Unlock();

  //擦除升级文件信息
  Pages = UPDATE_CONTENT_INFO_SIZE / 2048;
  Address = FLASH_UPDATE_CONTENT_INFO;
  for (int i = 0; i < Pages; i++) {
    FLASH_ErasePage(Address);
    Address += 2048;
  }

  //擦除升级内容
  Pages = MARLIN_CODE_SIZE / 2048;
  Address = FLASH_UPDATE_CONTENT;
  for (int i = 0; i < Pages; i++) {
    FLASH_ErasePage(Address);
    Address += 2048;
  }
  FLASH_Lock();
  UpdateDataSize = 0;
  UpdateInProgress = 1;
  UpdatePackRequest = 0;
}

/********************************************************
升级包处理
参数    pBuff:数据缓冲区指针
      DataLen:数据长度
*********************************************************/
void HMI_SC20::UpdatePackProcess(uint8_t * pBuff, uint16_t DataLen)
{
  uint32_t Address;
  uint16_t Packindex;
  uint16_t u16Value;
  uint16_t maxpack;

  Packindex = (uint16_t) ((pBuff[0] << 8) | pBuff[1]);
  maxpack = MARLIN_CODE_SIZE / 512;
  //暂定500包，即250K
  if ((Packindex < maxpack) && (UpdateInProgress == 1) && (Packindex == UpdatePackRequest)) {
    //减去包序号2个字节
    DataLen = DataLen - 2;
    UpdateDataSize = Packindex * 512 + DataLen;
    Address = FLASH_UPDATE_CONTENT + Packindex * 512;
    if ((DataLen % 2) != 0) DataLen++;
    FLASH_Unlock();
    for (int i = 0; i < DataLen; i = i + 2) {
      u16Value = ((pBuff[i + 3] << 8) | pBuff[i + 2]);
      FLASH_ProgramHalfWord(Address, u16Value);
      Address = Address + 2;
    }
    FLASH_Lock();
    UpdatePackRequest++;
    SendUpdatePackRequest(UpdatePackRequest);
  }
}

/********************************************************
升级文件下载结束应答
*********************************************************/
bool HMI_SC20::UpdateDownloadComplete(void)
{
  uint32_t Address;
  if (UpdateDataSize == 0) {
    return false;
  } else {
    //Init the watch dog to reboot system
    Address = FLASH_UPDATE_CONTENT_INFO;
    FLASH_Unlock();
    FLASH_ProgramWord(Address, 0xaa55ee11);
    FLASH_Lock();
    WatchDogInit();
    return true;
  }
}

/********************************************************
升级结束应答
*********************************************************/
void HMI_SC20::SendUpdateComplete(uint8_t Type)
{
  int i;

  i = 0;
  tmpBuff[i++] = 0xAA;
  tmpBuff[i++] = 6;
  tmpBuff[i++] = Type;
  PackedProtocal(tmpBuff, i);
}

/********************************************************
升级结束应答
*********************************************************/
void HMI_SC20::SendUpdateStatus(uint8_t Status)
{
  int i;

  i = 0;
  tmpBuff[i++] = EID_UPGRADE_RESP;
  tmpBuff[i++] = 5;
  tmpBuff[i++] = Status;
  PackedProtocal(tmpBuff, i);
}

/********************************************************
激光画方框
*********************************************************/
void HMI_SC20::LaserCoarseCalibrate(float X, float Y, float Z) {
  X = X / 1000.0f;
  Y = Y / 1000.0f;
  Z = Z / 1000.0f;
  
  // All axes home
  process_cmd_imd("G28");

  // Move to the Certain point
  do_blocking_move_to_logical_xy(X, Y, 30.0f);

  // Move to the Z
  do_blocking_move_to_logical_z(Z, 10.0f);  
}

/********************************************************
激光画方框
*********************************************************/
void HMI_SC20::DrawLaserCalibrateShape() {
  int i;
  int rowindex;
  float NextX, NextY;
  float StartX, StartY, RowSpace;
  float SquareSideLength;

  rowindex = 0;
  StartX = current_position[X_AXIS];
  StartY = current_position[Y_AXIS];
  RowSpace = 3;
  SquareSideLength = 10;

  // Move down 5
  do_blocking_move_to_logical_z(current_position[Z_AXIS] - 5, 20.0f);

  NextX = StartX;
  NextY = StartY;
  i = 1;

  // Fan On
  process_cmd_imd("M106 P0 S255");
  
  // Draw 10 square
  do {
    // Move to the start point
    do_blocking_move_to_logical_xy(NextX, NextY, 50.0f);
    
    // Laser on
    ExecuterHead.Laser.SetLaserPower(100.0f);

    // Draw square
    do_blocking_move_to_logical_xy(current_position[X_AXIS] + SquareSideLength, current_position[Y_AXIS], 5.0f);
    do_blocking_move_to_logical_xy(current_position[X_AXIS], current_position[Y_AXIS] - SquareSideLength, 5.0f);
    do_blocking_move_to_logical_xy(current_position[X_AXIS] - SquareSideLength, current_position[Y_AXIS], 5.0f);
    do_blocking_move_to_logical_xy(current_position[X_AXIS], current_position[Y_AXIS] + SquareSideLength, 5.0f);
    
    // Laser off
    ExecuterHead.Laser.SetLaserPower(0.0f);

    // Move up 1mm
    do_blocking_move_to_logical_z(current_position[Z_AXIS] + 1, 20.0f);

    // Caculate next x y
    if(i % 5 == 0) {
      rowindex++;
      NextX = StartX;
      NextY = StartY + rowindex * (SquareSideLength + RowSpace);
    }
    else {
      NextX = NextX + SquareSideLength + RowSpace;
    }
    i++;
  }while(i < 11);
  // Fan Off
  process_cmd_imd("M107 P0");
}

/**
 * DrawLaserCalibrateShape
 */
bool HMI_SC20::DrawLaserRuler(float StartX, float StartY, float StartZ, float Z_Increase, uint8_t Count) {
  int i;
  float next_x, next_y, next_z;
  float line_space;
  float line_len_short, line_len_long;

  line_space = 2;
  line_len_short = 5;
  line_len_long = 10;
  next_x = StartX;
  next_y = StartY;
  next_z = StartZ - ((float)Count / 2.0 * Z_Increase);

  if(next_z <= 5)
    return false;
  
  // Move to next Z
  do_blocking_move_to_logical_z(next_z, 20.0f);
  
  i = 0;

  // Fan On
  process_cmd_imd("M106 P0 S255");
  
  // Draw 10 square
  do {
    // Move to the start point
    do_blocking_move_to_logical_xy(next_x, next_y, 50.0f);
    
    // Laser on
    ExecuterHead.Laser.SetLaserPower(100.0f);

    // Draw Line
    if((i % 5) == 0)
      do_blocking_move_to_logical_xy(next_x, current_position[Y_AXIS] + line_len_long, 3.0f);
    else
      do_blocking_move_to_logical_xy(next_x, current_position[Y_AXIS] + line_len_short, 3.0f);
    
    // Laser off
    ExecuterHead.Laser.SetLaserPower(0.0f);

    // Move up 1mm
    do_blocking_move_to_logical_z(current_position[Z_AXIS] + Z_Increase, 20.0f);

    next_x = next_x + line_space;
    i++;
  }while(i < Count);
  
  // Fan Off
  process_cmd_imd("M107 P0");
  
  // Move to the center
  do_blocking_move_to_logical_xy(StartX, StartY, 50.0f);
  return true;
}


/********************************************************
激光画方框
*********************************************************/
void HMI_SC20::MovementProcess(float X, float Y, float Z, uint8_t Option) {
  X = X / 1000.0f;
  Y = Y / 1000.0f;
  Z = Z / 1000.0f;
  switch(Option) {
    case 0:
      process_cmd_imd("G28 Z");
      set_bed_leveling_enabled(false);
      break;

    case 1:
      do_blocking_move_to_logical_z(Z, 10.0f);
      do_blocking_move_to_logical_xy(X, Y, 30.0f);
      break;

    case 2:
      do_blocking_move_to_logical_z(current_position[Z_AXIS] + Z, 10.0f);
      do_blocking_move_to_logical_xy(current_position[X_AXIS] + X, current_position[Y_AXIS] + Y, 30.0f);
      break;
  }
}

/********************************************************
平自动调平处理
*********************************************************/
uint8_t HMI_SC20::HalfAutoCalibrate()
{
  int j;
  int Index;
  int indexx, indexy;

  if ((CMD_BUFF_EMPTY() == true) && (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)) {
    // Turn off the heaters
    thermalManager.disable_all_heaters();
    process_cmd_imd("G28");
    set_bed_leveling_enabled(false);

    // Set the Z max feedrate to 50mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = 50;

    // Set the current position of Z to Z_MAX_POS
    current_position[Z_AXIS] = Z_MAX_POS;
    sync_plan_position();
    indexx = 0;
    indexy = 0;
    Index = 0;
    endstops.enable_z_probe(true);
    for (j = 0; j < (GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y); j++) {
      indexx = CalibrateIndeX[Index];
      indexy = CalibrateIndeY[Index];
      Index++;
      // Move Z to 15mm
      do_blocking_move_to_logical_z(15);
      MeshPointZ[indexy * GRID_MAX_POINTS_X + indexx] = probe_pt(RAW_X_POSITION(_GET_MESH_X(indexx)), RAW_Y_POSITION(_GET_MESH_Y(indexy)), PROBE_PT_RAISE, 2);
      SERIAL_ECHOLNPAIR("Zvalue:", MeshPointZ[indexy * GRID_MAX_POINTS_X + indexx]);

      // Send the point index to HMI
      SendHalfCalibratePoint(0x03, indexy * GRID_MAX_POINTS_X + indexx + 1);
    }
    endstops.enable_z_probe(false);

    //Zoffset
    do_blocking_move_to_logical_z(7, 50);
    do_blocking_move_to_logical_xy(_GET_MESH_X(1), _GET_MESH_Y(1), 50.0f);

    // Recover the Z max feedrate to 20mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = 20;

    HMICommandSave = 1;

    CalibrateMethod = 1;
    return 0;
  }
  return 1;
}


/****************************************************
手动调平启动
***************************************************/
uint8_t HMI_SC20::ManualCalibrateStart()
{
  int i, j;

  if ((CMD_BUFF_EMPTY() == true) && (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)) {
    // Disable all heaters
    thermalManager.disable_all_heaters();
    process_cmd_imd("G28");
    set_bed_leveling_enabled(false);

    // Z limit switch at the higtest position
    if (Z_HOME_DIR > 0) current_position[Z_AXIS] = Z_MAX_POS;

    // Z limit switch at the lowest position
    else current_position[Z_AXIS] = 0;
    sync_plan_position();

    // Move Z to 20mm height
    do_blocking_move_to_logical_z(20);

    // Preset the index to 99 for initial status
    PointIndex = 99;

    for (i = 0; i < GRID_MAX_POINTS_Y; i++) {
      for (j = 0; j < GRID_MAX_POINTS_X; j++) {
        MeshPointZ[i * GRID_MAX_POINTS_X + j] = z_values[i][j];
      }
    }

    //标置手动调平
    CalibrateMethod = 2;
    return 0;
  }
  return 1;
}


/****************************************************
机器尺寸重定义
***************************************************/
void HMI_SC20::ResizeMachine(char * pBuff)
{
  uint16_t j;
  //长宽高
  j = 0;
  //X
  AXIS_MAX_POS(X, pBuff, j);

  //Y
  AXIS_MAX_POS(Y, pBuff, j);

  //Z
  AXIS_MAX_POS(Z, pBuff, j);

  //回原点方向
  //X
  AXIS_HOME_DIR(X, pBuff, j);

  //Y
  AXIS_HOME_DIR(Y, pBuff, j);

  //Z
  AXIS_HOME_DIR(Z, pBuff, j);

  //电机方向
  //X
  AXIS_DIR(X, pBuff, j);

  //Y
  AXIS_DIR(Y, pBuff, j);

  //Z
  AXIS_DIR(Z, pBuff, j);

  //E
  E_DIR = true;

  //Offset
  //X
  AXIS_HOME_OFFSET(X_AXIS, pBuff, j);

  //Y
  AXIS_HOME_OFFSET(Y_AXIS, pBuff, j);

  //Z
  AXIS_HOME_OFFSET(Z_AXIS, pBuff, j);

#if ENABLED(SW_MACHINE_SIZE)
  UpdateMachineDefines();
#endif

  CanModules.Init();
	
  //保存数据
  settings.save();
}

 /**
 * ReportModuleFirmwareVersion:Send module firmware version to SC20
 */
void HMI_SC20::ReportModuleFirmwareVersion(uint32_t ID, char *pVersion) {
  uint16_t i;
  
  i = 0;

  tmpBuff[i++] = 0xAA;
  tmpBuff[i++] = 7;
  tmpBuff[i++] = (uint8_t)(ID >> 24);
  tmpBuff[i++] = (uint8_t)(ID >> 16);
  tmpBuff[i++] = (uint8_t)(ID >> 8);
  tmpBuff[i++] = (uint8_t)(ID);
  
  for(int j=0;j<32;j++) {
    tmpBuff[i++] = pVersion[j];
    if(pVersion[j] == 0) break;
  }
  PackedProtocal(tmpBuff, i);

}


void HMI_SC20::PollingCommand(void)
{
  float fX, fY, fZ;
  uint32_t ID;
  int32_t int32Value;
  uint16_t j;
  uint16_t cmdLen;
  short i;
  char result;
  uint8_t CurStatus;
  bool GenReack = false;
  uint8_t eventId, OpCode, Result;
  #define MarkNeedReack(R) do{GenReack = true; Result = R;}while(0)
  i = GetCommand((unsigned char *) tmpBuff);
  if (i == (short) - 1) {
  }

  //屏幕协议
  else {
    cmdLen = (tmpBuff[2] << 8) | tmpBuff[3];
    eventId = tmpBuff[8];
    OpCode = tmpBuff[9];

    //上位指令调试
    if (eventId == EID_GCODE_REQ) {
      //指令尾补0
      j = cmdLen + 8;
      tmpBuff[j] = 0;
      Screen_enqueue_and_echo_commands(&tmpBuff[13], 0xffffffff, 0x02);
    }

    //GCode  打印
    else if (eventId == EID_FILE_GCODE_REQ) {
      SetGcodeState(GCODE_STATE_RECEIVED);
      //获取当前状态
      CurStatus = SystemStatus.GetCurrentPrinterStatus();

      //上位启动打印
      if ((CurStatus == STAT_RUNNING_ONLINE) || (CurStatus == STAT_PAUSE_ONLINE)) {
        //激光头
        if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
          //行号
          ID = BYTES_TO_32BITS(tmpBuff, 9);

          //指令尾补0
          j = cmdLen + 8;
          tmpBuff[j] = 0;
          if (Periph.GetDoorCheckFlag() == true) {
            //门已关闭
            if (Periph.IsDoorOpened() == false) {
              Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
            }
            else {
              SystemStatus.PauseTriggle(ManualPause);
              SystemStatus.SetCurrentPrinterStatus(STAT_PAUSE_ONLINE);
            }
          }
          else {
            Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
          }
        }

        //非激光头
        else {
          //行号
          ID = BYTES_TO_32BITS(tmpBuff, 9);

          //指令尾补0
          j = tmpBuff[3] + 8;
          tmpBuff[j] = 0;
          Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
        }
        SetGcodeState(GCODE_STATE_BUFFERED);
      }
    }

    //状态上报
    else if (eventId == EID_STATUS_REQ) {
      uint8_t StatuID;
      StatuID = tmpBuff[9];

      //获取当前状态
      CurStatus = SystemStatus.GetCurrentPrinterStatus();

      //查询状态
      if (StatuID == 0x01) {
        SendMachineStatus();
      }

      //查询异常
      else if (StatuID == 0x02) {
        SendMachineFaultFlag();
      }

      //联机打印
      else if (StatuID == 0x03) {
        SnapDbg(SNAP_INFO, "receive start work from SC\n");
        //待机状态中
        if (CurStatus == STAT_IDLE) {
          //设置打印状态
          SystemStatus.SetCurrentPrinterStatus(STAT_RUNNING_ONLINE);

          //激光执行头
          if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
            //Z  未知坐标
            if (axes_homed(Z_AXIS) == false) {
              //Z  轴回原点
              process_cmd_imd("G28");
            }

            //走到工件坐标
            do_blocking_move_to_logical_xy(0, 0);
          }
          PowerPanicData.Data.FilePosition = 0;
          PowerPanicData.Data.accumulator = 0;
          PowerPanicData.Data.HeaterTamp[0] = 0;
          PowerPanicData.Data.BedTamp = 0;
          PowerPanicData.Data.PositionData[0] = 0;
          PowerPanicData.Data.PositionData[1] = 0;
          PowerPanicData.Data.PositionData[2] = 0;
          PowerPanicData.Data.PositionData[3] = 0;
          PowerPanicData.Data.GCodeSource = GCODE_SOURCE_SCREEN;
          PowerPanicData.Data.MachineType = ExecuterHead.MachineType;

          //使能断电检测
          //EnablePowerPanicCheck();
          //使能断料检测
          process_cmd_imd("M412 S1");
          SendMachineStatusChange(0x03, 0);

          //屏幕锁定
          HMICommandSave = 1;

          lightbar.set_state(LB_STATE_WORKING);
          SnapDbg(SNAP_INFO, "start working ok!\n");
        }
        else {
          SnapDbg(SNAP_ERROR, "current state is not IDLE\n");
        }
      }

      //暂停
      else if (StatuID == 0x04) {
        //获取当前状态
        CurStatus = SystemStatus.GetCurrentPrinterStatus();
        if (CurStatus == STAT_RUNNING) {
          HmiRequestStatus = STAT_PAUSE;
          if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) ExecuterHead.Laser.SetLaserPower(0.0f);
          SystemStatus.PauseTriggle(ManualPause);
          lightbar.set_state(LB_STATE_STANDBY);
        }
        else if (CurStatus == STAT_RUNNING_ONLINE) {
          HmiRequestStatus = STAT_PAUSE_ONLINE;
          if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) ExecuterHead.Laser.SetLaserPower(0.0f);
          SystemStatus.PauseTriggle(ManualPause);
          lightbar.set_state(LB_STATE_STANDBY);
        }
      }

      //继续
      else if (StatuID == 0x05) {
        //获取当前状态
        CurStatus = SystemStatus.GetCurrentPrinterStatus();
        Result = E_FAILURE;
        //U  盘打印
        if (CurStatus == STAT_PAUSE) {
          //激光执行头
          if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
            //关闭检测
            Periph.StopDoorCheck();

            //门开关检测使能
            if (Periph.GetDoorCheckFlag() == true) {
              //门已关闭
              if (Periph.IsDoorOpened() == false) {
                //启动打印
                Result = SystemStatus.PauseResume();
              }

              //开启检测
              Periph.StartDoorCheck();
            }

            //禁能门开关检测
            else {
              Result = SystemStatus.PauseResume();
            }
          }
          else {
            Result = SystemStatus.PauseResume();
          }
        }

        //连机打印
        else if (CurStatus == STAT_PAUSE_ONLINE) {
          Result = SystemStatus.PauseResume();
        }

        if (Result == E_NO_RESRC) {
          SystemStatus.SetSystemFaultBit(FAULT_FLAG_FILAMENT);
          // send error back to screen
          SendPowerPanicResume(0x05, 1);
        }
        else {
          //清除故障标志
          SystemStatus.ClearSystemFaultBit(FAULT_FLAG_FILAMENT);
          HmiRequestStatus = STAT_RUNNING;
        }
        if(Result == E_SUCCESS)
          SendMachineStatusChange(StatuID, 0);
        else
          SendMachineStatusChange(StatuID, 1);
      }

      //停止
      else if (StatuID == 0x06) {
        //不在待机状态
        if (CurStatus != STAT_IDLE) {
          HmiRequestStatus = STAT_PAUSE_ONLINE;
          //上位机停止分2  种，立即停止
          SystemStatus.StopTriggle(ManualStop);

          //清除断电有效标置
          PowerPanicData.Data.Valid = 0;

          lightbar.set_state(LB_STATE_STANDBY);
        }
      }

      //打印结束
      else if (StatuID == 0x07) {
        //不在待机状态
        if (CurStatus != STAT_IDLE) {
          //触发停止
          SystemStatus.StopTriggle(EndPrint);

          //清除断电有效标置
          PowerPanicData.Data.Valid = 0;
          lightbar.set_state(LB_STATE_FINISH);
        }
      }

      //请求最近行号
      else if (StatuID == 0x08) {
        SendBreakPointData();
      }

      //请求打印进度
      else if (StatuID == 0x09) {
      #if (BAORD_VER == BOARD_SNAPMAKER2_v1)
        //文件打开
        if (card.isFileOpen() == true) //更新最后进度
          card.LastPercent = card.percentDone();

        //发送进度
        SendProgressPercent(card.LastPercent);
      #endif
      }

      //清除断电续打数据
      else if (StatuID == 0x0a) {
        //清除标置
        SystemStatus.ClearSystemFaultBit(0xffffffff);

        //断电数据有效
        if (PowerPanicData.Data.Valid == 1) {
          //清除Flash  有效位
          PowerPanicData.MaskPowerPanicData();
        }

        //标志断电续打无效
        PowerPanicData.Data.Valid = 0;

        //应答
        MarkNeedReack(0);
      }

      //联机续打
      else if (StatuID == 0x0b) {
        //获取当前状态
        CurStatus = SystemStatus.GetCurrentPrinterStatus();

        //激光执行头
        if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
          //外罩门检测开启
          if (Periph.GetDoorCheckFlag() == true) {
            //门已关闭且处于待机状态
            if ((Periph.IsDoorOpened() == false) && (CurStatus == STAT_IDLE)) {
              //启动打印
              PowerPanicData.PowerPanicResumeWork(NULL);

              //开启检测
              Periph.StartDoorCheck();
            }

            //门未关闭
            else {
              //发送失败
              SendPowerPanicResume(0x0b, 1);
            }
          }

          //不检测外罩门，待机状态
          else if (CurStatus == STAT_IDLE) {
            //启动打印
            PowerPanicData.PowerPanicResumeWork(NULL);
          }

          //不检测外罩门，非待机状态
          else {
            //发送失败
            SendPowerPanicResume(0x0b, 1);
          }
        }

        //CNC  或3D  打印
        else {
          // check if we have runout detected for 3D printer
          if ((MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType) && (CHECK_RUNOUT_SENSOR)) {
            SystemStatus.SetSystemFaultBit(FAULT_FLAG_FILAMENT);
            SendPowerPanicResume(0x0b, 1);
          }
          else {
            //处于待机状态
            if (CurStatus == STAT_IDLE) {
              //切换状态
              SystemStatus.SetCurrentPrinterStatus(STAT_PAUSE_ONLINE);

              //启动断电续打
              PowerPanicData.PowerPanicResumeWork(NULL);
            }
            //非待机状态
            else {
              //发送失败
              SendPowerPanicResume(0x0b, 1);
            }
          }
        }
      }

      //U  盘续打
      else if (StatuID == 0x0c) {
        //获取当前状态
        CurStatus = SystemStatus.GetCurrentPrinterStatus();

        //激光执行头
        if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
          //外罩门检测开启
          if (Periph.GetDoorCheckFlag() == true) {
            //门已关闭且处于待机状态
            if ((Periph.IsDoorOpened() == false) && (CurStatus == STAT_IDLE)) {
              //启动打印
              PowerPanicData.PowerPanicResumeWork(NULL);

              //开启检测
              Periph.StartDoorCheck();
            }

            //门未关闭或不处于待机状态
            else {
              //发送失败
              SendPowerPanicResume(0x0c, 1);
            }
          }

          //不检测外罩门，待机状态
          else if (CurStatus == STAT_IDLE) {
            //启动打印
            PowerPanicData.PowerPanicResumeWork(NULL);
          }

          //不检测外罩 门，非待机状态
          else {
            //发送失败
            SendPowerPanicResume(0x0c, 1);
          }
        }

        //CNC  或3D  打印
        else {
          //处于待机状态
          if (CurStatus == STAT_IDLE) {
            //启动断电续打
            PowerPanicData.PowerPanicResumeWork(NULL);
          }

          //非待机状态
          else {
            //发送失败
            SendPowerPanicResume(0x0c, 1);
          }
        }
      }
    }

    //操作指令
    else if (eventId == EID_SETTING_REQ) {
      switch (OpCode)
      {
        //设置尺寸
        case 1:
          ResizeMachine(&tmpBuff[10]);
          MarkNeedReack(0);
          break;

        //开启自动调平
        case 2:
          MarkNeedReack(HalfAutoCalibrate());
          break;

        //开启手动调平
        case 4:
          MarkNeedReack(ManualCalibrateStart());
          break;

        //移动到调平点
        case 5:
          if ((tmpBuff[10] < 10) && (tmpBuff[10] > 0)) {
            //有效索引
            if (PointIndex < 10) {
              //保存数据
              MeshPointZ[PointIndex] = current_position[Z_AXIS];
            }

            //更新点索引
            PointIndex = tmpBuff[10] -1;
            do_blocking_move_to_logical_z(current_position[Z_AXIS] + 5, 30);
            do_blocking_move_to_logical_xy(_GET_MESH_X(PointIndex % GRID_MAX_POINTS_X), _GET_MESH_Y(PointIndex / GRID_MAX_POINTS_Y), 60.0f);
            do_blocking_move_to_logical_z(current_position[Z_AXIS] - 5, 0.2);
            MarkNeedReack(0);
          }
          break;

        //Z  轴移动
        case 6:
          int32Value = BYTES_TO_32BITS(tmpBuff, 10);
          fZ = int32Value / 1000.0f;
          do_blocking_move_to_logical_z(current_position[Z_AXIS] + fZ, 20.0f);
          MarkNeedReack(0);
          break;

        //保存调平点
        case 7:
          if (CMD_BUFF_EMPTY() == true) {
            float delCenter = MeshPointZ[4] - current_position[Z_AXIS];
            //自动调平方式
            if (CalibrateMethod == 1) {
              //设置调平值
              for (i = 0; i < GRID_MAX_POINTS_Y; i++) {
                for (j = 0; j < GRID_MAX_POINTS_X; j++) {
                  sprintf(tmpBuff, "G29 W I%d J%d Z%0.3f", j, i, MeshPointZ[i * GRID_MAX_POINTS_X + j] - delCenter);
                  process_cmd_imd(tmpBuff);
                }
              }
              //保存数据
              settings.save();
            }

            //手动调平
            else if (CalibrateMethod == 2) {
              if (PointIndex != 99) {
                MeshPointZ[PointIndex] = current_position[Z_AXIS];

                //设置调平值
                for (i = 0; i < GRID_MAX_POINTS_Y; i++) {
                  for (j = 0; j < GRID_MAX_POINTS_X; j++) {
                    sprintf(tmpBuff, "G29 W I0 J0 Z%0.3f", MeshPointZ[i * GRID_MAX_POINTS_X + j]);
                    process_cmd_imd(tmpBuff);
                  }
                }
                //保存数据
                settings.save();
              }
            }

            //回原点
            strcpy(tmpBuff, "G28");
            process_cmd_imd(tmpBuff);

            //切换到绝对位置模式
            relative_mode = false;

            //清除标志
            CalibrateMethod = 0;

            //应答
            MarkNeedReack(0);

            //解除屏幕锁定
            HMICommandSave = 0;
          }
          break;

        //退出调平点
        case 8:
          if (CMD_BUFF_EMPTY() == true) {
            //Load
            settings.load();
            process_cmd_imd("G28");
            HMICommandSave = 0;

            //切换到绝对位置模式
            relative_mode = false;

            //应答
            MarkNeedReack(0);

            //解除屏幕锁定
            HMICommandSave = 0;
          }
          break;

        //出厂设置
        case 9:
          break;

        //读取激光Z  轴高度
        case 10:
          //读取
          ExecuterHead.Laser.LoadFocusHeight();
          SendLaserFocus(OpCode, ExecuterHead.Laser.FocusHeight);
          break;

        //设置激光Z  轴高度
        case 11:
          j = 10;
          BYTES_TO_32BITS_WITH_INDEXMOVE(int32Value, tmpBuff, j);
          ExecuterHead.Laser.SetLaserPower(0.0f);
          fZ = (float)int32Value / 1000.0f;
          if(fZ > 65) {
            MarkNeedReack(1);
          }
          else {
            ExecuterHead.Laser.SaveFocusHeight(fZ);
            ExecuterHead.Laser.LoadFocusHeight();
            MarkNeedReack(0);
          }
          break;

        //激光焦点粗调
        case 12:
          if(MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
            j = 10;
            BYTES_TO_32BITS_WITH_INDEXMOVE(fX, tmpBuff, j);
            BYTES_TO_32BITS_WITH_INDEXMOVE(fY, tmpBuff, j);
            BYTES_TO_32BITS_WITH_INDEXMOVE(fZ, tmpBuff, j);
            LaserCoarseCalibrate(fX, fY, fZ);
            //应答
            MarkNeedReack(0);
          }
          else {
            MarkNeedReack(1);
          }
          break;

        // Laser draw square
        case 13:
          if(MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
            DrawLaserCalibrateShape();
            MarkNeedReack(0);
          }
          else {
            MarkNeedReack(1);
          }
          break;

        // Laser draw ruler
        case 14:
          if(MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
            DrawLaserRuler(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 0.1f, 21);
            MarkNeedReack(0);
          }
          else {
            MarkNeedReack(1);
          }
          break;

        //读取尺寸参数
        case 20:
          SendMachineSize();
          break;
      }
    }
    //Movement Request
    else if (eventId == EID_MOVEMENT_REQ) {
      j = 10;
      BYTES_TO_32BITS_WITH_INDEXMOVE(fX, tmpBuff, j);
      BYTES_TO_32BITS_WITH_INDEXMOVE(fY, tmpBuff, j);
      BYTES_TO_32BITS_WITH_INDEXMOVE(fZ, tmpBuff, j);
      switch (OpCode)
      {
        //激光回原点应答
        case 0x01:
          //调平数据失效
          MovementProcess(0, 0, 0, 0);
          break;

        //绝对坐标移动轴
        case 0x02:
          MovementProcess(fX, fY, fZ, 1);
          break;

        //相对坐标移动轴
        case 0x03:
          MovementProcess(fX, fY, fZ, 2);
          break;
      }
      //应答
      MarkNeedReack(0);
    }
    //WIFI
    else if (eventId == EID_LAS_CAM_OP_REQ) {
      if (MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
        switch (OpCode)
        {
          //Set buildin wifi parameter
          case 0x01:
            j = 10;
            for (i = 0; i < 32; i++) {
              SSID[i] = tmpBuff[j++];
              if (SSID[i] == 0) break;
            }
            SSID[31] = 0;
            for (i = 0; i < 32; i++) {
              Password[i] = tmpBuff[j++];
              if (Password[i] == 0) break;
            }
            Password[31] = 0;
            SERIAL_ECHOLNPAIR("SSID:", SSID, " PWD:", Password);
            if(ExecuterHead.Laser.SetWifiParameter(SSID, Password) == 0) MarkNeedReack(0);
            else MarkNeedReack(1);
            break;

          //Read wifi connection status
          case 0x02:
            BuildinWifiIP[0] = 0;
            result = ExecuterHead.Laser.ReadWifiStatus(SSID, Password, BuildinWifiIP);
            SERIAL_ECHOLNPAIR("IP:", BuildinWifiIP);
            if (result == 0) SendWifiIP(OpCode, 0, SSID, Password, BuildinWifiIP);
            else if(result == 1) SendWifiIP(OpCode, 1, SSID, Password, (char*)"");
            else if(result == 2) SendWifiIP(OpCode, 2, (char*)"", (char*)"", (char*)"");
            break;

          case 0x03:
            j = 10;
            BYTES_TO_32BITS_WITH_INDEXMOVE(int32Value, tmpBuff, j);
            fX = int32Value / 1000.0f;
            BYTES_TO_32BITS_WITH_INDEXMOVE(int32Value, tmpBuff, j);
            fY = int32Value / 1000.0f;
            BYTES_TO_32BITS_WITH_INDEXMOVE(int32Value, tmpBuff, j);
            fZ = int32Value / 1000.0f;
            do_blocking_move_to_logical_xy(current_position[X_AXIS] +fX, current_position[Y_AXIS] +fY, 40);
            do_blocking_move_to_logical_z(current_position[Z_AXIS] +fZ, 40);

            //应答
            MarkNeedReack(0);
            break;

          case 0x04:
            j = 10;
            BYTES_TO_32BITS_WITH_INDEXMOVE(int32Value, tmpBuff, j);
            fX = int32Value / 1000.0f;
            BYTES_TO_32BITS_WITH_INDEXMOVE(int32Value, tmpBuff, j);
            fY = int32Value / 1000.0f;
            BYTES_TO_32BITS_WITH_INDEXMOVE(int32Value, tmpBuff, j);
            fZ = int32Value / 1000.0f;
            do_blocking_move_to_logical_xy(fX, fY, 40);
            do_blocking_move_to_logical_z(fZ, 40);

            //应答
            MarkNeedReack(0);
            break;
        }
      }
    }
    //升级
    else if (eventId == EID_UPGRADE_REQ) {
      //SERIAL_ECHOLN(OpCode);
      switch (OpCode)
      {
        //启动升级
        case 0:
          StartUpdate();
          SendGeneralReack((eventId + 1), OpCode, 0);
          SendUpdatePackRequest(UpdatePackRequest);
          SERIAL_ECHOLN("Start Update");
          break;

        //升级包数据
        case 1:
          UpdatePackProcess((uint8_t *) &tmpBuff[10], cmdLen - 2);
          break;

        //升级结束
        case 2:
          SERIAL_ECHOLN("End Update");
          if (UpdateDownloadComplete() == true) MarkNeedReack(0);
          else MarkNeedReack(1);
          break;

        //查询主控版本号
        case 3:
          RequestFirmwareVersion();
          break;

        //固件版本检测
        case 4:
          CheckFirmwareVersion(&tmpBuff[10]);
          break;

        //查询升级状态
        case 5:
          SendUpdateStatus(CanModules.GetUpdateStatus());
          break;

        //查询模块
        case 7:
          CanModules.EnumFirmwareVersion(true, false);
          break;
      }
    }
    else if (eventId == EID_ADDON_OP_REQ) {
      switch (OpCode) {
      case CMD_ADDON_CHK_ONLINE:
        break;

      case CMD_LB_QUERY_STATE:
        lightbar.sync2host();
        break;

      case CMD_LB_SET_MODE_BRIGHTNESS:
        Result = lightbar.set_mode((LightBarMode)tmpBuff[IDX_DATA0]);
        if ((LightBarMode)tmpBuff[IDX_DATA0] == LB_MODE_LIGHTING)
          Result = lightbar.set_brightness(tmpBuff[IDX_DATA0 + 1]);
        MarkNeedReack(!!Result);
        break;

      case CMD_LB_SWITCH:
        if (tmpBuff[IDX_DATA0])
          Result = lightbar.turn_on();
        else
          Result = lightbar.turn_off();
        MarkNeedReack(0);
        break;

      default:
        Result = 1;
        break;
      }
    }
    
    if (GenReack == true) SendGeneralReack((eventId + 1), OpCode, Result);

    //ReadTail = ReadHead;
  }
}


/***********************************************
发送进度
参数    Percent:进度值，取值范围0-100
************************************************/
void HMI_SC20::SendProgressPercent(uint8_t Percent)
{
  uint16_t i;
  i = 0;

  // EventID
  tmpBuff[i++] = EID_STATUS_RESP;

  // Opcode
  tmpBuff[i++] = 0x09;

  // Percent
  tmpBuff[i++] = 0;
  tmpBuff[i++] = 0;
  tmpBuff[i++] = 0;
  tmpBuff[i++] = Percent;
  PackedProtocal(tmpBuff, i);
}


/***********************************************
发送断电续打应答
参数    OpCode:操作码
      Result:结果，0表示成功，非0表示失败
************************************************/
void HMI_SC20::SendPowerPanicResume(uint8_t OpCode, uint8_t Result)
{
  uint16_t i;
  i = 0;

  //EventID
  tmpBuff[i++] = EID_STATUS_RESP;

  //Operation ID
  tmpBuff[i++] = OpCode;

  //结果
  tmpBuff[i++] = Result;
  PackedProtocal(tmpBuff, i);
}


/***********************************************
发送Wifi应答
参数    OpCode:操作码
      Result:结果，0表示成功，非0表示失败
************************************************/
void HMI_SC20::SendWifiIP(uint8_t OpCode, uint8_t Result, char * SSID, char * PWD, char * IP)
{
  uint16_t i;
  i = 0;

  //EventID
  tmpBuff[i++] = EID_LAS_CAM_OP_RESP;

  //Operation ID
  tmpBuff[i++] = OpCode;

  //结果
  tmpBuff[i++] = Result;
  for (int j = 0; j < 31; j++) {
    if (SSID[j] == 0) break;
    tmpBuff[i++] = SSID[j];
  }
  tmpBuff[i++] = 0;
  for (int j = 0; j < 31; j++) {
    if (PWD[j] == 0) break;
    tmpBuff[i++] = PWD[j];
  }
  tmpBuff[i++] = 0;
  for (int j = 0; j < 16; j++) {
    tmpBuff[i++] = IP[j];
    if (IP[j] == 0) break;
  }
  tmpBuff[i++] = 0;
  PackedProtocal(tmpBuff, i);
}


/***********************************************
发送激光焦点
************************************************/
void HMI_SC20::SendLaserFocus(uint8_t OpCode, float Height)
{
  uint32_t u32Value;
  uint16_t i;
  i = 0;

  //EventID
  tmpBuff[i++] = EID_SETTING_RESP;

  //获取尺寸
  tmpBuff[i++] = OpCode;

  //结果
  tmpBuff[i++] = 0;
  u32Value = (uint32_t) (Height * 1000.0f);
  BITS32_TO_BYTES(u32Value, tmpBuff, i);
  PackedProtocal(tmpBuff, i);
}


/***********************************************
发送尺寸信息
************************************************/
void HMI_SC20::SendMachineSize()
{
  int32_t int32Value;
  uint32_t u32Value;
  uint16_t i;
  i = 0;

  //EventID
  tmpBuff[i++] = EID_SETTING_RESP;

  //获取尺寸
  tmpBuff[i++] = 20;
  tmpBuff[i++] = 0;

  //Size
  u32Value = (uint32_t) (X_MAX_POS * 1000);
  BITS32_TO_BYTES(u32Value, tmpBuff, i);
  u32Value = (uint32_t) (Y_MAX_POS * 1000);
  BITS32_TO_BYTES(u32Value, tmpBuff, i);
  u32Value = (uint32_t) (Z_MAX_POS * 1000);
  BITS32_TO_BYTES(u32Value, tmpBuff, i);

  //Offset
  int32Value = (int32_t) (home_offset[X_AXIS] *1000.0f);
  BITS32_TO_BYTES(int32Value, tmpBuff, i);
  int32Value = (int32_t) (home_offset[Y_AXIS] *1000.0f);
  BITS32_TO_BYTES(int32Value, tmpBuff, i);
  int32Value = (int32_t) (home_offset[Z_AXIS] *1000.0f);
  BITS32_TO_BYTES(int32Value, tmpBuff, i);
  PackedProtocal(tmpBuff, i);
}


/***********************************************
升级包请求
参数    PackRequested:请求的包序号
***********************************************/
void HMI_SC20::SendUpdatePackRequest(uint16_t PackRequested)
{
  uint16_t i;
  i = 0;

  tmpBuff[i++] = 0xAA;
  tmpBuff[i++] = 1;
  tmpBuff[i++] = (PackRequested >> 8);
  tmpBuff[i++] = (PackRequested);
  PackedProtocal(tmpBuff, i);
}

/***********************************************
发送通用应答
参数    Resultl:结果，0表示成功，非0表示失败
***********************************************/
void HMI_SC20::SendHalfCalibratePoint(uint8_t Opcode, uint8_t Index)
{
  uint16_t i;
  i = 0;

  tmpBuff[i++] = 0x0a;
  tmpBuff[i++] = Opcode;
  tmpBuff[i++] = 0;
  tmpBuff[i++] = Index;
  PackedProtocal(tmpBuff, i);
}

/***********************************************
发送通用应答
参数    Resultl:结果，0表示成功，非0表示失败
***********************************************/
void HMI_SC20::SendGeneralReack(uint8_t EventID, uint8_t OpCode, uint8_t Result)
{
  uint16_t i;
  i = 0;

  tmpBuff[i++] = EventID;
  tmpBuff[i++] = OpCode;
  tmpBuff[i++] = Result;
  PackedProtocal(tmpBuff, i);
}

/***********************************************
发送断点数据
************************************************/
void HMI_SC20::SendBreakPointData()
{
  uint16_t i;
  i = 0;

  //EventID
  tmpBuff[i++] = EID_STATUS_RESP;
  tmpBuff[i++] = 0x08;
  tmpBuff[i++] = PowerPanicData.Data.Valid;
  tmpBuff[i++] = PowerPanicData.Data.GCodeSource;
  BITS32_TO_BYTES(PowerPanicData.Data.FilePosition, tmpBuff, i);
  PackedProtocal(tmpBuff, i);
}

/***********************************************
发送报警
************************************************/
void HMI_SC20::SendMachineFaultFlag()
{
  uint16_t i;
  i = 0;

  //EventID
  tmpBuff[i++] = 0x08;

  //异常上报
  tmpBuff[i++] = 0x02;

  //异常标志
  uint32_t SysFaultFlag;
  SysFaultFlag = SystemStatus.GetSystemFault();
  BITS32_TO_BYTES(SysFaultFlag, tmpBuff, i);

  //打印文件源
  if (SystemStatus.GetCurrentPrinterStatus() == STAT_IDLE) tmpBuff[i++] = 3;
  else tmpBuff[i++] = PowerPanicData.Data.GCodeSource;

  //行号
  BITS32_TO_BYTES(PowerPanicData.Data.FilePosition, tmpBuff, i);
  PackedProtocal(tmpBuff, i);
}

/***********************************************
发送状态切变
************************************************/
void HMI_SC20::SendMachineStatusChange(uint8_t Status, uint8_t Result)
{
  uint16_t i;
  i = 0;

  //EventID
  tmpBuff[i++] = EID_STATUS_RESP;

  //目前状态
  tmpBuff[i++] = Status;

  //处理结果
  tmpBuff[i++] = Result;
  PackedProtocal(tmpBuff, i);
}

/***********************************************
发送状态信息
************************************************/
void HMI_SC20::SendMachineStatus()
{
  float fValue;
  uint32_t u32Value;
  uint16_t i;
  uint16_t j;
  i = 0;

  //EventID
  tmpBuff[i++] = EID_STATUS_RESP;

  //同步状态
  tmpBuff[i++] = 0x01;

  //坐标
  fValue = LOGICAL_X_POSITION(stepper.position(X_AXIS) * planner.steps_to_mm[X_AXIS]);
  u32Value = (uint32_t) (fValue * 1000);
  BITS32_TO_BYTES(u32Value, tmpBuff, i);
  fValue = LOGICAL_Y_POSITION(stepper.position(Y_AXIS) * planner.steps_to_mm[Y_AXIS]);
  u32Value = (uint32_t) (fValue * 1000);
  BITS32_TO_BYTES(u32Value, tmpBuff, i);
  fValue = LOGICAL_Z_POSITION(stepper.position(Z_AXIS) * planner.steps_to_mm[Z_AXIS]);
  u32Value = (uint32_t) (fValue * 1000);
  BITS32_TO_BYTES(u32Value, tmpBuff, i);
  fValue = stepper.position(E_AXIS) *planner.steps_to_mm[E_AXIS];
  u32Value = (uint32_t) (fValue * 1000);
  BITS32_TO_BYTES(u32Value, tmpBuff, i);

  //温度
  int16_t T0, TB, T0S, TBS;

  //if(current_temperature[0] >= 0)
  T0 = (int16_t)
  thermalManager.temp_hotend[0].current;
  T0S = (int16_t)
  thermalManager.temp_hotend[0].target;

  //if(current_temperature_bed >= 0)
  TB = (int16_t)
  thermalManager.temp_bed.current;
  TBS = (int16_t)
  thermalManager.temp_bed.target;
  tmpBuff[i++] = (uint8_t) ((int) TB >> 8);
  tmpBuff[i++] = (uint8_t) ((int) TB);
  tmpBuff[i++] = (uint8_t) ((int) TBS >> 8);
  tmpBuff[i++] = (uint8_t) ((int) TBS);
  tmpBuff[i++] = (uint8_t) ((int) T0 >> 8);
  tmpBuff[i++] = (uint8_t) ((int) T0);
  tmpBuff[i++] = (uint8_t) ((int) T0S >> 8);
  tmpBuff[i++] = (uint8_t) ((int) T0S);

  //FeedRate
  //tmpBuff[i++] = (uint8_t)(HmiFeedRate >> 8);
  //tmpBuff[i++] = (uint8_t)(HmiFeedRate);
  tmpBuff[i++] = 0;
  tmpBuff[i++] = 0;

  //LaserPower
  uint32_t LaserPower = ExecuterHead.Laser.GetPower();
  tmpBuff[i++] = (uint8_t)(LaserPower >> 24);
  tmpBuff[i++] = (uint8_t)(LaserPower >> 16);
  tmpBuff[i++] = (uint8_t)(LaserPower >> 8);
  tmpBuff[i++] = (uint8_t)(LaserPower);

  //RPM
  uint16_t RPM;
  RPM = ExecuterHead.CNC.GetRPM();
  tmpBuff[i++] = 0;
  tmpBuff[i++] = 0;
  tmpBuff[i++] = (uint8_t) (RPM >> 8);
  tmpBuff[i++] = (uint8_t) (RPM);

  //打印机状态
  j = SystemStatus.GetCurrentPrinterStatus();
  tmpBuff[i++] = (uint8_t) (j);

  //外设状态
  //tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 24);
  //tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 16);
  //tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 8);
  tmpBuff[i++] = (uint8_t) (SystemStatus.GetPeriphDeviceStatus());

  //执行头类型
  tmpBuff[i++] = ExecuterHead.MachineType;

  //CNC  转速
  tmpBuff[i++] = (uint8_t) (RPM >> 8);
  tmpBuff[i++] = (uint8_t) (RPM);
  PackedProtocal(tmpBuff, i);
}


//发送Gcode
void HMI_SC20::SendGcode(char * GCode, uint8_t EventID)
{
  uint8_t i;
  i = 0;

  //EventID
  tmpBuff[i++] = EventID;
  while (*GCode != 0) tmpBuff[i++] = *GCode++;
  PackedProtocal(tmpBuff, i);
}


#endif //ENABLED(HMI_SC20)

