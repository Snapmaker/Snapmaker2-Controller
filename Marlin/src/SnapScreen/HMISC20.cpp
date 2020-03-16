
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
#include "../snap_module/quickstop.h"
#include "../snap_module/M1028.h"
#include "../snap_module/coordinate_mgr.h"
#include "../libs/GenerialFunctions.h"


#define SEND_BUFF_SIZE    1024
#define PACK_BUFFER_SIZE  1024
#define RECV_BUFF_SIZE    2048
#define ONE_CMD_MAX_SIZE  1024
#define STATUS_BUFF_SIZE  128

static char status_buff[STATUS_BUFF_SIZE];

static char *tmpBuff;
static char checkout_cmd[2][ONE_CMD_MAX_SIZE];
// buffer used to packed a command to be sent
static char SendBuff[SEND_BUFF_SIZE];
static char packBuff[PACK_BUFFER_SIZE];
static uint8_t ReadBuff[RECV_BUFF_SIZE];

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

//检测指令缓冲是否为空
#define CMD_BUFF_EMPTY() (commands_in_queue>0?false:true)

//U  盘枚举成功
#if ENABLED(SDSUPPORT)
#define IS_UDISK_INSERTED IS_SD_INSERTED()

#else

#define IS_UDISK_INSERTED false
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define CHECK_RUNOUT_SENSOR     runout.is_filament_runout()
#else
  #define CHECK_RUNOUT_SENSOR     (0)
#endif

extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;

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
    nexthead = (tmphead + 1) % RECV_BUFF_SIZE;
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
  tmplen = (uint16_t) ((tmphead + RECV_BUFF_SIZE -tmptail) % RECV_BUFF_SIZE);

  //数据长度足够
  while (tmplen > 9) {
    if (ReadBuff[tmptail] != 0xAA) {
      tmptail = (tmptail + 1) % RECV_BUFF_SIZE;
      tmplen--;

      //更新读指针
      ReadTail = tmptail;
      continue;
    }
    if (ReadBuff[(tmptail + 1) % RECV_BUFF_SIZE] != 0x55) {
      tmptail = (tmptail + 2) % RECV_BUFF_SIZE;
      tmplen = tmplen - 2;

      //更新读指针
      ReadTail = tmptail;
      continue;
    }

    //读取包长
    uint8_t cmdLen0 = ReadBuff[(tmptail + 2) % RECV_BUFF_SIZE];
    uint8_t cmdLen1 = ReadBuff[(tmptail + 3) % RECV_BUFF_SIZE];
    uint16_t commandLen = (uint16_t) ((cmdLen0 << 8) | cmdLen1);

    //包长效验错误
    if ((((commandLen >> 8) & 0xff) ^ (commandLen & 0xff)) != ReadBuff[(tmptail + 5) % RECV_BUFF_SIZE]) {
      tmptail = (tmptail + 2) % RECV_BUFF_SIZE;
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
        tmptail = (tmptail + 1) % RECV_BUFF_SIZE;
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
        SNAP_DEBUG_CMD_CHECKSUM_ERROR(true);
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

  packBuff[i++] = 0xAA;
  packBuff[i++] = 3;
  // igore the front 10 characters 'Snapmaker_'
  for(int j=10;j<32;j++) {
    packBuff[i++] = Version[j];
    if(Version[j] == 0) break;
  }
  PackedProtocal(packBuff, i);
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
  packBuff[i++] = 0xAA;
  packBuff[i++] = 4;
  packBuff[i++] = Result;
  PackedProtocal(packBuff, i);
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
  packBuff[i++] = 0xAA;
  packBuff[i++] = 6;
  packBuff[i++] = Type;
  PackedProtocal(packBuff, i);
}

/********************************************************
升级结束应答
*********************************************************/
void HMI_SC20::SendUpdateStatus(uint8_t Status)
{
  int i;

  i = 0;
  packBuff[i++] = EID_UPGRADE_RESP;
  packBuff[i++] = 5;
  packBuff[i++] = Status;
  PackedProtocal(packBuff, i);
}

/********************************************************
激光画方框
*********************************************************/
void HMI_SC20::LaserCoarseCalibrate(float X, float Y, float Z) {
  X = X / 1000.0f;
  Y = Y / 1000.0f;
  Z = Z / 1000.0f;

  float max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];
  planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

  // Move to the Certain point
  do_blocking_move_to_logical_xy(X, Y, speed_in_calibration[X_AXIS]);

  // Move to the Z
  do_blocking_move_to_logical_z(Z, speed_in_calibration[Z_AXIS]);

  planner.settings.max_feedrate_mm_s[Z_AXIS] = max_z_speed;
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
  move_to_limited_z(current_position[Z_AXIS] - 5, 20.0f);

  NextX = StartX;
  NextY = StartY;
  i = 1;

  // Fan On
  process_cmd_imd("M106 P0 S255");

  // Draw 10 square
  do {
    // Move to the start point
    move_to_limited_xy(NextX, NextY, 50.0f);

    // Laser on
    ExecuterHead.Laser.SetLaserPower(100.0f);

    // Draw square
    move_to_limited_xy(current_position[X_AXIS] + SquareSideLength, current_position[Y_AXIS], 5.0f);
    move_to_limited_xy(current_position[X_AXIS], current_position[Y_AXIS] - SquareSideLength, 5.0f);
    move_to_limited_xy(current_position[X_AXIS] - SquareSideLength, current_position[Y_AXIS], 5.0f);
    move_to_limited_xy(current_position[X_AXIS], current_position[Y_AXIS] + SquareSideLength, 5.0f);

    // Laser off
    ExecuterHead.Laser.SetLaserPower(0.0f);

    // Move up 1mm
    move_to_limited_z(current_position[Z_AXIS] + 1, 20.0f);

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
  int i = 0;
  float next_x, next_y, next_z;
  float line_space;
  float line_len_short, line_len_long;

  line_space = 2;
  line_len_short = 5;
  line_len_long = 10;
  next_x = StartX - (int)(Count / 2) * 2;
  next_y = StartY;
  next_z = StartZ - ((float)(Count - 1) / 2.0 * Z_Increase);

  if(next_z <= 5)
    return false;

  // Move to next Z
  move_to_limited_z(next_z, 20.0f);

  // Draw 10 Line
  do {
    // Move to the start point
    move_to_limited_xy(next_x, next_y, speed_in_calibration[X_AXIS]);
    planner.synchronize();

    // Laser on
    ExecuterHead.Laser.SetLaserPower(laser_pwr_in_cali);

    // Draw Line
    if((i % 5) == 0)
      move_to_limited_xy(next_x, next_y + line_len_long, speed_in_draw_ruler);
    else
      move_to_limited_xy(next_x, next_y + line_len_short, speed_in_draw_ruler);

    planner.synchronize();

    // Laser off
    ExecuterHead.Laser.SetLaserPower(0.0f);

    // Move up Z increase
    if(i != (Count - 1))
      move_to_limited_z(current_position[Z_AXIS] + Z_Increase, 20.0f);

    next_x = next_x + line_space;
    i++;
  } while(i < Count);

  planner.synchronize();

  // Move to beginning
  move_to_limited_z(StartZ, 20.0f);
  move_to_limited_xy(StartX, StartY, 20.0f);
  planner.synchronize();
  return true;
}


/********************************************************
 * API for movement
*********************************************************/
void HMI_SC20::MovementProcess(float X, float Y, float Z, float F, uint8_t Option) {
  X = X / 1000.0f;
  Y = Y / 1000.0f;
  Z = Z / 1000.0f;

  if (F != 0)
    F = F / 1000.0f;

  switch(Option) {
    case 0:
      process_cmd_imd("G28 Z");
      set_bed_leveling_enabled(false);
      break;

    case 1:
      do_blocking_move_to_logical_z(Z, F? F : 10.0f);
      do_blocking_move_to_logical_xy(X, Y, F? F : 30.0f);
      break;

    case 2:
      // current_position[] is native position, so cannot use API 'do_blocking_move_to_logical_<axis>'
      // it only get logical position
      move_to_limited_z(current_position[Z_AXIS] + Z, F? F : 10.0f);
      move_to_limited_xy(current_position[X_AXIS] + X, current_position[Y_AXIS] + Y, F? F : 30.0f);
      break;
  }

  planner.synchronize();
}


void HMI_SC20::MoveE(float extrude_len, float extrude_speed, float retract_len, float retract_speed, uint8_t Option) {
  extrude_len = extrude_len / 1000.0f;
  extrude_speed = extrude_speed / 60000.0f;
  retract_len = retract_len / 1000.0f;
  retract_speed = retract_speed / 60000.0f;

  switch (Option) {
  // extrude firstly
  case 0:
    current_position[E_AXIS] += extrude_len;
    line_to_current_position(extrude_speed);
    planner.synchronize();
    current_position[E_AXIS] -= retract_len;
    line_to_current_position(retract_speed);
    break;

  // retract firstly
  case 1:
    current_position[E_AXIS] -= retract_len;
    line_to_current_position(retract_speed);
    planner.synchronize();
    current_position[E_AXIS] += extrude_len;
    line_to_current_position(extrude_speed);
    break;

  default:
    break;
  }

  planner.synchronize();
}

/********************************************************
平自动调平处理
*********************************************************/
uint8_t HMI_SC20::HalfAutoCalibrate(bool fast_leveling)
{
  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  LOG_I("e temp: %.2f / %.2f\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
  LOG_I("b temp: %.2f / %.2f\n", thermalManager.degBed(), thermalManager.degTargetBed());

  if ((CMD_BUFF_EMPTY() == true) && (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)) {
    // Turn off the heaters
    thermalManager.disable_all_heaters();

    if (!go_home_before_cali && all_axes_homed() &&
      (!position_shift[X_AXIS] && !position_shift[Y_AXIS] && !position_shift[Z_AXIS])) {
      if (current_position[Z_AXIS] < z_limit_in_cali)
        move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
      move_to_limited_x(0, XY_PROBE_FEEDRATE_MM_S);
      planner.synchronize();
    }
    else
      process_cmd_imd("G28");
    process_cmd_imd("G1029 P3"); // set the default probe points, hardcoded

    set_bed_leveling_enabled(false);

    // change the Z max feedrate
    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    endstops.enable_z_probe(true);

    // move quicky firstly to decrease the time
    do_blocking_move_to_z(z_position_before_calibration, speed_in_calibration[Z_AXIS]);
    planner.synchronize();

    uint8_t ret = auto_probing(true, fast_leveling);

    endstops.enable_z_probe(false);

    // Recover the Z max feedrate to 20mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = orig_max_z_speed;

    HMICommandSave = 1;

    CalibrateMethod = 1;
    return ret;
  }
  return 1;
}


/****************************************************
手动调平启动
***************************************************/
uint8_t HMI_SC20::ManualCalibrateStart()
{
  int i, j;
  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  // when user do manual leveling, clear this var to disable fast-calibration
  nozzle_height_probed = 0;

  if ((CMD_BUFF_EMPTY() == true) && (MACHINE_TYPE_3DPRINT == ExecuterHead.MachineType)) {

    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    // Disable all heaters
    thermalManager.disable_all_heaters();
    if (!go_home_before_cali && all_axes_homed() &&
      (!position_shift[X_AXIS] && !position_shift[Y_AXIS] && !position_shift[Z_AXIS])) {
      if (current_position[Z_AXIS] < z_limit_in_cali)
        move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
      move_to_limited_x(0, XY_PROBE_FEEDRATE_MM_S);
      planner.synchronize();
    }
    else
      process_cmd_imd("G28");
    process_cmd_imd("G1029 P3"); // set the default probe points, hardcoded

    set_bed_leveling_enabled(false);

    bilinear_grid_manual();

    // Move Z to 20mm height
    do_blocking_move_to_z(z_position_before_calibration, speed_in_calibration[Z_AXIS]);

    do_blocking_move_to_z(15, 10);

    // Preset the index to 99 for initial status
    PointIndex = 99;

    for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
      for (i = 0; i < GRID_MAX_POINTS_X; i++) {
        MeshPointZ[j * GRID_MAX_POINTS_X + i] = z_values[i][j];
      }
    }

    planner.settings.max_feedrate_mm_s[Z_AXIS] = orig_max_z_speed;
    //标置手动调平
    CalibrateMethod = 2;
    return 0;
  }
  return 1;
}


#if 0
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
#endif

 /**
 * ReportModuleFirmwareVersion:Send module firmware version to SC20
 */
void HMI_SC20::ReportModuleFirmwareVersion(uint32_t ID, char *pVersion) {
  uint16_t i;

  i = 0;

  packBuff[i++] = 0xAA;
  packBuff[i++] = 7;
  packBuff[i++] = (uint8_t)(ID >> 24);
  packBuff[i++] = (uint8_t)(ID >> 16);
  packBuff[i++] = (uint8_t)(ID >> 8);
  packBuff[i++] = (uint8_t)(ID);

  for(int j=0;j<32;j++) {
    packBuff[i++] = pVersion[j];
    if(pVersion[j] == 0) break;
  }
  PackedProtocal(packBuff, i);

}

/**
 * ReportLinearLength:Send linear module length to SC20
 */
void HMI_SC20::ReportLinearLength() {
  uint16_t i;
  uint16_t j;
  uint32_t ID;
  uint32_t Length;

  i = 0;

  packBuff[i++] = 0x9A;
  packBuff[i++] = 4;
  for(j=0;j<CanModules.LinearModuleCount;j++) {
    ID = CanModules.LinearModuleID[j];
    packBuff[i++] = (uint8_t)(ID >> 24);
    packBuff[i++] = (uint8_t)(ID >> 16);
    packBuff[i++] = (uint8_t)(ID >> 8);
    packBuff[i++] = (uint8_t)(ID);
    Length = CanModules.GetLinearModuleLength(j) * 1000.0f;
    packBuff[i++] = (uint8_t)(Length >> 24);
    packBuff[i++] = (uint8_t)(Length >> 16);
    packBuff[i++] = (uint8_t)(Length >> 8);
    packBuff[i++] = (uint8_t)(Length);
  }

  PackedProtocal(packBuff, i);
}

/**
 * ReportLinearLead:Send linear module lead to SC20
 */
void HMI_SC20::ReportLinearLead() {
  uint16_t i;
  uint16_t j;
  uint32_t ID;
  uint32_t Lead;

  i = 0;

  packBuff[i++] = 0x9A;
  packBuff[i++] = 6;
  for(j=0;j<CanModules.LinearModuleCount;j++) {
    ID = CanModules.LinearModuleID[j];
    packBuff[i++] = (uint8_t)(ID >> 24);
    packBuff[i++] = (uint8_t)(ID >> 16);
    packBuff[i++] = (uint8_t)(ID >> 8);
    packBuff[i++] = (uint8_t)(ID);
    Lead = CanModules.GetLinearModuleLead(j) * 1000.0f;
    packBuff[i++] = (uint8_t)(Lead >> 24);
    packBuff[i++] = (uint8_t)(Lead >> 16);
    packBuff[i++] = (uint8_t)(Lead >> 8);
    packBuff[i++] = (uint8_t)(Lead);
  }

  PackedProtocal(packBuff, i);
}

/**
 * ReportLinearLead:Send linear module lead to SC20
 */
void HMI_SC20::ReportLinearModuleMacID(void) {
  uint16_t i;
  uint16_t j;
  uint32_t ID;

  i = 0;

  packBuff[i++] = 0x9A;
  packBuff[i++] = 2;
  for(j=0;j<CanModules.LinearModuleCount;j++) {
    ID = CanModules.LinearModuleID[j];
    packBuff[i++] = (uint8_t)(ID >> 24);
    packBuff[i++] = (uint8_t)(ID >> 16);
    packBuff[i++] = (uint8_t)(ID >> 8);
    packBuff[i++] = (uint8_t)(ID);
  }

  PackedProtocal(packBuff, i);
}


extern uint8_t hmi_commands_in_queue;
void HMI_SC20::PollingCommand(bool nested) {
  int len;

  len = GetCommand((unsigned char *) checkout_cmd[next_cmd_idx]);
  // invalid command whose PDU len is less than 8
  if (len < 8)
    return;

  // if this command is heartbeat checking, ack it directly
  if (checkout_cmd[next_cmd_idx][IDX_EVENT_ID] == EID_STATUS_REQ &&
        checkout_cmd[next_cmd_idx][IDX_OP_CODE] == 1) {
    SendMachineStatus();
    LOG_V("Heartbeat: ");
    if (debug.GetLevel() <= SNAP_DEBUG_LEVEL_TRACE) {
      for (int i=0; i<10; i++) {
        LOG_T("%08X ", *((uint32_t *)status_buff + i));
      }
      LOG_T("\n\n");
    }

    // make sure we are working
    if (SystemStatus.GetCurrentStatus() == SYSTAT_WORK) {
      // and no movement planned
      if(!planner.movesplanned()) {
        // and we have replied screen
        if (current_line && current_line == debug.GetSCGcodeLine()) {
          // then we known maybe screen lost out last reply
          SendGeneralReack(EID_STATUS_RESP, 0xC, 1);
          LOG_I("waiting HMI command, current line: %u\n", current_line);
        }
      }
    }
    return;
  }

  // if it's not heartbeat checking, but another command is being handled
  // we drop this command
  if (is_handling_cmd) {
    LOG_W("handling another cmd, reject cmd [%u:%u]\n", checkout_cmd[next_cmd_idx][IDX_EVENT_ID],
            checkout_cmd[next_cmd_idx][IDX_OP_CODE]);
    return;
  }

  is_handling_cmd = true;

  // need to change next_cmd_idx before calling HandleOneCommand()
  // becase PollingCommand maybe call nested by HandleOneCommand()
  tmpBuff = checkout_cmd[next_cmd_idx];
  if (next_cmd_idx)
    next_cmd_idx = 0;
  else
    next_cmd_idx = 1;

  HandleOneCommand(nested);

  is_handling_cmd = false;
}


void HMI_SC20::HandleOneCommand(bool reject_sync_write)
{
  float fX, fY, fZ;
  uint32_t ID;
  int32_t int32Value;
  uint16_t j;
  uint16_t cmdLen;
  short i;
  char result;
  SysStatus cur_status;
  SysStage cur_stage;
  WorkingPort cur_port;
  ErrCode err;
  bool GenReack = false;
  uint8_t eventId, OpCode, Result;
  #define MarkNeedReack(R) do{GenReack = true; Result = R;}while(0)

  {
    cmdLen = (tmpBuff[2] << 8) | tmpBuff[3];
    eventId = tmpBuff[8];
    OpCode = tmpBuff[9];

    // get current status
    cur_status = SystemStatus.GetCurrentStatus();
    cur_stage = SystemStatus.GetCurrentStage();
    cur_port = SystemStatus.GetWorkingPort();

    // debug command
    if (eventId == EID_GCODE_REQ) {
      // pad '0' in the end of command
      j = cmdLen + 8;
      tmpBuff[j] = 0;
      Screen_enqueue_and_echo_commands(&tmpBuff[13], 0xffffffff, 0x02);
    }

    //gcode from screen
    else if (eventId == EID_FILE_GCODE_REQ) {
      // need to check if we are in working with screen
      if (SystemStatus.GetWorkingPort() == WORKING_PORT_SC &&
          (cur_status == SYSTAT_WORK || cur_status == SYSTAT_RESUME_WAITING)) {
        // line number
        ID = BYTES_TO_32BITS(tmpBuff, 9);

        if (ID > current_line || current_line == 0) {
          // pad '0' to the end of string command
          j = cmdLen + 8;
          tmpBuff[j] = 0;

          current_line = ID;

          // when we are resuming, won't handle any Gcode
          if (cur_stage == SYSTAGE_RESUMING) {
            if (SystemStatus.ResumeOver() == E_SUCCESS) {
              Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
            }
            else {
              SendEvent(EID_FILE_GCODE_RESP, &tmpBuff[9], 4);
            }
          }
          else
            Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
        }
        else if (ID == current_line) {
          // if we have replied this line, just sent ok
          if (ID == debug.GetSCGcodeLine()) {
            SendEvent(EID_FILE_GCODE_RESP, &tmpBuff[9], 4);
            return;
          }
          else
            return;
        }
        else {
          LOG_E("recv line[%u] less than cur line[%u]\n", ID, current_line);
          return;
        }
      }
    }

    // query or set system status
    else if (eventId == EID_STATUS_REQ) {
      uint8_t StatuID;
      StatuID = tmpBuff[9];

      // query exception
      if (StatuID == 0x02) {
        SendMachineFaultFlag();
      }

      // screen want to start a work
      else if (StatuID == 0x03) {
        LOG_I("SC req START WORK\n");
        err = SystemStatus.StartWork(TRIGGER_SOURCE_SC);
        if (E_SUCCESS == err) {
          // lock screen
          HMICommandSave = 1;

          current_line = 0;

          MarkNeedReack(0);
          LOG_I("trigger WORK: ok\n");
        }
        else {
          LOG_W("failed to start work: err= %d\n", err);
          MarkNeedReack(err);
        }
      }

      // screen request a pause
      else if (StatuID == 0x04) {
        LOG_I("SC trigger PAUSE\n");
        err = SystemStatus.PauseTrigger(TRIGGER_SOURCE_SC);
        if (err == E_SUCCESS) {
          RequestStatus = HMI_REQ_PAUSE;
          LOG_I("trigger PAUSE: ok\n");
        }
        else {
          // other status cannot be paused
          MarkNeedReack((uint8_t)err);
          LOG_W("trigger PAUSE: failed, err = %d\n", err);
        }
      }

      // resume work
      else if (StatuID == 0x05) {
        LOG_I("SC trigger RESUME\n");
        // trigger a resuming, need to ack screen when resume work
        err = SystemStatus.ResumeTrigger(TRIGGER_SOURCE_SC);
        if (err == E_SUCCESS) {
          RequestStatus = HMI_REQ_RESUME;
          if (powerpanic.Data.FilePosition > 0)
            current_line =  powerpanic.Data.FilePosition - 1;
          else
            current_line = 0;
          SNAP_DEBUG_SET_GCODE_LINE(0);
          powerpanic.SaveCmdLine(powerpanic.Data.FilePosition);
          LOG_I("trigger RESUME: ok\n");
        }
        else {
          MarkNeedReack((uint8_t)err);
          LOG_W("trigger RESUME: failed, err = %d\n", err);
        }
      }

      // stop work
      else if (StatuID == 0x06) {
        LOG_I("SC trigger STOP\n");
        err = SystemStatus.StopTrigger(TRIGGER_SOURCE_SC);
        if (err == E_SUCCESS) {
          RequestStatus = HMI_REQ_STOP;
          LOG_I("trigger STOP: ok\n");
        }
        else {
          MarkNeedReack((uint8_t)err);
          LOG_W("trigger STOP: failed, err = %d\n", err);
        }
      }

      // finish work
      else if (StatuID == 0x07) {
        LOG_I("SC trigger FINISH\n");
        err = SystemStatus.StopTrigger(TRIGGER_SOURCE_FINISH);
        // make sure we are working
        if (err != SYSTAT_WORK) {
          RequestStatus = HMI_REQ_FINISH;
          lightbar.set_state(LB_STATE_FINISH);
          LOG_I("trigger FINISH: ok\n");
        }
        else {
          MarkNeedReack((uint8_t)err);
          LOG_W("trigger FINISH: failed, err = %d\n", err);
        }
      }

      // request the latest line number
      else if (StatuID == 0x08) {
        if (cur_stage == SYSTAGE_PAUSE)
          LOG_I("SC req line number: %d\n", powerpanic.Data.FilePosition);
        else
          LOG_I("SC req line number: %d\n", powerpanic.pre_data_.FilePosition);
        SendBreakPointData();
      }

      // request progress
      else if (StatuID == 0x09) {
        // not supported in snapmaker2
        LOG_I("not support cmd: [08, 09]\n");
        MarkNeedReack(1);
      }

      // clear power panic data
      else if (StatuID == 0x0a) {
        if (cmdLen < 6) {
          LOG_I("SC req clear power loss bits\n");
          SystemStatus.ClearExceptionByFaultFlag(FAULT_FLAG_POWER_LOSS);
          if (powerpanic.pre_data_.Valid == 1) {
            // clear flash data
            LOG_I("mask flash data ...");
            powerpanic.MaskPowerPanicData();
            powerpanic.pre_data_.Valid = 0;
            LOG_I("Done!\n");
          }
        }
        else {
          uint32_t fault_bit = BYTES_TO_32BITS(tmpBuff, 10);
          LOG_I("SC req clear exception, fault bits: 0x%08X\n", fault_bit);
          fault_bit &= FAULT_FLAG_SC_CLEAR_MASK;
          SystemStatus.ClearExceptionByFaultFlag(fault_bit);
        }

        // ack
        MarkNeedReack(0);
      }

      // recovery work from power loss
      else if (StatuID == 0x0b) {
        LOG_I("SC trigger restore from power-loss\n");
        // recovery work and ack to screen
        if (cur_status != SYSTAT_IDLE) {
          MarkNeedReack(E_NO_SWITCHING_STA);
        }
        else {
          if (cur_status != SYSTAT_IDLE) {
            MarkNeedReack(E_NO_SWITCHING_STA);
            LOG_E("cannot trigger recovery at current status: %d\n", cur_status);
          }
          else {
            // bug: why will we receive two consecutive recovery command @TODO
            SystemStatus.SetCurrentStatus(SYSTAT_RESUME_TRIG);
            err = powerpanic.ResumeWork();
            if (E_SUCCESS == err) {
              SystemStatus.SetCurrentStatus(SYSTAT_RESUME_WAITING);
              SystemStatus.SetWorkingPort(WORKING_PORT_SC);
              powerpanic.Data.FilePosition = powerpanic.pre_data_.FilePosition;
              if (powerpanic.Data.FilePosition > 0)
                current_line =  powerpanic.Data.FilePosition - 1;
              else
                current_line = 0;
              SNAP_DEBUG_SET_GCODE_LINE(0);
              powerpanic.SaveCmdLine(powerpanic.Data.FilePosition);
              MarkNeedReack(0);
              LOG_I("trigger RESTORE: ok\n");
            }
            else {
              LOG_I("trigger RESTORE: failed, err = %d\n", err);
              MarkNeedReack(err);
              SystemStatus.SetCurrentStatus(SYSTAT_IDLE);
            }
          }
        }
      }

      // not supported now
      else if (StatuID == 0x0c) {
        MarkNeedReack(1);
      }
      // chamber status
      else if (StatuID == 0x0d) {
      }
      // homing status
      else if (StatuID == 0x0e) {
        LOG_V("SC req homing!\n");
        CoordinateMgrReportStatus(eventId, OpCode);
      }
      // query coordinates data
      else if (StatuID == 0xf) {
        LOG_V("SC req change log level");
        SNAP_DEBUG_SET_LEVEL(1, (SnapDebugLevel)tmpBuff[IDX_DATA0]);
      }
      // not supported command
      else {
        MarkNeedReack(1);
      }
    }

    // change settings
    else if (eventId == EID_SETTING_REQ) {
      switch (OpCode)
      {
        // set size of machine
        case 1:
          //ResizeMachine(&tmpBuff[10]);
          //MarkNeedReack(0);
          break;

        // enable auto level bed
        case 2:
          MarkNeedReack(HalfAutoCalibrate(false));
          break;

        // enble manual level bed
        case 4:
          LOG_I("SC req manual level\n");
          MarkNeedReack(ManualCalibrateStart());
          break;

        // move to leveling point
        case 5:
          LOG_I("SC req move to pont: %d\n", tmpBuff[10]);
          if ((tmpBuff[10] < 10) && (tmpBuff[10] > 0)) {
            // check point index
            if (PointIndex < 10) {
              // save point index
              MeshPointZ[PointIndex] = current_position[Z_AXIS];
              LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", PointIndex, current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

              // if got new point, raise Z firstly
              if ((PointIndex != tmpBuff[10] -1) && current_position[Z_AXIS] < z_position_before_calibration)
                do_blocking_move_to_z(current_position[Z_AXIS] + 3, speed_in_calibration[Z_AXIS]);
            }

            // move to new point
            PointIndex = tmpBuff[10] -1;
            do_blocking_move_to_logical_xy(_GET_MESH_X(PointIndex % GRID_MAX_POINTS_X),
                            _GET_MESH_Y(PointIndex / GRID_MAX_POINTS_Y), speed_in_calibration[X_AXIS]);
            MarkNeedReack(0);
          }
          break;

        // move z axis
        case 6:
          int32Value = (int32_t)BYTES_TO_32BITS(tmpBuff, 10);
          fZ = int32Value / 1000.0f;

          // sometimes the bed plane will be under the low limit point
          // to make z can move down always by user, we don't use limited API
          do_blocking_move_to_z(current_position[Z_AXIS] + fZ, speed_in_calibration[Z_AXIS]);
          MarkNeedReack(0);
          break;

        // save the cordinate of leveling points
        case 7:
          LOG_I("SC req save data of leveling\n");
          planner.synchronize();
          if (CalibrateMethod == 2 && PointIndex < 10) {
            // save the last point
            MeshPointZ[PointIndex] = current_position[Z_AXIS];
            LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", PointIndex, current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
            for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
              for (i = 0; i < GRID_MAX_POINTS_X; i++) {
                z_values[i][j] = MeshPointZ[j * GRID_MAX_POINTS_X + i];
              }
            }

            bed_level_virt_interpolate();
            settings.save();
          }
          else if (CalibrateMethod == 1) {
            process_cmd_imd("G1029 S0");
          }
          else {
            MarkNeedReack(1);
            break;
          }

          LOG_I("new leveling data:\n");
          for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
            for (i = 0; i < GRID_MAX_POINTS_X; i++) {
              LOG_I("%.2f ", z_values[i][j]);
            }
            LOG_I("\n");
          }

          set_bed_leveling_enabled(true);

          // move to stop
          move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
          planner.synchronize();

          // make sure we are in absolute mode
          relative_mode = false;

          // clear flag
          CalibrateMethod = 0;

          // ack to screen
          MarkNeedReack(0);

          // unlock PC port
          HMICommandSave = 0;
          break;

        // exit leveling
        case 8:
          LOG_I("SC req exit level\n");
          if (CMD_BUFF_EMPTY() == true) {

            //Load
            settings.load();

            set_bed_leveling_enabled(true);

            // move to stop
            move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
            planner.synchronize();

            HMICommandSave = 0;

            CalibrateMethod = 0;

            // make sure we are in absolute mode
            relative_mode = false;

            MarkNeedReack(0);

            // unlock PC port
            HMICommandSave = 0;
          }
          break;

        // restore to factory mode
        case 9:
          break;

        //读取激光Z  轴高度
        case 10:
          LOG_I("SC req Focus Height\n");
          //读取
          ExecuterHead.Laser.LoadFocusHeight();
          SendLaserFocus(OpCode, ExecuterHead.Laser.FocusHeight);
          break;

        //设置激光Z  轴高度
        case 11:
          LOG_I("Laser: SC set Z axis\n");
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
          LOG_I("Laser: rough focusing\n");

          if (!all_axes_homed()) {
            LOG_E("Machine is not be homed!\n");
            MarkNeedReack(2);
            break;
          }

          if(MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
            j = 10;
            BYTES_TO_32BITS_WITH_INDEXMOVE(fX, tmpBuff, j);
            BYTES_TO_32BITS_WITH_INDEXMOVE(fY, tmpBuff, j);
            BYTES_TO_32BITS_WITH_INDEXMOVE(fZ, tmpBuff, j);
            LOG_I("Laser will move to (%.2f, %.2f, %.2f)\n", fX, fY, fZ);
            LaserCoarseCalibrate(fX, fY, fZ);
            //应答
            MarkNeedReack(0);
          }
          else {
            MarkNeedReack(1);
          }
          break;

        // Laser draw ruler
        case 13:
          LOG_I("Laser: SC req draw ruler\n");
          if(MACHINE_TYPE_LASER == ExecuterHead.MachineType) {
            if (cmdLen < 6) {
              LOG_W("Laser: use default Z offset: 0.5 mm\n", cmdLen);
              DrawLaserRuler(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 0.5f, 21);
            }
            else {
              j = 10;
              fZ = BYTES_TO_32BITS(tmpBuff, j);
              fZ /= 1000;
              LOG_I("Laser: Z offset from SC is %.3f\n", fZ);
              DrawLaserRuler(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], fZ, 21);
            }
            MarkNeedReack(0);
          }
          else {
            MarkNeedReack(1);
          }
          break;

        case 14:
          LOG_I("SC req fast calibration, nozzle height: %.2f\n", nozzle_height_probed);
          // auto leveling, only offset between probe and extruder is known
          if (nozzle_height_probed <= 0 || nozzle_height_probed > MAX_NOZZLE_HEIGHT_PROBED) {
            MarkNeedReack(2);
            LOG_E("Invalid Z offset: %.3f!\n", nozzle_height_probed);
            break;
          }

          if (HalfAutoCalibrate(true)) {
            MarkNeedReack(1);
            LOG_E("Auto calibration failed!\n");
            break;
          }

          process_cmd_imd("G1029 S1");

          LOG_I("new leveling data:\n");
          for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
            for (i = 0; i < GRID_MAX_POINTS_X; i++) {
              LOG_I("%.2f ", z_values[i][j]);
            }
            LOG_I("\n");
          }

          set_bed_leveling_enabled(true);

          // move to stop
          move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
          planner.synchronize();


          CalibrateMethod = 0;
          HMICommandSave = 0;
          MarkNeedReack(0);
          LOG_I("SC req fast calibration: Done!\n");
          break;

        case 0xF:
            LOG_I("SC req change env\n");
            j = IDX_DATA0 + 1;
            BYTES_TO_32BITS_WITH_INDEXMOVE(int32Value, tmpBuff, j);
            fX = int32Value / 1000.0f;
            if (planner.sync_cnt > 0 && tmpBuff[IDX_DATA0] == RENV_TYPE_ZOFFSET) {
//            if (false && reject_sync_write && tmpBuff[IDX_DATA0] == RENV_TYPE_ZOFFSET) {
              LOG_I("REJECT_SYNC_WRITE, RENV_TYPE_ZOFFSET %d\n", planner.sync_cnt);
              MarkNeedReack(E_REJECT_SYNC_WRITE);
            } else {
              err = SystemStatus.ChangeRuntimeEnv(tmpBuff[IDX_DATA0], fX);
              if (err == E_SUCCESS) {
                MarkNeedReack(0);
              }
              else if (err == E_INVALID_STATE) {
                MarkNeedReack(2);
                LOG_E("not supported parameter by current Tool head\n");
              }
              else {
                MarkNeedReack(1);
                LOG_E("invalid parameter\n");
              }
            }
          break;

        // clear auto-leveling data
        case 0x10:
          LOG_I("SC req clear leveling data\n");
          set_bed_leveling_enabled(false);
          for (i = 0; i < GRID_MAX_POINTS_X; i++)
            for (j = 0; j < GRID_MAX_POINTS_Y; j++)
              z_values[i][j] = DEFAUT_LEVELING_HEIGHT;

          bed_level_virt_interpolate();

          nozzle_height_probed = 0;

          set_bed_leveling_enabled(true);

          MarkNeedReack(0);
          break;

        //读取尺寸参数
        case 20:
          SendMachineSize();
          break;
      }
    }
    //Movement Request
    else if (eventId == EID_MOVEMENT_REQ) {
      float F = 0;
      switch (OpCode) {
      //激光回原点应答
      case 0x01:
        //调平数据失效
        MovementProcess(0, 0, 0, 0, 0);
        break;

      //绝对坐标移动轴
      case 0x02:
      case 0x03:
        j = IDX_DATA0;
        BYTES_TO_32BITS_WITH_INDEXMOVE(fX, tmpBuff, j);
        BYTES_TO_32BITS_WITH_INDEXMOVE(fY, tmpBuff, j);
        BYTES_TO_32BITS_WITH_INDEXMOVE(fZ, tmpBuff, j);
        if (cmdLen >= 18) {
          BYTES_TO_32BITS_WITH_INDEXMOVE(F, tmpBuff, j);
        }
        MovementProcess(fX, fY, fZ, F, OpCode - 1);
        MarkNeedReack(0);
        break;

      case 0x04:
        if (thermalManager.tooColdToExtrude(0)) {
          LOG_E("temperature is cool, cannot move E!\n");
          MarkNeedReack(1);
          break;
        }
        j = IDX_DATA0 + 1;
        // avoid to alloc new temp var, use the common var:
        // extrude length
        BYTES_TO_32BITS_WITH_INDEXMOVE(fX, tmpBuff, j);
        // extrude speed
        BYTES_TO_32BITS_WITH_INDEXMOVE(fY, tmpBuff, j);
        // retract length
        BYTES_TO_32BITS_WITH_INDEXMOVE(fZ, tmpBuff, j);
        // retract speed
        BYTES_TO_32BITS_WITH_INDEXMOVE(F, tmpBuff, j);
        MoveE(fX, fY, fZ, F, tmpBuff[IDX_DATA0]);
        MarkNeedReack(0);
        break;

      default:
        LOG_E("not supported command!\n");
        MarkNeedReack(1);
        break;
      }
    }
    //WIFI & Bluetooth
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
            move_to_limited_xy(current_position[X_AXIS] +fX, current_position[Y_AXIS] +fY, 40);
            move_to_limited_z(current_position[Z_AXIS] +fZ, 40);

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

          // Set bluetooth name
          case 0x05:
            j = 10;
            for (i = 0; i < 31; i++) {
              bluetooth_name[i] = tmpBuff[j++];
              if (bluetooth_name[i] == 0) break;
            }
            bluetooth_name[31] = 0;

            SERIAL_ECHOLNPAIR("BlueTooth Name:", bluetooth_name);
            if(ExecuterHead.Laser.SetBluetoothName(bluetooth_name) == 0) MarkNeedReack(0);
            else MarkNeedReack(1);
            break;

          // Read Bluetooth name
          case 0x06:
            bluetooth_name[0] = 0;
            result = ExecuterHead.Laser.ReadBluetoothName(bluetooth_name);
            SERIAL_ECHOLNPAIR("Bluetooth Name:", bluetooth_name);
            if (result == 0) SendBluetoothName(OpCode, 0, bluetooth_name);
            else if(result == 1) SendBluetoothName(OpCode, 1, (char*)"");
            else if(result == 2) SendBluetoothName(OpCode, 2, (char*)"");
            break;

          // Read Bluetooth MAC
          case 0x07:
            result = ExecuterHead.Laser.ReadBluetoothMac(bluetooth_mac);
            if (result == 0) SendBluetoothMac(OpCode, 0, bluetooth_mac);
            else if(result == 2) SendBluetoothMac(OpCode, 2, bluetooth_mac);
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
          LOG_I("SC req FW ver\n");
          RequestFirmwareVersion();
          break;

        //固件版本检测
        case 4:
          LOG_I("SC check FW ver\n");
          CheckFirmwareVersion(&tmpBuff[10]);
          break;

        //查询升级状态
        case 5:
          LOG_I("SC req up state\n");
          SendUpdateStatus(CanModules.GetUpdateStatus());
          break;

        //查询模块
        case 7:
          LOG_I("SC req MODULE ver\n");
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

      case CMD_ENCLOSURE_FAN_SWITCH:
        if(tmpBuff[IDX_DATA0] == 0)
          Periph.SetEnclosureFanSpeed(0);
        else
          Periph.SetEnclosureFanSpeed(255);
          MarkNeedReack(0);
        break;

      default:
        Result = 1;
        break;
      }
    }
    else if (eventId == 0x99) {
      if (OpCode == 0) {
        // trigger powerloss, now is disabled
      }
      // Set MacID
      else if(OpCode == 1) {
        j = 10;
        uint32_t old_MacID;
        uint32_t new_MacID;
        BYTES_TO_32BITS_WITH_INDEXMOVE(old_MacID, tmpBuff, j);
        BYTES_TO_32BITS_WITH_INDEXMOVE(new_MacID, tmpBuff, j);
        if(CanModules.SetMacID(old_MacID, new_MacID) == true)
          MarkNeedReack(0);
        else
          MarkNeedReack(1);
      }
      // List out the MacID
      else if(OpCode == 2) {
        ReportLinearModuleMacID();
      }
      // Set linear module length
      else if(OpCode == 3) {
        j = 10;
        uint32_t new_length;
        BYTES_TO_32BITS_WITH_INDEXMOVE(ID, tmpBuff, j);
        BYTES_TO_32BITS_WITH_INDEXMOVE(new_length, tmpBuff, j);
        ID = ((uint32_t)ID << 1);
        new_length = new_length / 1000.0f;
        SERIAL_ECHOLNPAIR("ID", ID, "New Len:", new_length);
        if(CanModules.SetAxesLength(ID, new_length) == true)
          MarkNeedReack(0);
        else
          MarkNeedReack(1);
      }

      // Get linear module length
      else if(OpCode == 4) {
        CanModules.GetAxesLength();
        ReportLinearLength();
      }

      // Set linear module lead
      else if(OpCode == 5) {
        j = 10;
        uint32_t new_lead;
        BYTES_TO_32BITS_WITH_INDEXMOVE(ID, tmpBuff, j);
        BYTES_TO_32BITS_WITH_INDEXMOVE(new_lead, tmpBuff, j);
        ID = ((uint32_t)ID << 1);
        new_lead = new_lead / 1000.0f;
        SERIAL_ECHOLNPAIR("ID", ID, "New Lead:", new_lead);
        if(CanModules.SetAxesLead(ID, new_lead) == true)
          MarkNeedReack(0);
        else
          MarkNeedReack(1);
      }

      // Get linear module lead
      else if(OpCode == 6) {
        CanModules.GetAxesLead();
        ReportLinearLead();
      }

    }

    if (GenReack == true) SendGeneralReack((eventId + 1), OpCode, Result);

    //ReadTail = ReadHead;
  }
}


#if 0
/***********************************************
发送进度
参数    Percent:进度值，取值范围0-100
************************************************/
void HMI_SC20::SendProgressPercent(uint8_t Percent)
{
  uint16_t i;
  i = 0;

  // EventID
  packBuff[i++] = EID_STATUS_RESP;

  // Opcode
  packBuff[i++] = 0x09;

  // Percent
  packBuff[i++] = 0;
  packBuff[i++] = 0;
  packBuff[i++] = 0;
  packBuff[i++] = Percent;
  PackedProtocal(packBuff, i);
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
  packBuff[i++] = EID_STATUS_RESP;

  //Operation ID
  packBuff[i++] = OpCode;

  //结果
  packBuff[i++] = Result;
  PackedProtocal(packBuff, i);
}
#endif

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
  packBuff[i++] = EID_LAS_CAM_OP_RESP;

  //Operation ID
  packBuff[i++] = OpCode;

  //结果
  packBuff[i++] = Result;
  for (int j = 0; j < 31; j++) {
    if (SSID[j] == 0) break;
    packBuff[i++] = SSID[j];
  }
  packBuff[i++] = 0;
  for (int j = 0; j < 31; j++) {
    if (PWD[j] == 0) break;
    packBuff[i++] = PWD[j];
  }
  packBuff[i++] = 0;
  for (int j = 0; j < 16; j++) {
    packBuff[i++] = IP[j];
    if (IP[j] == 0) break;
  }
  packBuff[i++] = 0;
  PackedProtocal(packBuff, i);
}

/***********************************************
发送蓝牙名字查询应答
参数    OpCode:操作码
      Result:结果，0表示成功，非0表示失败
************************************************/
void HMI_SC20::SendBluetoothName(uint8_t OpCode, uint8_t Result, char * Name)
{
  uint16_t i;
  i = 0;

  //EventID
  packBuff[i++] = EID_LAS_CAM_OP_RESP;

  //Operation ID
  packBuff[i++] = OpCode;

  //结果
  packBuff[i++] = Result;
  for (int j = 0; j < 31; j++) {
    if (Name[j] == 0) break;
    packBuff[i++] = Name[j];
  }
  packBuff[i++] = 0;

  PackedProtocal(packBuff, i);
}

/***********************************************
发送蓝牙名字查询应答
参数    OpCode:操作码
      Result:结果，0表示成功，非0表示失败
************************************************/
void HMI_SC20::SendBluetoothMac(uint8_t OpCode, uint8_t Result, uint8_t * Mac)
{
  uint16_t i;
  i = 0;

  //EventID
  packBuff[i++] = EID_LAS_CAM_OP_RESP;

  //Operation ID
  packBuff[i++] = OpCode;

  //结果
  packBuff[i++] = Result;
  for (int j = 0; j < 6; j++) {
    packBuff[i++] = Mac[j];
  }

  PackedProtocal(packBuff, i);
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
  packBuff[i++] = EID_SETTING_RESP;

  //获取尺寸
  packBuff[i++] = OpCode;

  //结果
  packBuff[i++] = 0;
  u32Value = (uint32_t) (Height * 1000.0f);
  BITS32_TO_BYTES(u32Value, packBuff, i);
  PackedProtocal(packBuff, i);
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
  packBuff[i++] = EID_SETTING_RESP;

  //获取尺寸
  packBuff[i++] = 20;
  packBuff[i++] = 0;

  //Machine size type
  packBuff[i++] = CanModules.GetMachineSizeType();

  //Size
  u32Value = (uint32_t) (X_MAX_POS * 1000);
  BITS32_TO_BYTES(u32Value, packBuff, i);
  u32Value = (uint32_t) (Y_MAX_POS * 1000);
  BITS32_TO_BYTES(u32Value, packBuff, i);
  u32Value = (uint32_t) (Z_MAX_POS * 1000);
  BITS32_TO_BYTES(u32Value, packBuff, i);

  //Home Dir
  int32Value = (int32_t) (X_HOME_DIR);
  BITS32_TO_BYTES(int32Value, packBuff, i);
  int32Value = (int32_t) (Y_HOME_DIR);
  BITS32_TO_BYTES(int32Value, packBuff, i);
  int32Value = (int32_t) (Z_HOME_DIR);
  BITS32_TO_BYTES(int32Value, packBuff, i);

  //Dir
  int32Value = X_DIR == true?(int32_t)1:(int32_t)-1;
  BITS32_TO_BYTES(int32Value, packBuff, i);
  int32Value = Y_DIR == true?(int32_t)1:(int32_t)-1;
  BITS32_TO_BYTES(int32Value, packBuff, i);
  int32Value = Z_DIR == true?(int32_t)1:(int32_t)-1;
  BITS32_TO_BYTES(int32Value, packBuff, i);

  //Offset
  int32Value = (int32_t) (home_offset[X_AXIS] *1000.0f);
  BITS32_TO_BYTES(int32Value, packBuff, i);
  int32Value = (int32_t) (home_offset[Y_AXIS] *1000.0f);
  BITS32_TO_BYTES(int32Value, packBuff, i);
  int32Value = (int32_t) (home_offset[Z_AXIS] *1000.0f);
  BITS32_TO_BYTES(int32Value, packBuff, i);
  PackedProtocal(packBuff, i);
}


/***********************************************
升级包请求
参数    PackRequested:请求的包序号
***********************************************/
void HMI_SC20::SendUpdatePackRequest(uint16_t PackRequested)
{
  uint16_t i;
  i = 0;

  packBuff[i++] = 0xAA;
  packBuff[i++] = 1;
  packBuff[i++] = (PackRequested >> 8);
  packBuff[i++] = (PackRequested);
  PackedProtocal(packBuff, i);
}

/***********************************************
发送通用应答
参数    Resultl:结果，0表示成功，非0表示失败
***********************************************/
void HMI_SC20::SendHalfCalibratePoint(uint8_t Opcode, uint8_t Index)
{
  uint16_t i;
  i = 0;

  packBuff[i++] = 0x0a;
  packBuff[i++] = Opcode;
  packBuff[i++] = 0;
  packBuff[i++] = Index;
  PackedProtocal(packBuff, i);
}

/***********************************************
发送通用应答
参数    Resultl:结果，0表示成功，非0表示失败
***********************************************/
void HMI_SC20::SendGeneralReack(uint8_t EventID, uint8_t OpCode, uint8_t Result)
{
  uint16_t i;
  i = 0;

  packBuff[i++] = EventID;
  packBuff[i++] = OpCode;
  packBuff[i++] = Result;
  PackedProtocal(packBuff, i);
}

/***********************************************
发送断点数据
************************************************/
void HMI_SC20::SendBreakPointData()
{
  uint16_t i;
  i = 0;

  //EventID
  packBuff[i++] = EID_STATUS_RESP;
  packBuff[i++] = 0x08;
  packBuff[i++] = powerpanic.pre_data_.Valid;
  packBuff[i++] = powerpanic.pre_data_.GCodeSource;

  if (SystemStatus.GetCurrentStage() == SYSTAGE_PAUSE) {
    // resume from pause, use this line
    BITS32_TO_BYTES(powerpanic.Data.FilePosition, packBuff, i);
  }
  else {
    // resume form power-loss, use this line
    BITS32_TO_BYTES(powerpanic.pre_data_.FilePosition, packBuff, i);
  }

  PackedProtocal(packBuff, i);
}

/***********************************************
发送报警
************************************************/
void HMI_SC20::SendMachineFaultFlag(uint32_t flag)
{
  uint16_t i;
  i = 0;

  //EventID
  packBuff[i++] = EID_STATUS_RESP;

  //异常上报
  packBuff[i++] = 0x02;

  //异常标志
  if (!flag) {
    flag = SystemStatus.GetSystemFault();
  }
  BITS32_TO_BYTES(flag, packBuff, i);

  //打印文件源
  if (SystemStatus.GetCurrentStage() == SYSTAGE_IDLE) packBuff[i++] = 3;
  else packBuff[i++] = powerpanic.Data.GCodeSource;

  //行号
  BITS32_TO_BYTES(powerpanic.Data.FilePosition, packBuff, i);
  PackedProtocal(packBuff, i);
}

/***********************************************
发送状态切变
************************************************/
void HMI_SC20::SendMachineStatusChange(uint8_t Status, uint8_t Result)
{
  uint16_t i;
  i = 0;

  //EventID
  packBuff[i++] = EID_STATUS_RESP;

  //目前状态
  packBuff[i++] = Status;

  //处理结果
  packBuff[i++] = Result;
  PackedProtocal(packBuff, i);
}

/***********************************************
发送状态信息
************************************************/
void HMI_SC20::SendMachineStatus()
{
  float fValue;
  int32_t tmp;
  uint16_t i = 0;

  // SOF
  status_buff[i++] = 0xAA;
  status_buff[i++] = 0x55;

  // data length, 1B
  i++;
  i++;

  // protocol version
  status_buff[i++] = 0x00;

  // checksum of length, 1B
  i++;

  // checksum of packet, 2B
  i++;
  i++;

  // EventID
  status_buff[i++] = EID_STATUS_RESP;

  // operation code
  status_buff[i++] = 0x01;

  // current logical position
  tmp = (int32_t) (NATIVE_TO_LOGICAL(current_position[X_AXIS], X_AXIS) * 1000);
  BITS32_TO_BYTES(tmp, status_buff, i);
  tmp = (int32_t) (NATIVE_TO_LOGICAL(current_position[Y_AXIS], Y_AXIS) * 1000);
  BITS32_TO_BYTES(tmp, status_buff, i);
  tmp = (int32_t) (NATIVE_TO_LOGICAL(current_position[Z_AXIS], Z_AXIS) * 1000);
  BITS32_TO_BYTES(tmp, status_buff, i);
  tmp = (int32_t) (current_position[E_AXIS] * 1000);
  BITS32_TO_BYTES(tmp, status_buff, i);

  // temperature
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
  status_buff[i++] = (uint8_t) (TB >> 8);
  status_buff[i++] = (uint8_t) (TB);
  status_buff[i++] = (uint8_t) (TBS >> 8);
  status_buff[i++] = (uint8_t) (TBS);
  status_buff[i++] = (uint8_t) (T0 >> 8);
  status_buff[i++] = (uint8_t) (T0);
  status_buff[i++] = (uint8_t) (T0S >> 8);
  status_buff[i++] = (uint8_t) (T0S);

  //FeedRate
  fValue = (last_feedrate * 60);
  status_buff[i++] = (uint8_t)(((uint16_t)fValue)>>8);
  status_buff[i++] = (uint8_t)((uint16_t)fValue);

  // LaserPower
  uint32_t LaserPower = ExecuterHead.Laser.GetPower();
  status_buff[i++] = (uint8_t)(LaserPower >> 24);
  status_buff[i++] = (uint8_t)(LaserPower >> 16);
  status_buff[i++] = (uint8_t)(LaserPower >> 8);
  status_buff[i++] = (uint8_t)(LaserPower);

  // RPM of CNC
  uint16_t RPM;
  RPM = ExecuterHead.CNC.GetRPM();
  status_buff[i++] = 0;
  status_buff[i++] = 0;
  status_buff[i++] = (uint8_t) (RPM >> 8);
  status_buff[i++] = (uint8_t) (RPM);

  // current status
  status_buff[i++] = (uint8_t) SystemStatus.MapCurrentStatusForSC();

  // add-on status
  status_buff[i++] = (uint8_t) (SystemStatus.GetPeriphDeviceStatus());

  // executor type
  status_buff[i++] = ExecuterHead.MachineType;

  // corrent the length
  status_buff[2] = (uint8_t) ((i - 8) >> 8);
  status_buff[3] = (uint8_t) (i - 8);
  status_buff[5] = status_buff[2] ^status_buff[3];

  uint32_t checksum = 0;
  for (int j = 8; j < (i - 1); j = j + 2) checksum += (uint32_t) (((uint8_t) status_buff[j] << 8) | (uint8_t) status_buff[j + 1]);
  if ((i - 8) % 2) checksum += (uint8_t)status_buff[i - 1];
  while (checksum > 0xffff) checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
  checksum = ~checksum;
  status_buff[6] = checksum >> 8;
  status_buff[7] = checksum;

  HmiWriteData(status_buff, i);
}


void HMI_SC20::SendEvent(uint8_t EventID, char *buffer, uint16_t len) {
  uint8_t i = 0;

  packBuff[i++] = EventID;

  while (len-- > 0) packBuff[i++] = *buffer++;

  PackedProtocal(packBuff, i);
}

#endif //ENABLED(HMI_SC20)

