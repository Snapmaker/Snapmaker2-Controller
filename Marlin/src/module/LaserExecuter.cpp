#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "LaserExecuter.h"
#include "CanBus.h"
#include "CanDefines.h"
#include "ExecuterManager.h"
#include "StatusControl.h"

#include "../snap_module/snap_dbg.h"

// time to delay close fan, 5s
#define TIME_TO_CLOSE_FAN (120 * 1000)

static const uint16_t LaserPowerTable[]=
{
  0,
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};

#define LASERSerial MSerial3

#define TimSetPwm(n)  Tim1SetCCR4(n)

/**
 * Init
 */
void LaserExecuter::Init()
{
  Tim1PwmInit();
  LASERSerial.begin(115200);
  LoadFocusHeight();

  power_limit_ = LASER_POWER_NORMAL_LIMIT;

  fan_state_ = LAESR_FAN_STA_CLOSED;
  fan_tick_ = 0;
}

/**
 * check if FAN is open, if not, will open it
 */
void LaserExecuter::CheckFan(uint16_t p) {
  uint8_t data[2];

  switch (fan_state_) {
  case LAESR_FAN_STA_OPEN:
    if (p == 0) {
      fan_state_ = LAESR_FAN_STA_TO_BE_CLOSED;
      fan_tick_ = millis();
    }
    break;

  case LAESR_FAN_STA_TO_BE_CLOSED:
    if (p > 0) {
      fan_state_ = LAESR_FAN_STA_OPEN;
      fan_tick_ = 0;
    }
    break;

  case LAESR_FAN_STA_CLOSED:
    if (p > 0) {
      data[0] = 0;
      data[1] = 255;
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_FAN, data, 2);
      fan_state_ = LAESR_FAN_STA_OPEN;
      fan_tick_ = 0;
    }
    break;
  }
}

/**
 * try to close fan if needed
 */
void LaserExecuter::TryCloseFan() {
  uint8_t data[2];

  if (ExecuterHead.MachineType != MACHINE_TYPE_LASER)
    return;

  if (fan_state_ == LAESR_FAN_STA_TO_BE_CLOSED) {
    if ((millis() - fan_tick_) > TIME_TO_CLOSE_FAN) {
      data[0] = 0;
      data[1] = 0;
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_FAN, data, 2);
      fan_state_ = LAESR_FAN_STA_CLOSED;
      fan_tick_ = 0;
    }
  }
}

/**
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserPower(float Percent)
{
  int integer;
  float decimal;
  uint16_t pwmvalue;

  SysStatus cur_stat = SystemStatus.GetCurrentStatus();

  if (cur_stat == SYSTAT_PAUSE_TRIG || cur_stat == SYSTAT_END_TRIG)
    return;

  if (Percent > power_limit_)
    Percent = power_limit_;

  last_percent = Percent;
  integer = Percent;
  decimal = Percent - integer;
  pwmvalue = LaserPowerTable[integer] + (LaserPowerTable[integer + 1] - LaserPowerTable[integer]) * decimal;

  last_pwm = pwmvalue;

  CheckFan(pwmvalue);

  TimSetPwm(pwmvalue);
}

/**
 * Off:Laser off without changing the power
 */
void LaserExecuter::Off()
{
  CheckFan(0);
  TimSetPwm(0);
}

/**
 * On:Laser on and use the last power
 */
void LaserExecuter::On()
{
  SysStatus cur_stat = SystemStatus.GetCurrentStatus();

  if (cur_stat == SYSTAT_PAUSE_TRIG || cur_stat == SYSTAT_END_TRIG) {
    LOG_W("cannot open laser during pause/stop triggered!\n");
    return;
  }
  CheckFan(last_pwm);
  TimSetPwm(last_pwm);
}


/**
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserLowPower()
{
  SetLaserPower(0.5f);
}

#if ENABLED(EXECUTER_CANBUS_SUPPORT)
/**
 * SaveFocusHeight:Save the focus height
 * para height:the height of focus
 */
void LaserExecuter::SaveFocusHeight(float height)
{
  uint8_t Data[8];
  uint32_t intheight;
  intheight = height * 1000;
  if(FocusHeight > 65000)
    FocusHeight = 65000;

  Data[0] = (uint8_t)(intheight >> 8);
  Data[1] = (uint8_t)(intheight);
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_LASER_FOCUS, Data, 2);
}

/**
 * SaveFocusHeight:Save the focus height
 */
void LaserExecuter::SaveFocusHeight()
{
  uint8_t Data[8];
  uint32_t intheight;
  if(FocusHeight > 65000)
    FocusHeight = 65000;
  intheight = FocusHeight * 1000;

  Data[0] = (uint8_t)(intheight >> 8);
  Data[1] = (uint8_t)(intheight);
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_LASER_FOCUS, Data, 2);
}

/**
 * LoadFocusHeight:Load focus height from laser executer
 * return :true if load success or else false
 */
bool LaserExecuter::LoadFocusHeight()
{
  millis_t tmptick;
  uint8_t Data[3];

  Data[0] = 0;
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_REPORT_LASER_FOCUS, Data, 1);
  // Delay for update
  tmptick = millis() + 50;
  while(tmptick > millis());
  return 0;
}
#endif // ENABLED(EXECUTER_CANBUS_SUPPORT)

/**
 * UpdateLaserPower:Update the laser power without output to the laser
 *para NewPower:New power the update
 */
void LaserExecuter::UpdateLaserPower(float NewPower) {
  last_percent = NewPower;
}

/**
 *PackedProtocal:Pack up the data in protocal
 */
void LaserExecuter::PackedProtocal(uint8_t *pData, uint16_t len)
{
  uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;

  while(len--)
    tmpBuff[i++] = *pData++;

  //重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;

	LASERSerial.write(tmpBuff, i);
}

/**
 * ReadWifiStatus:Read wifi connection status from buildin wifi module
 * para IP:The pointer to the IP buffer
 * return:0 for connected, 1 for disconnect, -1 for wifi unexisting
 */
char LaserExecuter::GetReply(uint8_t *Buff, millis_t Timeout) {

  int c;
  millis_t tmptick;
  uint16_t i;
  uint16_t j;
  uint16_t DataLen;
  uint32_t GetChecksum;
  uint32_t calCheck = 0;

  tmptick = millis() + Timeout;
  i = 0;
  while(1) {
    if(millis() > tmptick)
      return (char)-1;
    do {
      c = LASERSerial.read();
      if(c != -1) {
        if(i == 0) {
          if(c == 0xAA) Buff[i++] = c;
        }
        else if(i == 1) {
          if(c == 0x55) Buff[i++] = c;
          else i = 0;
        }
        //Check data length
        else if(i == 6) {
          if((Buff[2] ^ Buff[3]) == Buff[5]) Buff[i++] = c;
          else i = 0;
        }
        //Continue receive date
        else if(i > 6) Buff[i++] = c;
        else Buff[i++] = c;
      }
    }while(c != -1);

    if(i > 8) {
      DataLen = ((uint16_t)Buff[2] << 8) | Buff[3];
      if((DataLen + 8) <= i) {
        calCheck = 0;
      	for(j = 8;j<(DataLen + 8 - 1);j = j + 2)
      		calCheck += (((uint16_t)Buff[j] << 8) | Buff[j + 1]);
      	if((DataLen - 8) % 2)
      		calCheck += (uint8_t)Buff[DataLen + 8 -1];
      	while(calCheck > 0xffff)
      		calCheck = ((calCheck >> 16) & 0xffff) + (calCheck & 0xffff);
      	calCheck = (~calCheck) & 0xffff;
        GetChecksum = (uint32_t)((Buff[6] << 8) | Buff[7]);
        SERIAL_ECHOLN(calCheck);
        SERIAL_ECHOLN(GetChecksum);
        if(calCheck == GetChecksum) return 0;
      }
    }
  }
  return (char)-1;
}


/**
 * ReadWifiStatus:Read wifi connection status from buildin wifi module
 * para IP:The pointer to the IP buffer
 * return:0 for connected, 1 for disconnect, -1 for wifi unexisting
 */
char LaserExecuter::ReadWifiStatus(char *SSID, char *Password, char *IP) {
  uint16_t i;
  uint16_t j;
  uint8_t buff[70];
  uint8_t v;

  LASERSerial.flush();
  buff[0] = 0x03;
  PackedProtocal(buff, 1);
  if(GetReply(buff, 1500) == 0) {
    if((buff[8] == 4) && (buff[9] == 0)) {
      j = 11;
      for(i=0;i<32;i++) {
        v = buff[j++];
        if(v == 0) break;
        SSID[i] = v;
      }
      SSID[i] = 0;

      for(i=0;i<32;i++) {
        v = buff[j++];
        if(v == 0) break;
        Password[i] = v;
      }
      Password[i] = 0;

      for(i=0;i<15;i++) {
        v = buff[j++];
        if(v == 0) break;
        IP[i] = v;
      }
      IP[i] = 0;
      if(buff[10] == 1) return 0;
      else return (char)1;
    }
  }
  return (char)2;
}

/**
 * SetWifiParameter:Set wifi parameter
 * para SSID:
 * para Password:
 */
char LaserExecuter::SetWifiParameter(char *SSID, char *Password)
{
  uint8_t i;
  uint8_t j;
  uint8_t Buff[90];

  i = 0;
  Buff[i++] = 0x01;
  for(j=0;j<32;j++)
  {
    if(SSID[j] == 0)
      break;
    Buff[i++] = SSID[j];
  }
  Buff[i++] = 0;

  for(j=0;j<32;j++)
  {
    if(Password[j] == 0)
      break;
    Buff[i++] = Password[j];
  }
  Buff[i++] = 0;
  PackedProtocal(Buff, i);
  if(GetReply(Buff, 500) == 0) {
    if((Buff[8] == 0x02) && (Buff[9] == 0)) return 0;
  }
  return (char)-1;
}

/**
 * SetBluetoothName:Set BT name
 * para Name:The name of the BT
 * ret  None
 */
char LaserExecuter::SetBluetoothName(char *Name)
{
  uint8_t i;
  uint8_t j;
  uint8_t Buff[90];

  i = 0;
  Buff[i++] = 0x11;
  Buff[i++] = 0;
  for(j=0;j<32;j++)
  {
    if(Name[j] == 0)
      break;
    Buff[i++] = Name[j];
  }
  Buff[i++] = 0;

  PackedProtocal(Buff, i);
  if(GetReply(Buff, 500) == 0) {
    if((Buff[8] == 0x12) && (Buff[9] == 0)) return 0;
  }
  return (char)-1;
}

/**
 * ReadBlueToothName:Read BT Name
 * para Name:The pointer to the Name buffer
 * return:0 for read success, 1 for unname, 2 for timeout
 */
char LaserExecuter::ReadBluetoothName(char *Name) {
  uint16_t i;
  uint16_t j;
  uint8_t buff[70];
  uint8_t v;

  LASERSerial.flush();
  // Unused ,for reserved
  buff[0] = 0x13;
  buff[1] = 0;
  PackedProtocal(buff, 2);
  if(GetReply(buff, 1500) == 0) {
    if(buff[8] == 0x14) {
      if(buff[9] == 0) {
        j = 10;
        for(i=0;i<32;i++) {
          v = buff[j++];
          if(v == 0) break;
          Name[i] = v;
        }
        Name[i] = 0;
        return 0;
      }
      else {
        Name[0] = 0;
        return (char)1;
      }
    }
  }
  return (char)2;
}

/**
 * ReadBluetoothMac:Read BlueTooth's MAC
 * para Name:The pointer to the Name buffer
 * return:0 for read success,  2 for timeout
 */
char LaserExecuter::ReadBluetoothMac(uint8_t *Mac) {
  uint16_t i;
  uint8_t buff[32];

  LASERSerial.flush();
  // Unused ,for reserved
  buff[0] = 0x15;
  buff[1] = 0;
  PackedProtocal(buff, 2);
  if(GetReply(buff, 1500) == 0) {
    if(buff[8] == 0x16) {
      if(buff[9] == 0) {
        for(i=0;i<6;i++) {
          Mac[i] = buff[10 + i];
        }
        return 0;
      }
    }
  }
  return (char)2;
}


uint16_t LaserExecuter::GetTimPwm() {
  return Tim1GetCCR4();
}

/**
 * change power limit, will be call when open / close chamber door
*/
void LaserExecuter::ChangePowerLimit(float limit) {
  if (limit > LASER_POWER_NORMAL_LIMIT)
    limit = LASER_POWER_NORMAL_LIMIT;

  // if previous limit is larger than now, need to check need to lower current output
  if (last_percent > limit) {
    ChangePower(limit);

    if (GetTimPwm() > last_pwm) {
      // lower current output
      CheckFan(last_pwm);
      TimSetPwm(last_pwm);
    }
  }

  power_limit_ = limit;
}

/**
 * change power value, but not change output power
*/
void LaserExecuter::ChangePower(float percent) {
  int integer;
  float decimal;

  if (percent > power_limit_)
    percent = power_limit_;

  last_percent = percent;
  integer = percent;
  decimal = percent - integer;

  last_percent = percent;
  last_pwm = LaserPowerTable[integer] + (LaserPowerTable[integer + 1] - LaserPowerTable[integer]) * decimal;
}
