#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "LaserExecuter.h"
#include "CanBus.h"

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
  LastPercent = Percent;
  integer = Percent;
  decimal = Percent - integer;
  pwmvalue = LaserPowerTable[integer] + (LaserPowerTable[integer + 1] - LaserPowerTable[integer]) * decimal;
  TimSetPwm(pwmvalue);
}

/**
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserPower(uint16_t PwmValue)
{
  TimSetPwm(PwmValue);
}

/**
 * LaserOff:Laser off without changing the power
 */
void LaserExecuter::LaserOff()
{
  TimSetPwm(0);
}

/**
 * LaserOn:Laser on and use the last power
 */
void LaserExecuter::LaserOn()
{
  SetLaserPower(LastPercent);
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
void LaserExecuter::SaveFocusHeight(uint8_t index, float height)
{
  uint8_t Data[8];
  uint32_t intheight;
  intheight = height * 1000;

  Data[0] = index;
  Data[1] = 0;
  Data[2] = (uint8_t)(intheight >> 24);
  Data[3] = (uint8_t)(intheight >> 16);
  Data[4] = (uint8_t)(intheight >> 8);
  Data[5] = (uint8_t)(intheight);

}

/**
 * SaveFocusHeight:Save the focus height
 */
void LaserExecuter::SaveFocusHeight()
{
  uint8_t Data[8];
  uint32_t intheight;
  intheight = FocusHeight * 1000;

  Data[0] = LastSetIndex;
  Data[1] = 0;
  Data[2] = (uint8_t)(intheight >> 24);
  Data[3] = (uint8_t)(intheight >> 16);
  Data[4] = (uint8_t)(intheight >> 8);
  Data[5] = (uint8_t)(intheight);
  
}

/**
 * LoadFocusHeight:Load focus height from laser executer
 * return :true if load success or else false
 */
bool LaserExecuter::LoadFocusHeight()
{
  uint8_t Data[3];

  Data[0] = LastSetIndex;
  Data[1] = 2;
 
}
#endif // ENABLED(EXECUTER_CANBUS_SUPPORT)

/**
 * SavePlatformHeight:Save the focus platform heigh
 *para height:The value of the platform height
 */
void LaserExecuter::SavePlatformHeight(float height)
{
  PlatformHeight = height;
  settings.save();
}

/**
 * LoadPlatformHeight:Load the focus height
 * para height:the height of focus
 */
void LaserExecuter::LoadPlatformHeight()
{
  settings.load();
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
char LaserExecuter::ReadWifiStatus(char *SSID, char *Password, char *IP)
{
  int c;
  millis_t tmptick;
  uint16_t i;
  uint16_t j;
  uint8_t buff[70];
  char v;
  
  buff[0] = 0x03;
  PackedProtocal(buff, 1);
  tmptick = millis() + 500;
  i = 0;

  buff[2] = 0xff;
  buff[3] = 0xff;
  buff[5] = 0xff;
  IP[0] = 0;
  while(1)
  {
    if(millis() > tmptick)
      return (char)-1;
    do
    {
      c = LASERSerial.read();
      if(c != -1)
      {
        if(i == 0)
        {
          if(c == 0xAA)
            buff[i++] = c;
        }
        else if(i == 1)
        {
          if(c == 0x55)
            buff[i++] = c;
          else
            i = 0;
        }
        //Check data length
        else if(i == 6)
        {
          if((buff[2] ^ buff[3]) == buff[5])
            buff[i++] = c;
          else
            i = 0;
        }
        //Continue receive date
        else if(i > 6)
        {
          buff[i++] = c;
        }
        else
        {
          buff[i++] = c;
        }
      }
    }while(c != -1);

    //Data receive complete
    if((i >= (buff[3] + 8)) && (i > 8))
    {
      uint32_t calCheck = 0;
      uint32_t GetChecksum;
      calCheck = 0;
    	for(j = 8;j<(i - 1);j = j + 2)
    		calCheck += (((uint16_t)buff[j] << 8) | buff[j + 1]);
    	if((i - 8) % 2)
    		calCheck += (uint8_t)buff[i-1];
    	while(calCheck > 0xffff)
    		calCheck = ((calCheck >> 16) & 0xffff) + (calCheck & 0xffff);
    	calCheck = (~calCheck) & 0xffff;
      GetChecksum = (uint32_t)((buff[6] << 8) | buff[7]);
      SERIAL_ECHOLN(calCheck);
      SERIAL_ECHOLN(GetChecksum);
      if(calCheck == GetChecksum)
      {
        j = 11;
        for(i=0;i<16;i++)
        {
          v = buff[j++];
          if(v == 0)
            break;
          SSID[i] = v;
        }
        SSID[i] = 0;

        for(i=0;i<31;i++)
        {
          v = buff[j++];
          if(v == 0)
            break;
          Password[i] = v;
        }
        Password[i] = 0;

        for(i=0;i<31;i++)
        {
          v = buff[j++];
          if(v == 0)
            break;
          IP[i] = v;
        }
        IP[i] = 0;
        
        return (char)buff[10];
      }
      return (char)-1;
    }
  }
}

/**
 * SetWifiParameter:Set wifi parameter
 * para SSID:
 * para Password:
 */
void LaserExecuter::SetWifiParameter(char *SSID, char *Password)
{
  uint8_t i;
  uint8_t j;
  uint8_t SendBuff[70];
  
  i = 0;
  SendBuff[i++] = 0x01;
  for(j=0;j<32;j++)
  {
    if(SSID[j] == 0)
      break;
    SendBuff[i++] = SSID[j];
  }
  SendBuff[i++] = 0;
  
  for(j=0;j<32;j++)
  {
    if(Password[j] == 0)
      break;
    SendBuff[i++] = Password[j];
  }
  SendBuff[i++] = 0;
  PackedProtocal(SendBuff, i);
}


