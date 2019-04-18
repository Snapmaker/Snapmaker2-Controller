/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(USB_HOST_UDISK_SUPPORT)

#include "../../core/serial.h"

#include "Sd2Card_Usbhost.h"

void Sd2Card::idle() {
  USBH_Process(&UsbhCoreHandle, &UsbHost);
}

// Marlin calls this function to check whether an USB drive is inserted.
// This is equivalent to polling the SD_DETECT when using SD cards.
bool Sd2Card::isInserted() {
 return DiskInserted;
}

// Marlin calls this to initialize an SD card once it is inserted.
bool Sd2Card::init(const uint8_t sckRateID/*=0*/, const pin_t chipSelectPin/*=SD_CHIP_SELECT_PIN*/) {
  UDiskStat = 1;
  USBH_Init(&UsbhCoreHandle, USB_OTG_FS_CORE_ID, &UsbHost, &USBH_MSC_cb, &USR_cb);
  SERIAL_ECHOLNPGM("Udisk Inited!");
  return true;
}

bool Sd2Card::readBlock(uint32_t block, uint8_t* dst) {
  WaitUDisk();
  if(UsbReadBlock(block, dst) == 0)
    return true;
  else
    return false;
}

bool Sd2Card::writeBlock(uint32_t block, const uint8_t* src) {
  WaitUDisk();
  if(UsbWriteBlock(block, (uint8_t*)src) == 0)
    return true;
  else
    return false;
}

int Sd2Card::UsbReadBlock(uint32_t sector, uint8_t *pBuff) {
  uint8_t status;
  status = 0;
  if(WaitUDisk() == 0)
  {
    if (HCD_IsDeviceConnected(&UsbhCoreHandle)) {
  		do
  		{
  			status = USBH_MSC_Read10(&UsbhCoreHandle, pBuff, sector, 512);
  			USBH_MSC_HandleBOTXfer(&UsbhCoreHandle, &UsbHost);

  			if (!HCD_IsDeviceConnected(&UsbhCoreHandle)) { 
  				return 1;
  			}
  		} while ((status == USBH_MSC_BUSY));
  	}

  	if (status == USBH_MSC_OK) {
  		return 0;
  	}
  }
	return 1;
}

int Sd2Card::UsbWriteBlock(uint32_t sector, uint8_t *pBuff) {
  uint8_t status;
  status = 0;
  if(WaitUDisk() == 0)
  {
    if (HCD_IsDeviceConnected(&UsbhCoreHandle)) {
  		do
  		{
  			status = USBH_MSC_Write10(&UsbhCoreHandle, pBuff, sector, 512);
  			USBH_MSC_HandleBOTXfer(&UsbhCoreHandle, &UsbHost);

  			if (!HCD_IsDeviceConnected(&UsbhCoreHandle)) {
  				return 1;
  			}
  		} while (status == USBH_MSC_BUSY);
  	}

  	if (status == USBH_MSC_OK) {
  		return 0;
  	}
  }
	return 1;
}

//等待U  盘操作
char Sd2Card::WaitUDisk(void)
{
  uint32_t tmptick;
	UDiskStat = 1;
	tmptick = millis();
	while(UDiskStat)
	{
		USBH_Process(&UsbhCoreHandle, &UsbHost);
		if(UsbhCoreHandle.host.ConnSts == 0)
			return (char)-2;
		if((millis() - tmptick) > (4 * 1000))
			return (char)-1;
	}
	return 0;
}

void Sd2Card::isr(void)
{
  USBH_OTG_ISR_Handler(&UsbhCoreHandle);
}

#endif // USB_HOST_UDISK_SUPPORT