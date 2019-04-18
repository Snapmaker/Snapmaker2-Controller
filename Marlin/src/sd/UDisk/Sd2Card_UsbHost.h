#pragma once

//#define USB_DEBUG 1

#include "../SdFatConfig.h"
#include "../SdInfo.h"

#include "../../HAL/HAL_GD32F1/UsbHost/inc/usbh_core.h"
#include "../../HAL/HAL_GD32F1/UsbHost/inc/usb_bsp.h"
#include "../../HAL/HAL_GD32F1/UsbHost/inc/usbh_hcs.h"
#include "../../HAL/HAL_GD32F1/UsbHost/inc/usbh_stdreq.h"
#include "../../HAL/HAL_GD32F1/UsbHost/inc/usbh_core.h"
#include "../../HAL/HAL_GD32F1/UsbHost/inc/usb_hcd_int.h"
#include "../../HAL/HAL_GD32F1/UsbHost/inc/usbh_msc_scsi.h"
#include "../../HAL/HAL_GD32F1/UsbHost/inc/usbh_msc_bot.h"


extern volatile bool DiskInserted;
extern uint8_t UDiskStat;
extern USBH_Class_cb_TypeDef  USBH_MSC_cb;
extern USBH_Usr_cb_TypeDef USR_cb;

class Sd2Card {
  public:
    bool init(const uint8_t sckRateID/*=0*/, const pin_t chipSelectPin/*=SD_CHIP_SELECT_PIN*/);

    void idle();

    inline bool readStart(const uint32_t block)                             { pos = block; return true; }
    inline bool readData(uint8_t* dst)                                      { return readBlock(pos++, dst); }
    inline bool readStop() const                                            { return true; }

    inline bool writeStart(const uint32_t block, const uint32_t eraseCount) { UNUSED(eraseCount); pos = block; return true; }
    inline bool writeData(uint8_t* src)                                     { return writeBlock(pos++, src); }
    inline bool writeStop() const                                           { return true; }

    bool readBlock(uint32_t block, uint8_t* dst);
    bool writeBlock(uint32_t blockNumber, const uint8_t* src);

    static bool isInserted();
    void isr(void);

  private:
    char WaitUDisk(void);
    void __irq_otg_fs();
    int UsbWriteBlock(uint32_t sector, uint8_t *pBuff);
    int UsbReadBlock(uint32_t sector, uint8_t *pBuff);

  private:
    USB_OTG_CORE_HANDLE UsbhCoreHandle;
    USBH_HOST UsbHost;
    uint32_t pos;
};
