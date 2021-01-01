#ifndef __EEPROM_H
#define __EEPROM_H

#include <wirish.h>
#include "flash_stm32.h"
#include "../../../../../Marlin/src/core/macros.h"

// HACK ALERT. This definition may not match your processor
// To Do. Work out correct value for EEPROM_PAGE_SIZE on the STM32F103CT6 etc 
//#define MCU_GD32F105VET

#ifndef EEPROM_PAGE_SIZE
	#if defined (MCU_STM32F103RB)
		#define EEPROM_PAGE_SIZE	(uint16)0x400  /* Page size = 1KByte */
	#elif defined (MCU_STM32F103ZE) || defined (MCU_STM32F103RE) || defined (MCU_STM32F103RD)
		#define EEPROM_PAGE_SIZE	(uint16)0x800  /* Page size = 2KByte */
    #elif defined (MCU_GD32F105VE)
        #define EEPROM_PAGE_SIZE	(uint16)0x800  /* Page size = 2KByte */
	#else
		#error	"No MCU type specified. Add something like -DMCU_STM32F103RB to your compiler arguments (probably in a Makefile)."
	#endif
#endif

#ifndef EEPROM_START_ADDRESS
	#if defined (MCU_STM32F103RB)
		#define EEPROM_START_ADDRESS	((uint32)(0x8000000 + 128 * 1024 - 2 * EEPROM_PAGE_SIZE))
	#elif defined (MCU_STM32F103ZE) || defined (MCU_STM32F103RE)
		#define EEPROM_START_ADDRESS	((uint32)(0x8000000 + 512 * 1024 - 2 * EEPROM_PAGE_SIZE))
	#elif defined (MCU_STM32F103RD)
		#define EEPROM_START_ADDRESS	((uint32)(0x8000000 + 384 * 1024 - 2 * EEPROM_PAGE_SIZE))
    #elif defined (MCU_GD32F105VE)
    #define EEPROM_START_ADDRESS	((uint32)(FLASH_MARLIN_EEPROM))
    //#define EEPROM_START_ADDRESS	((uint32)(0x8000000 + 512 * 1024 - 2 * EEPROM_PAGE_SIZE))
	#else
		#error	"No MCU type specified. Add something like -DMCU_STM32F103RB to your compiler arguments (probably in a Makefile)."
	#endif
#endif

/* Pages 0 and 1 base and end addresses */
#define EEPROM_PAGE0_BASE		((uint32)(EEPROM_START_ADDRESS + 0x000))
#define EEPROM_PAGE1_BASE		((uint32)(EEPROM_START_ADDRESS + EEPROM_PAGE_SIZE))

/* Page status definitions */
#define EEPROM_ERASED			((uint16)0xFFFF)	/* PAGE is empty */
#define EEPROM_RECEIVE_DATA		((uint16)0xEEEE)	/* PAGE is marked to receive data */
#define EEPROM_VALID_PAGE		((uint16)0x0000)	/* PAGE containing valid data */

/* Page full define */
enum : uint16
	{
	EEPROM_OK				= ((uint16)0x0000),
	EEPROM_OUT_SIZE			= ((uint16)0x0081),
	EEPROM_BAD_ADDRESS		= ((uint16)0x0082),
	EEPROM_BAD_FLASH		= ((uint16)0x0083),
	EEPROM_NOT_INIT			= ((uint16)0x0084),
	EEPROM_SAME_VALUE		= ((uint16)0x0085),
	EEPROM_NO_VALID_PAGE	= ((uint16)0x00AB)
	};

#define EEPROM_DEFAULT_DATA		0xFFFF


class EEPROMClass
{
public:
	EEPROMClass(void);

	uint16 init(void);
	uint16 init(uint32, uint32, uint32);

	uint16 format(void);

	uint16 erases(uint16 *);
	uint16 read (uint16 address);
	uint16 read (uint16 address, uint16 *data);
	uint16 write(uint16 address, uint16 data);
	uint16 update(uint16 address, uint16 data);
	uint16 count(uint16 *);
	uint16 maxcount(void);

	uint32 PageBase0;
	uint32 PageBase1;
	uint32 PageSize;
	uint16 Status;
private:
	FLASH_Status EE_ErasePage(uint32);

	uint16 EE_CheckPage(uint32, uint16);
	uint16 EE_CheckErasePage(uint32, uint16);
	uint16 EE_Format(void);
	uint32 EE_FindValidPage(void);
	uint16 EE_GetVariablesCount(uint32, uint16);
	uint16 EE_PageTransfer(uint32, uint32, uint16);
	uint16 EE_VerifyPageFullWriteVariable(uint16, uint16);
};

extern EEPROMClass EEPROM;

#endif	/* __EEPROM_H */
