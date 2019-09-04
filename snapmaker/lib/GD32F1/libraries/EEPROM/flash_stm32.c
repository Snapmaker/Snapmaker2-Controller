//#include "libmaple.h"
#include "libmaple/util.h"
#include "libmaple/flash.h"
#include "flash_stm32.h"

#define FLASH_KEY1			((uint32)0x45670123)
#define FLASH_KEY2			((uint32)0xCDEF89AB)

/* Delay definition */
#define EraseTimeout		((uint32)0x000B0000)
#define ProgramTimeout		((uint32)0x00002000)

/**
  * @brief  Inserts a time delay.
  * @param  None
  * @retval None
  */
static void delay(void)
{
	__IO uint32 i = 0;
	for(i = 0xFF; i != 0; i--) { }
}

/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP or FLASH_COMPLETE
  */
FLASH_Status FLASH_GetStatus(void)
{
	if ((FLASH_BASE->SR & FLASH_SR_BSY) == FLASH_SR_BSY)
		return FLASH_BUSY;

	if ((FLASH_BASE->SR & FLASH_SR_PGERR) != 0)
		return FLASH_ERROR_PG;

	if ((FLASH_BASE->SR & FLASH_SR_WRPRTERR) != 0 )
		return FLASH_ERROR_WRP;

	if ((FLASH_BASE->SR & FLASH_OBR_OPTERR) != 0 )
		return FLASH_ERROR_OPT;

	return FLASH_COMPLETE;
}

/**
  * @brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
  * @param  Timeout: FLASH progamming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastOperation(uint32 Timeout)
{ 
	FLASH_Status status;

	/* Check for the Flash Status */
	status = FLASH_GetStatus();
	/* Wait for a Flash operation to complete or a TIMEOUT to occur */
	while ((status == FLASH_BUSY) && (Timeout != 0x00))
	{
		delay();
		status = FLASH_GetStatus();
		Timeout--;
	}
	if (Timeout == 0)
		status = FLASH_TIMEOUT;
	/* Return the operation status */
	return status;
}

/**
  * @brief  Erases a specified FLASH page.
  * @param  Page_Address: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ErasePage(uint32 Page_Address)
{
	FLASH_Status status = FLASH_COMPLETE;
	/* Check the parameters */
	ASSERT(IS_FLASH_ADDRESS(Page_Address));
	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(EraseTimeout);
  
	if(status == FLASH_COMPLETE)
	{
		/* if the previous operation is completed, proceed to erase the page */
		FLASH_BASE->CR |= FLASH_CR_PER;
		FLASH_BASE->AR = Page_Address;
		FLASH_BASE->CR |= FLASH_CR_STRT;

		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(EraseTimeout);
		if(status != FLASH_TIMEOUT)
		{
			/* if the erase operation is completed, disable the PER Bit */
			FLASH_BASE->CR &= ~FLASH_CR_PER;
		}
		FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);
	}
	/* Return the Erase Status */
	return status;
}

/**
  * @brief  Returns the FLASH Bank1 Status.
  * @note   This function can be used for all STM32F10x devices, it is equivalent
  *         to FLASH_GetStatus function.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP or FLASH_COMPLETE
  */
FLASH_Status FLASH_GetBank1Status(void)
{
  FLASH_Status flashstatus = FLASH_COMPLETE;
  
  if((FLASH_BASE->SR & FLASH_FLAG_BANK1_BSY) == FLASH_FLAG_BANK1_BSY) 
  {
    flashstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH_BASE->SR & FLASH_FLAG_BANK1_PGERR) != 0)
    { 
      flashstatus = FLASH_ERROR_PG;
    }
    else 
    {
      if((FLASH_BASE->SR & FLASH_FLAG_BANK1_WRPRTERR) != 0 )
      {
        flashstatus = FLASH_ERROR_WRP;
      }
      else
      {
        flashstatus = FLASH_COMPLETE;
      }
    }
  }
  /* Return the Flash Status */
  return flashstatus;
}

#if defined(STM32F10X_XL) || defined(GD32F305VG)
/**
  * @brief  Returns the FLASH Bank2 Status.
  * @note   This function can be used for STM32F10x_XL density devices.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *        FLASH_ERROR_WRP or FLASH_COMPLETE
  */
FLASH_Status FLASH_GetBank2Status(void)
{
  FLASH_Status flashstatus = FLASH_COMPLETE;
  
  if((FLASH_BASE->SR2 & (FLASH_FLAG_BANK2_BSY & 0x7FFFFFFF)) == (FLASH_FLAG_BANK2_BSY & 0x7FFFFFFF)) 
  {
    flashstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH_BASE->SR2 & (FLASH_FLAG_BANK2_PGERR & 0x7FFFFFFF)) != 0)
    { 
      flashstatus = FLASH_ERROR_PG;
    }
    else 
    {
      if((FLASH_BASE->SR2 & (FLASH_FLAG_BANK2_WRPRTERR & 0x7FFFFFFF)) != 0 )
      {
        flashstatus = FLASH_ERROR_WRP;
      }
      else
      {
        flashstatus = FLASH_COMPLETE;
      }
    }
  }
  /* Return the Flash Status */
  return flashstatus;
}
#endif /* STM32F10X_XL */


/**
  * @brief  Waits for a Flash operation on Bank1 to complete or a TIMEOUT to occur.
  * @note   This function can be used for all STM32F10x devices, 
  *         it is equivalent to FLASH_WaitForLastOperation.
  * @param  Timeout: FLASH programming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastBank1Operation(uint32_t Timeout)
{ 
  FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the Flash Status */
  status = FLASH_GetBank1Status();
  /* Wait for a Flash operation to complete or a TIMEOUT to occur */
  while((status == FLASH_FLAG_BANK1_BSY) && (Timeout != 0x00))
  {
    status = FLASH_GetBank1Status();
    Timeout--;
  }
  if(Timeout == 0x00 )
  {
    status = FLASH_TIMEOUT;
  }
  /* Return the operation status */
  return status;
}

#if defined(STM32F10X_XL) || defined(GD32F305VG)
/**
  * @brief  Waits for a Flash operation on Bank2 to complete or a TIMEOUT to occur.
  * @note   This function can be used only for STM32F10x_XL density devices.
  * @param  Timeout: FLASH programming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastBank2Operation(uint32_t Timeout)
{ 
  FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the Flash Status */
  status = FLASH_GetBank2Status();
  /* Wait for a Flash operation to complete or a TIMEOUT to occur */
  while((status == (FLASH_FLAG_BANK2_BSY & 0x7FFFFFFF)) && (Timeout != 0x00))
  {
    status = FLASH_GetBank2Status();
    Timeout--;
  }
  if(Timeout == 0x00 )
  {
    status = FLASH_TIMEOUT;
  }
  /* Return the operation status */
  return status;
}
#endif /* STM32F10X_XL */

/**
  * @brief  Programs a half word at a specified address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_ProgramHalfWord(uint32 Address, uint16 Data)
{
	FLASH_Status status = FLASH_BAD_ADDRESS;

	if (IS_FLASH_ADDRESS(Address))
	{
    #if defined(STM32F10X_XL) || defined(GD32F305VG)
      /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(ProgramTimeout);
    
    if(Address < ((uint32_t)0x807FFFF))
    {
      if(status == FLASH_COMPLETE)
      {
        /* if the previous operation is completed, proceed to program the new data */
        FLASH_BASE->CR |= FLASH_CR_PG;
    
        *(__IO uint16_t*)Address = Data;
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastBank1Operation(ProgramTimeout);

        /* Disable the PG Bit */
        FLASH_BASE->CR &= ~(FLASH_CR_PG | 0xFFFF0000);
      }
    }
    else
    {
      if(status == FLASH_COMPLETE)
      {
        /* if the previous operation is completed, proceed to program the new data */
        FLASH_BASE->CR2 |= FLASH_CR_PG;
    
        *(__IO uint16_t*)Address = Data;
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastBank2Operation(ProgramTimeout);

        /* Disable the PG Bit */
        FLASH_BASE->CR &= ~(FLASH_CR_PG | 0xFFFF0000);
      }
    }
    #else
  
		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(ProgramTimeout);
		if(status == FLASH_COMPLETE)
		{
			/* if the previous operation is completed, proceed to program the new data */
			FLASH_BASE->CR |= FLASH_CR_PG;
			*(__IO uint16*)Address = Data;
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(ProgramTimeout);
			if(status != FLASH_TIMEOUT)
			{
				/* if the program operation is completed, disable the PG Bit */
				FLASH_BASE->CR &= ~FLASH_CR_PG;
			}
			FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);
		}
    #endif
	}
	return status;
}

/**
  * @brief  Programs a word at a specified address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_ProgramWord(uint32 Address, uint32 Data)
{
	FLASH_Status status = FLASH_BAD_ADDRESS;
  __IO uint32_t tmp = 0;

  #if defined(STM32F10X_XL) || defined(GD32F305VG)
  if(Address < ((uint32_t)0x807FFFF - 2))
  { 
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastBank1Operation(ProgramTimeout); 
    if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to program the new first 
        half word */
      FLASH_BASE->CR |= FLASH_CR_PG;
  
      *(__IO uint16_t*)Address = (uint16_t)Data;
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(ProgramTimeout);
 
      if(status == FLASH_COMPLETE)
      {
        /* if the previous operation is completed, proceed to program the new second 
        half word */
        tmp = Address + 2;

        *(__IO uint16_t*) tmp = Data >> 16;
    
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation(ProgramTimeout);
        
        /* Disable the PG Bit */
        FLASH_BASE->CR &= ~(FLASH_CR_PG | 0xFFFF0000);
      }
      else
      {
        /* Disable the PG Bit */
        FLASH_BASE->CR &= ~(FLASH_CR_PG | 0xFFFF0000);
       }
    }
  }
  else if(Address == ((uint32_t)0x807FFFF - 1))
  {
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastBank1Operation(ProgramTimeout);

    if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to program the new first 
        half word */
      FLASH_BASE->CR |= FLASH_CR_PG;
  
      *(__IO uint16_t*)Address = (uint16_t)Data;

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastBank1Operation(ProgramTimeout);
      
	  /* Disable the PG Bit */
      FLASH_BASE->CR &= ~(FLASH_CR_PG | 0xFFFF0000);
    }
    else
    {
      /* Disable the PG Bit */
      FLASH_BASE->CR &= ~(FLASH_CR_PG | 0xFFFF0000);
    }

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastBank2Operation(ProgramTimeout);

    if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to program the new second 
      half word */
      FLASH_BASE->CR2 |= FLASH_CR_PG;
      tmp = Address + 2;

      *(__IO uint16_t*) tmp = Data >> 16;
    
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastBank2Operation(ProgramTimeout);
        
      /* Disable the PG Bit */
      FLASH_BASE->CR2 &= ~(FLASH_CR_PG | 0xFFFF0000);
    }
    else
    {
      /* Disable the PG Bit */
      FLASH_BASE->CR2 &= ~(FLASH_CR_PG | 0xFFFF0000);
    }
  }
  else
  {
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastBank2Operation(ProgramTimeout);

    if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to program the new first 
        half word */
      FLASH_BASE->CR2 |= FLASH_CR_PG;
  
      *(__IO uint16_t*)Address = (uint16_t)Data;
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastBank2Operation(ProgramTimeout);
 
      if(status == FLASH_COMPLETE)
      {
        /* if the previous operation is completed, proceed to program the new second 
        half word */
        tmp = Address + 2;

        *(__IO uint16_t*) tmp = Data >> 16;
    
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastBank2Operation(ProgramTimeout);
        
        /* Disable the PG Bit */
        FLASH_BASE->CR2 &= ~(FLASH_CR_PG | 0xFFFF0000);
      }
      else
      {
        /* Disable the PG Bit */
        FLASH_BASE->CR2 &= ~(FLASH_CR_PG | 0xFFFF0000);
      }
    }
  }
  #else
  if (IS_FLASH_ADDRESS(Address))
  {
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(ProgramTimeout);
    if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to program the new data */
      FLASH_BASE->CR |= FLASH_CR_PG;
      *(__IO uint32*)Address = Data;
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(ProgramTimeout);
      if(status != FLASH_TIMEOUT)
      {
        /* if the program operation is completed, disable the PG Bit */
        FLASH_BASE->CR &= ~FLASH_CR_PG;
      }
      FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);
    }
  }
  #endif
	return status;
}


/**
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
	/* Authorize the FPEC Access */
	FLASH_BASE->KEYR = FLASH_KEY1;
	FLASH_BASE->KEYR = FLASH_KEY2;
  #if defined(GD32F305VG)
  FLASH_BASE->KEYR2 = FLASH_KEY1;
  FLASH_BASE->KEYR2 = FLASH_KEY2;
  #endif
}

/**
  * @brief  Locks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
	/* Set the Lock Bit to lock the FPEC and the FCR */
	FLASH_BASE->CR |= FLASH_CR_LOCK;
  #if defined(GD32F305VG)
  FLASH_BASE->CR2 |= FLASH_CR_LOCK;
  #endif
}
