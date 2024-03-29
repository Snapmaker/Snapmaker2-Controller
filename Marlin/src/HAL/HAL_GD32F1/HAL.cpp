/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2017 Victor Perez
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

/**
 * HAL for stm32duino.com based on Libmaple and compatible (STM32F1)
 */

#ifdef __GD32F1__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "HAL.h"
#include <STM32ADC.h>
#include "../../inc/MarlinConfig.h"

#include <libmaple/usart.h>
#include "HardwareSerial.h"
// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

#define __I
#define __IO volatile
 typedef struct
 {
   __I  uint32_t CPUID;                   /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
   __IO uint32_t ICSR;                    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
   __IO uint32_t VTOR;                    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
   __IO uint32_t AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
   __IO uint32_t SCR;                     /*!< Offset: 0x010 (R/W)  System Control Register                               */
   __IO uint32_t CCR;                     /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
   __IO uint8_t  SHP[12];                 /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
   __IO uint32_t SHCSR;                   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
   __IO uint32_t CFSR;                    /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register                    */
   __IO uint32_t HFSR;                    /*!< Offset: 0x02C (R/W)  HardFault Status Register                             */
   __IO uint32_t DFSR;                    /*!< Offset: 0x030 (R/W)  Debug Fault Status Register                           */
   __IO uint32_t MMFAR;                   /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register                      */
   __IO uint32_t BFAR;                    /*!< Offset: 0x038 (R/W)  BusFault Address Register                             */
   __IO uint32_t AFSR;                    /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       */
   __I  uint32_t PFR[2];                  /*!< Offset: 0x040 (R/ )  Processor Feature Register                            */
   __I  uint32_t DFR;                     /*!< Offset: 0x048 (R/ )  Debug Feature Register                                */
   __I  uint32_t ADR;                     /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register                            */
   __I  uint32_t MMFR[4];                 /*!< Offset: 0x050 (R/ )  Memory Model Feature Register                         */
   __I  uint32_t ISAR[5];                 /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register                   */
        uint32_t RESERVED0[5];
   __IO uint32_t CPACR;                   /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register                   */
 } SCB_Type;

// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address  */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address  */

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct           */

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos              16                                             /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */

#define SCB_AIRCR_PRIGROUP_Pos              8                                             /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------
#ifdef SERIAL_USB
  USBSerial SerialUSB;
#endif

uint16_t HAL_adc_result;

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------
STM32ADC adc(ADC1);

uint8_t adc_pins[] = {
  #if HAS_TEMP_ADC_0
    TEMP_0_PIN,
  #endif
  #if HAS_HEATED_BED
    TEMP_BED_PIN,
  #endif
  #if HAS_HEATED_CHAMBER
    TEMP_CHAMBER_PIN,
  #endif
  #if HAS_TEMP_ADC_1
    TEMP_1_PIN,
  #endif
  #if HAS_TEMP_ADC_2
    TEMP_2_PIN,
  #endif
  #if HAS_TEMP_ADC_3
    TEMP_3_PIN,
  #endif
  #if HAS_TEMP_ADC_4
    TEMP_4_PIN,
  #endif
  #if HAS_TEMP_ADC_5
    TEMP_5_PIN,
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    FILWIDTH_PIN,
  #endif
};

enum TEMP_PINS : char {
  #if HAS_TEMP_ADC_0
    TEMP_0,
  #endif
  #if HAS_HEATED_BED
    TEMP_BED,
  #endif
  #if HAS_HEATED_CHAMBER
    TEMP_CHAMBER,
  #endif
  #if HAS_TEMP_ADC_1
    TEMP_1,
  #endif
  #if HAS_TEMP_ADC_2
    TEMP_2,
  #endif
  #if HAS_TEMP_ADC_3
    TEMP_3,
  #endif
  #if HAS_TEMP_ADC_4
    TEMP_4,
  #endif
  #if HAS_TEMP_ADC_5
    TEMP_5,
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    FILWIDTH,
  #endif
    ADC_PIN_COUNT
};

uint16_t HAL_adc_results[ADC_PIN_COUNT];


// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------
static void NVIC_SetPriorityGrouping(uint32_t PriorityGroup) {
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);               /* only values 0..7 are used          */

  reg_value  =  SCB->AIRCR;                                                   /* read old register configuration    */
  reg_value &= ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk);             /* clear bits to change               */
  reg_value  =  (reg_value                                 |
                ((uint32_t)0x5FA << SCB_AIRCR_VECTKEY_Pos) |
                (PriorityGroupTmp << 8));                                     /* Insert write key and priorty group */
  SCB->AIRCR =  reg_value;
}

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

//
// Leave PA11/PA12 intact if USBSerial is not used
//
#if SERIAL_USB
  namespace wirish { namespace priv {
    #if SERIAL_PORT > 0
      #if SERIAL_PORT2
        #if SERIAL_PORT2 > 0
          void board_setup_usb(void) {}
        #endif
      #else
        void board_setup_usb(void) {}
      #endif
    #endif
  } }
#endif

void HAL_init(void) {
  // choose group 4, all bits is for preemption
  NVIC_SetPriorityGrouping(0x3);
}

/* VGPV Done with defines
// disable interrupts
void cli(void) { noInterrupts(); }

// enable interrupts
void sei(void) { interrupts(); }
*/

void HAL_clear_reset_source(void) {
  RCC_BASE->CSR |= RCC_CSR_RMVF_BIT;
}

uint8_t HAL_get_reset_source(void) { 
  uint8_t sta = 0;
  uint32_t reg = RCC_BASE->CSR;
  if (reg & (RCC_CSR_WWDGRSTF_BIT | RCC_CSR_IWDGRSTF_BIT)) {
    sta |= RST_WATCHDOG;
  }

  if (reg & RCC_CSR_SFTRSTF_BIT) {
    sta |= RST_SOFTWARE;
  }

  if (reg & RCC_CSR_PINRSTF_BIT) {
    sta |= RST_EXTERNAL;
  }

  if (reg & RCC_CSR_PORRSTF_BIT) {
    sta |= RST_POWER_ON;
  }

  if (reg & RCC_CSR_LPWRRSTF_BIT) {
    sta |= RST_BROWN_OUT;
  }

  return sta; 
}

void _delay_ms(const int delay_ms) { delay(delay_ms); }

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

/**
 * TODO: Change this to correct it for libmaple
 */

// return free memory between end of heap (or end bss) and whatever is current

/*
#include "wirish/syscalls.c"
//extern caddr_t _sbrk(int incr);
#ifndef CONFIG_HEAP_END
extern char _lm_heap_end;
#define CONFIG_HEAP_END ((caddr_t)&_lm_heap_end)
#endif

extern "C" {
  static int freeMemory() {
    char top = 't';
    return &top - reinterpret_cast<char*>(sbrk(0));
  }
  int freeMemory() {
    int free_memory;
    int heap_end = (int)_sbrk(0);
    free_memory = ((int)&free_memory) - ((int)heap_end);
    return free_memory;
  }
}
*/

// --------------------------------------------------------------------------
// ADC
// --------------------------------------------------------------------------
// Init the AD in continuous capture mode
void HAL_adc_init(void) {
  // configure the ADC
  adc.calibrate();
  adc.setSampleRate(ADC_SMPR_41_5); // ?
  adc.setPins(adc_pins, ADC_PIN_COUNT);
  adc.setDMA(HAL_adc_results, (uint16_t)ADC_PIN_COUNT, (uint32_t)(DMA_MINC_MODE | DMA_CIRC_MODE), (void (*)())NULL);
  adc.setScanMode();
  adc.setContinuous();
  adc.startConversion();
}

void HAL_adc_start_conversion(const uint8_t adc_pin) {
  TEMP_PINS pin_index = TEMP_0;
  switch (adc_pin) {
    #if HAS_TEMP_ADC_0
      case TEMP_0_PIN: pin_index = TEMP_0; break;
    #endif
    #if HAS_HEATED_BED
      case TEMP_BED_PIN: pin_index = TEMP_BED; break;
    #endif
    #if HAS_HEATED_CHAMBER
      case TEMP_CHAMBER_PIN: pin_index = TEMP_CHAMBER; break;
    #endif
    #if HAS_TEMP_ADC_1
      case TEMP_1_PIN: pin_index = TEMP_1; break;
    #endif
    #if HAS_TEMP_ADC_2
      case TEMP_2_PIN: pin_index = TEMP_2; break;
    #endif
    #if HAS_TEMP_ADC_3
      case TEMP_3_PIN: pin_index = TEMP_3; break;
    #endif
    #if HAS_TEMP_ADC_4
      case TEMP_4_PIN: pin_index = TEMP_4; break;
    #endif
    #if HAS_TEMP_ADC_5
      case TEMP_5_PIN: pin_index = TEMP_5; break;
    #endif
    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      case FILWIDTH_PIN: pin_index = FILWIDTH; break;
    #endif
  }
  HAL_adc_result = (HAL_adc_results[(int)pin_index] >> 2) & 0x3FF; // shift to get 10 bits only.
}

uint16_t HAL_adc_get_result(void) {
  return HAL_adc_result;
}

// inline uint32_t SaveSR(void)
// {
// 	__asm
// 	(
// 	"mrs r0, primask\n"
// 	"CPSID I\n"
// 	"bx lr"
// 	);
// }

// inline void RestoreSR(uint32_t Sr)
// {
// 	__asm
// 	(
// 	"msr primask, r0\n"
// 	"bx lr"
// 	);
// }

// void EnterCritical(uint8_t option)
// {
// 	static uint32_t cpu_sr = 0;
// 	if(option == 0)
// 		RestoreSR(cpu_sr);
// 	else
// 		cpu_sr = SaveSR();
// }

void HAL_uart_reset_rx(HardwareSerial &serial) {
  usart_reset_rx(serial.c_dev());
}

#endif // __STM32F1__
