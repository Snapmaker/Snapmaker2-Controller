/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2016 Victor Perez victor_pv@hotmail.com
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
 * persistent_store_flash.cpp
 * HAL for stm32duino and compatible (STM32F1)
 * Implementation of EEPROM settings in SDCard
 */

#ifdef __GD32F1__

#include "../../inc/MarlinConfig.h"

// This is for EEPROM emulation in flash
#if BOTH(EEPROM_SETTINGS, FLASH_EEPROM_EMULATION)

#include "../shared/persistent_store_api.h"

#include <flash_stm32.h>
#include <EEPROM.h>

// Store settings in the last two pages
// Flash pages must be erased before writing, so keep track.
bool firstWrite = false;
uint32_t pageBase = EEPROM_START_ADDRESS;

#define HAL_GD32F1_EEPROM_SIZE 4096
char HAL_GD32F1_eeprom_content[HAL_GD32F1_EEPROM_SIZE];


bool PersistentStore::access_start() {
  uint32_t Address;
  uint16_t *pBuff;
  Address = pageBase;
  pBuff = (uint16_t*)HAL_GD32F1_eeprom_content;
  for (int i=0;i<HAL_GD32F1_EEPROM_SIZE;i=i+2) {
      *pBuff++ = *((uint16_t*)Address);
      Address += 2;
  }
  firstWrite = true;
  return true;
}

bool PersistentStore::access_finish() {
  uint32_t Address;
  uint16_t *pu16value;


  Address = pageBase;
  pu16value = (uint16_t*)HAL_GD32F1_eeprom_content;
  FLASH_Unlock();
  FLASH_ErasePage(EEPROM_PAGE0_BASE);
  FLASH_ErasePage(EEPROM_PAGE1_BASE); 
  for (int i=0;i<HAL_GD32F1_EEPROM_SIZE;i=i+2) {
    FLASH_ProgramHalfWord(Address, *pu16value++);
    Address += 2;
  }
  FLASH_Lock();
  return true;

}

bool PersistentStore::write_data(int &pos, const uint8_t *value, const size_t size, uint16_t *crc) {
  int bytewritten = 0;
  int bytetowrite = size;
  uint8_t* Buff = (uint8_t *)value;
  while (bytetowrite--) {
    HAL_GD32F1_eeprom_content[pos++] = *Buff++;
    bytewritten++;
  }

  crc16(crc, value, size);
  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t* value, const size_t size, uint16_t *crc, const bool writing/*=true*/) {
  for (uint16_t i = 0; i < size; i++) {
    uint8_t c = HAL_GD32F1_eeprom_content[pos + i];
    if (writing) value[i] = c;
    crc16(crc, &c, 1);
  }
  pos += size;
  return false;
}

size_t PersistentStore::capacity() { return 4095; }

#endif // EEPROM_SETTINGS && EEPROM FLASH
#endif // __GD32F1__
