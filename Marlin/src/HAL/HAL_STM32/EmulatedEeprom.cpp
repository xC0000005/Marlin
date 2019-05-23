/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

 #if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

/**
 * Description: functions flash emulation EEPROM.
 * Not platform dependent.
 */

#include "../../inc/MarlinConfig.h"

#include <EEPROM.h>

#if ENABLED(EEPROM_SETTINGS) && DISABLED(I2C_EEPROM, SPI_EEPROM)

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "HAL.h"


// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// EEPROM
// --------------------------------------------------------------------------


void eeprom_write_byte(uint8_t *pos, unsigned char value) {
  uint16_t eeprom_address = (unsigned) pos;
  EEPROM[eeprom_address] = value;
}

uint8_t eeprom_read_byte(uint8_t *pos) {
  uint16_t data = 0xFF;
  data = EEPROM[*pos];
  return (unsigned char)data;
}

void eeprom_read_block(void *__dst, const void *__src, size_t __n) {
  uint16_t data = 0xFF;
  uint16_t eeprom_address = (unsigned) __src;

  for (uint8_t c = 0; c < __n; c++) {
    data = EEPROM[eeprom_address + c];
    *((uint8_t*)__dst + c) = data;
  }
}

void eeprom_update_block(const void *__src, void *__dst, size_t __n) {

}

#endif // EEPROM_SETTINGS && (!I2C_EEPROM && !SPI_EEPROM)
#endif // STM32GENERIC && STM32F4
