/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2017 Victor Perez
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

  #include "../../inc/MarlinConfig.h"

#ifdef NOT_LIB
  //#include "sdio_sd.h"

  bool SDIO_Init(void) {
  }

  bool SDIO_ReadBlock(uint32_t block, uint8_t *dst) {
  }

  bool SDIO_WriteBlock(uint32_t block, const uint8_t *src) {
  }

  uint32_t SDIO_GetCardSize(void) {
    return 0;
  }

#else
  #include "HAL_sdio_Stm32.h"

  SD_CardInfo SdCardInfo;

  bool SDIO_Init(void) {
    (void)BSP_SD_DeInit();

  	if (BSP_SD_Init() == MSD_OK) {
  		BSP_SD_GetCardInfo(&SdCardInfo);
  		return true;
  	}
  	return false;
  }

  bool SDIO_ReadBlock(uint32_t block, uint8_t *dst) {
    uint64_t blockAddress = 0;

    if (BSP_SD_GetCardState() != 0) {
      return false;
    }

    //if (SdCardInfo.CardType != CARD_SDHC_SDXC) {
      blockAddress = block * 512U;
    //}
    //else {
    //  blockAddress = block;
    //}

    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR("Reading Block   ", block);
    SERIAL_ECHOLNPAIR(" To address ", (uint32_t)dst);
    SERIAL_ECHOLNPAIR("Adjusted Address   ", (uint32_t)blockAddress);
    SERIAL_FLUSH();

    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR("Adjusted Address   ", (uint32_t)blockAddress);
    SERIAL_FLUSH();

    bool result = BSP_SD_ReadBlocks((uint32_t *)dst, blockAddress, 512, 1);

    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR("Read Result   ", result);
    SERIAL_FLUSH();

    return result == MSD_OK;
  }

  bool SDIO_WriteBlock(uint32_t block, const uint8_t *src) {
    uint64_t blockAddress = 0;

    if (BSP_SD_GetCardState() != 0) {
      return false;
    }

    //if (SdCardInfo.CardType != CARD_SDHC_SDXC) {
      blockAddress = block * 512U;
    //}
    //else {
    //  blockAddress = block;
    //}

    return BSP_SD_WriteBlocks((uint32_t *)src, blockAddress, 512, 1) == MSD_OK;
  }

  uint32_t SDIO_GetCardSize(void) {
    return 0;
  }
#endif // SDIO LIABLE

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
