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

#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "variant.h"
/* Could be redefined in variant.h or using build_opt.h */
#ifndef SD_DATATIMEOUT
#define SD_DATATIMEOUT         100000000U
#endif

/*SD Card information structure */
#if defined (STM32F4xx) || defined(STM32F7xx) || defined(STM32L4xx)
#define HAL_SD_CardInfoTypedef         HAL_SD_CardInfoTypeDef
#define BSP_SD_CardInfo                HAL_SD_CardInfoTypeDef
#define HAL_SD_WideBusOperation_Config HAL_SD_ConfigWideBusOperation
#define HAL_SD_Get_CardInfo            HAL_SD_GetCardInfo
#endif

#define SD_CardInfo HAL_SD_CardInfoTypedef

/*SD status structure definition */
#define MSD_OK                   ((uint8_t)0x00)
#define MSD_ERROR                ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT ((uint8_t)0x02)

/* SD Exported Constants */
#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)
#define SD_DETECT_NONE           NUM_DIGITAL_PINS

/* SD Exported Functions */
uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_CSSet(GPIO_TypeDef *csport, uint32_t cspin);
uint8_t BSP_SD_DeInit(void);
uint8_t BSP_SD_ITConfig(void);

uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_Erase(uint64_t StartAddr, uint64_t EndAddr);
#if defined (STM32F4xx) || defined(STM32F7xx) || defined(STM32L4xx)
uint8_t BSP_SD_GetCardState(void);
#else /* (STM32F1xx) || defined(STM32F2xx) || defined(STM32L1xx) */
HAL_SD_TransferStateTypedef BSP_SD_GetStatus(void);
#endif
void    BSP_SD_GetCardInfo(HAL_SD_CardInfoTypedef *CardInfo);
uint8_t BSP_SD_IsDetected(void);

/* These __weak function can be surcharged by application code in case the current settings (e.g. DMA stream)
   need to be changed for specific needs */
void    BSP_SD_MspInit(SD_HandleTypeDef *hsd, void *Params);
void    BSP_SD_Detect_MspInit(SD_HandleTypeDef *hsd, void *Params);
void    BSP_SD_MspDeInit(SD_HandleTypeDef *hsd, void *Params);

#ifdef __cplusplus
}
#endif
