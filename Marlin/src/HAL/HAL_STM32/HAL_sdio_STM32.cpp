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

  #include "HAL_sdio_Stm32.h"

  static SD_HandleTypeDef uSdHandle;
  static uint32_t SD_detect_gpio_pin = GPIO_PIN_All;
  static GPIO_TypeDef *SD_detect_gpio_port = GPIOA;
  #if defined (STM32F4xx) || defined(STM32F7xx) || defined(STM32L4xx)
  #define SD_OK                         HAL_OK
  #define SD_TRANSFER_OK                ((uint8_t)0x00)
  #define SD_TRANSFER_BUSY              ((uint8_t)0x01)
  #else /* (STM32F1xx) || defined(STM32F2xx) || defined(STM32L1xx) */
  static SD_CardInfo uSdCardInfo;
  #endif

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

    if (SdCardInfo.CardType != CARD_SDHC_SDXC) {
      blockAddress = block * 512U;
    }
    else {
      blockAddress = block;
    }

    return BSP_SD_ReadBlocks((uint32_t *)dst, blockAddress, 512, 1) == MSD_OK;
  }

  bool SDIO_WriteBlock(uint32_t block, const uint8_t *src) {
    uint64_t blockAddress = 0;

    if (BSP_SD_GetCardState() != 0) {
      return false;
    }

    if (SdCardInfo.CardType != CARD_SDHC_SDXC) {
      blockAddress = block * 512U;
    }
    else {
      blockAddress = block;
    }

    return BSP_SD_WriteBlocks((uint32_t *)src, blockAddress, 512, 1) == MSD_OK;
  }

  uint32_t SDIO_GetCardSize(void) {
    return 0;
  }

/**
  * @brief  Initializes the SD card device with CS check if any.
  * @retval SD status
  */
uint8_t BSP_SD_Init(void)
{
  uint8_t sd_state = MSD_OK;
  GPIO_InitTypeDef gpio_init_structure;

  uSdHandle.Instance = SDIO;
  uSdHandle.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
  uSdHandle.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
  uSdHandle.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
  uSdHandle.Init.BusWide             = SDIO_BUS_WIDE_1B;
  uSdHandle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  uSdHandle.Init.ClockDiv            = SDIO_TRANSFER_CLK_DIV;

  if(SD_detect_gpio_pin != GPIO_PIN_All) {
    /* Msp SD Detect pin initialization */
    BSP_SD_Detect_MspInit(&uSdHandle, NULL);
    if(BSP_SD_IsDetected() != SD_PRESENT)   /* Check if SD card is present */
    {
      return MSD_ERROR_SD_NOT_PRESENT;
    }
  }

  /* Enable clocks */
  __HAL_RCC_SDIO_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Common GPIO configuration */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF12_SDIO;

  /* GPIOC configuration */
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;

  HAL_GPIO_Init(GPIOC, &gpio_init_structure);

  /* GPIOD configuration */
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /* HAL SD initialization */
#if defined (STM32F4xx) || defined(STM32F7xx) || defined(STM32L4xx)
  if(HAL_SD_Init(&uSdHandle) != SD_OK)
#else /* (STM32F1xx) || defined(STM32F2xx) || defined(STM32L1xx) */
  if(HAL_SD_Init(&uSdHandle, &uSdCardInfo) != SD_OK)
#endif
  {
    sd_state = MSD_ERROR;
  }

  /* Configure SD Bus width */
  if(sd_state == MSD_OK)
  {
    /* Enable wide operation */
    if(HAL_SD_WideBusOperation_Config(&uSdHandle, SDIO_BUS_WIDE_4B) != SD_OK)
    {
      sd_state = MSD_ERROR;
    }
    else
    {
      sd_state = MSD_OK;
    }
  }
  return  sd_state;
}

/**
  * @brief  Set the SD card device detect pin and port.
  * @param  csport one of the gpio port
  * @param  cspin one of the gpio pin
  * @retval SD status
  */
uint8_t BSP_SD_CSSet(GPIO_TypeDef *csport, uint32_t cspin)
{
  if(csport != 0) {
    SD_detect_gpio_pin = cspin;
    SD_detect_gpio_port = csport;
	return MSD_OK;
  }
  return MSD_ERROR;
}

/**
  * @brief  DeInitializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_DeInit(void)
{
  uint8_t sd_state = MSD_OK;

  uSdHandle.Instance = SDIO;

  /* HAL SD deinitialization */
  if(HAL_SD_DeInit(&uSdHandle) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  HAL_NVIC_DisableIRQ(SDIO_IRQn);

  __HAL_RCC_SDIO_CLK_DISABLE();

  return  sd_state;
}

/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval Returns 0
  */
uint8_t BSP_SD_ITConfig(void)
{
  uint8_t sd_state = MSD_OK;
  GPIO_InitTypeDef gpio_init_structure;
  IRQn_Type sd_detect_EXTI_IRQn = EXTI0_IRQn;

  /* Configure Interrupt mode for SD detection pin */
  gpio_init_structure.Pin = SD_detect_gpio_pin;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Mode = GPIO_MODE_IT_RISING_FALLING;
  HAL_GPIO_Init(SD_detect_gpio_port, &gpio_init_structure);

  if(SD_detect_gpio_pin == GPIO_PIN_0) {
      sd_detect_EXTI_IRQn = EXTI0_IRQn;
  } else {
    if(SD_detect_gpio_pin == GPIO_PIN_1) {
      sd_detect_EXTI_IRQn = EXTI1_IRQn;
    } else {
      if(SD_detect_gpio_pin == GPIO_PIN_2) {
        sd_detect_EXTI_IRQn = EXTI2_IRQn;
      } else {
        if(SD_detect_gpio_pin == GPIO_PIN_3) {
          sd_detect_EXTI_IRQn = EXTI3_IRQn;
        } else {
          if(SD_detect_gpio_pin == GPIO_PIN_4) {
            sd_detect_EXTI_IRQn = EXTI4_IRQn;
          } else {
            if((SD_detect_gpio_pin == GPIO_PIN_5) ||\
               (SD_detect_gpio_pin == GPIO_PIN_6) ||\
               (SD_detect_gpio_pin == GPIO_PIN_7) ||\
               (SD_detect_gpio_pin == GPIO_PIN_8) ||\
               (SD_detect_gpio_pin == GPIO_PIN_9)) {
              sd_detect_EXTI_IRQn = EXTI9_5_IRQn;
            } else {
              if((SD_detect_gpio_pin == GPIO_PIN_10) ||\
                 (SD_detect_gpio_pin == GPIO_PIN_11) ||\
                 (SD_detect_gpio_pin == GPIO_PIN_12) ||\
                 (SD_detect_gpio_pin == GPIO_PIN_13) ||\
                 (SD_detect_gpio_pin == GPIO_PIN_14) ||\
                 (SD_detect_gpio_pin == GPIO_PIN_15)) {
                sd_detect_EXTI_IRQn = EXTI15_10_IRQn;
              } else {
                sd_state = MSD_ERROR;
              }
            }
          }
        }
      }
    }
  }
  if(sd_state == MSD_OK) {
    /* Enable and set SD detect EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(sd_detect_EXTI_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ(sd_detect_EXTI_IRQn);
  }
  return sd_state;
}

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @retval Returns if SD is detected or not
 */
uint8_t BSP_SD_IsDetected(void)
{
  uint8_t  status = SD_PRESENT;

  /* Check SD card detect pin */
  if (HAL_GPIO_ReadPin(SD_detect_gpio_port, SD_detect_gpio_pin) == GPIO_PIN_SET)
  {
    status = SD_NOT_PRESENT;
  }

    return status;
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  BlockSize: SD card data block size, that should be 512
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  if(HAL_SD_ReadBlocks(&uSdHandle, (uint8_t *)pData, ReadAddr, 1, 512) != SD_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  BlockSize: SD card data block size, that should be 512
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks)
{
  if(HAL_SD_WriteBlocks(&uSdHandle, (uint8_t *)pData, WriteAddr, BlockSize, NumOfBlocks) != SD_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Initializes the SD Detect pin MSP.
  * @param  hsd: SD handle
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_SD_Detect_MspInit(SD_HandleTypeDef *hsd, void *Params)
{
  UNUSED(hsd);
  UNUSED(Params);
  GPIO_InitTypeDef  gpio_init_structure;

  /* GPIO configuration in input for uSD_Detect signal */
  gpio_init_structure.Pin       = SD_detect_gpio_pin;
  gpio_init_structure.Mode      = GPIO_MODE_INPUT;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(SD_detect_gpio_port, &gpio_init_structure);
}

#if defined (STM32F4xx) || defined(STM32F7xx) || defined(STM32L4xx)
/**
  * @brief  Gets the current SD card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_SD_GetCardState(void)
{
  return((HAL_SD_GetCardState(&uSdHandle) == HAL_SD_CARD_TRANSFER ) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}
#else /* (STM32F1xx) || defined(STM32F2xx) || defined(STM32L1xx) */
/**
  * @brief  Gets the current SD card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  *            @arg  SD_TRANSFER_ERROR: Data transfer error
  */
HAL_SD_TransferStateTypedef BSP_SD_GetStatus(void)
{
  return(HAL_SD_GetStatus(&uSdHandle));
}
#endif

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  */
void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_Get_CardInfo(&uSdHandle, CardInfo);
}

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
