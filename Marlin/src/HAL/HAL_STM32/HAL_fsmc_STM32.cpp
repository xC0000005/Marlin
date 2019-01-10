/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016, 2017 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * u8g_com_stm32duino_fsmc.cpp
 *
 * Communication interface for FSMC
 */

 #include "../../inc/MarlinConfig.h"

#if HAS_GRAPHICAL_LCD
#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "U8glib.h"

static uint32_t FSMC_Initialized = 0;

static void HAL_FSMC_MspInit(void){
  /* USER CODE BEGIN FSMC_MspInit 0 */

  /* USER CODE END FSMC_MspInit 0 */
  GPIO_InitTypeDef GPIO_InitStruct;
  if (FSMC_Initialized) {
    return;
  }
  FSMC_Initialized = 1;

  __HAL_RCC_FSMC_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* GPIO_InitStruct */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

#define LCD_READ_ID     0x04   /* Read display identification information */

/* Timing configuration */
#define FSMC_ADDRESS_SETUP_TIME   15  // AddressSetupTime
#define FSMC_DATA_SETUP_TIME      15  // DataSetupTime

void LCD_IO_Init(uint8_t cs, uint8_t rs);
void LCD_IO_WriteData(uint16_t RegValue);
void LCD_IO_WriteReg(uint8_t Reg);
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);

static uint8_t msgInitCount = 2; // Ignore all messages until 2nd U8G_COM_MSG_INIT

uint8_t u8g_com_st_core_fsmc_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  if (msgInitCount) {
    if (msg == U8G_COM_MSG_INIT) msgInitCount--;
    if (msgInitCount) return -1;
  }

  static uint8_t isCommand;

  switch(msg) {
    case U8G_COM_MSG_STOP:
      break;
    case U8G_COM_MSG_INIT:
      u8g_SetPIOutput(u8g, U8G_PI_RESET);

      LCD_IO_Init(u8g->pin_list[U8G_PI_CS], u8g->pin_list[U8G_PI_A0]);
      u8g_Delay(100);

      if (arg_ptr != NULL)
        *((uint32_t *)arg_ptr) = LCD_IO_ReadData(LCD_READ_ID, 3);

      isCommand = 0;
      break;

    case U8G_COM_MSG_ADDRESS:           // define cmd (arg_val = 0) or data mode (arg_val = 1)
      isCommand = arg_val == 0 ? 1 : 0;
      break;

    case U8G_COM_MSG_RESET:
      u8g_SetPILevel(u8g, U8G_PI_RESET, arg_val);
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      if (isCommand)
        LCD_IO_WriteReg(arg_val);
      else
        LCD_IO_WriteData((uint16_t)arg_val);
      break;

    case U8G_COM_MSG_WRITE_SEQ:

      for (uint8_t i = 0; i < arg_val; i += 2)
        LCD_IO_WriteData(*(uint16_t *)(((uint32_t)arg_ptr) + i));
      break;
  }
  return 1;
}

#define  LCD_REG       (*((volatile unsigned short *) 0x60000000)) // RS = 0 Command
#define LCD_RAM       (*((volatile unsigned short *) 0x60020000)) // RS = 1 Data

static inline void Driver_LcdFSMCWriteReg(uint16_t Index)
{
      LCD_REG = Index;
}

static inline uint16_t Driver_LcdFSMCReadReg(void)
{
  return (LCD_REG);
}

static inline void Driver_LcdFSMCWriteData(uint16_t Data)
{
      LCD_RAM = Data;
}

static inline uint16_t Driver_LcdFSMCReadData(void)
{
  return (LCD_RAM);
}

static void LCD_WriteReg(uint16_t Reg, uint16_t RegValue) {
  Driver_LcdFSMCWriteReg(Reg);
  Driver_LcdFSMCWriteData(RegValue);
}

uint16_t LCD_ReadReg(uint16_t reg) {
  Driver_LcdFSMCWriteReg(reg);
  return Driver_LcdFSMCReadReg();
}

static void LCD_WriteData(uint16_t data) {
  Driver_LcdFSMCWriteData(data);
}

static uint16_t LCD_ReadData(void) {
  uint16_t tmp;
  Driver_LcdFSMCWriteReg(0x22);
  tmp = Driver_LcdFSMCReadData();
  tmp = Driver_LcdFSMCReadData();
  return tmp;
}

void LCD_Init(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(50);
  Driver_LcdFSMCWriteReg(0x11);  // Sleep out
  HAL_Delay(50);

  LCD_WriteReg(0xF0, 0xC3);
  LCD_WriteReg(0xF0, 0x96);
  LCD_WriteReg(0x36, 0x28);
  LCD_WriteReg(0x3A, 0x55);
  LCD_WriteReg(0xB4, 0x01);
  LCD_WriteReg(0xB7, 0xC6);
  LCD_WriteReg(0xC1, 0x15);
  LCD_WriteReg(0xC2, 0xAF);
  LCD_WriteReg(0xC3, 0x09);
  LCD_WriteReg(0xC5, 0x22);
  LCD_WriteReg(0xC6, 0x0);
  LCD_WriteReg(0xE8, 0x40);
  LCD_WriteReg(0x8A, 0x0);
  LCD_WriteData(0x0);
  LCD_WriteData(0x0);
  LCD_WriteData(0x29);
  LCD_WriteData(0x19);
  LCD_WriteData(0xA5);
  LCD_WriteData(0x33);
  LCD_WriteReg(0xE0, 0xF0);
  LCD_WriteData(0x04);
  LCD_WriteData(0x08);
  LCD_WriteData(0x09);
  LCD_WriteData(0x08);
  LCD_WriteData(0x15);
  LCD_WriteData(0x2F);
  LCD_WriteData(0x42);
  LCD_WriteData(0x46);
  LCD_WriteData(0x28);
  LCD_WriteData(0x15);
  LCD_WriteData(0x16);
  LCD_WriteData(0x29);
  LCD_WriteData(0x2D);
  LCD_WriteReg(0xE1, 0xF0);
  LCD_WriteData(0x04);
  LCD_WriteData(0x09);
  LCD_WriteData(0x09);
  LCD_WriteData(0x08);
  LCD_WriteData(0x15);
  LCD_WriteData(0x2E);
  LCD_WriteData(0x46);
  LCD_WriteData(0x46);
  LCD_WriteData(0x28);
  LCD_WriteData(0x15);
  LCD_WriteData(0x15);
  LCD_WriteData(0x29);
  LCD_WriteData(0x2D);
  LCD_WriteReg(0x2A, 0x0);
  LCD_WriteData(0x0);
  LCD_WriteData(0x01);
  LCD_WriteData(0x3F);
  LCD_WriteReg(0x2B, 0x0);
  LCD_WriteData(0x0);
  LCD_WriteData(0x01);
  LCD_WriteData(0xDF);
#if 1
  Driver_LcdFSMCWriteReg(0x20);  // Invert off
#else
  Driver_LcdFSMCWriteReg(0x21);
#endif
  ////
  LCD_WriteReg(0x53, 0x24);
  LCD_WriteReg(0xF0, 0x3C);
  LCD_WriteReg(0xF0, 0x69);
  HAL_Delay(150);
  Driver_LcdFSMCWriteReg(0x29);
  Driver_LcdFSMCWriteReg(0x2C);
}

void LCD_IO_Init(uint8_t cs, uint8_t rs) {
  HAL_FSMC_MspInit();
  LCD_Init();
  pinMode(PD3, OUTPUT);
  digitalWrite(PD3, HIGH);
}

void LCD_IO_WriteData(uint16_t RegValue) {
  LCD_RAM = RegValue;
	__DSB();
}

void LCD_IO_WriteReg(uint8_t Reg) {
  LCD_REG = Reg;
	__DSB();
}

uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize) {
	volatile uint32_t data;
	LCD_REG = (uint16_t)RegValue;
	__DSB();

	data = LCD_RAM; // dummy read
	data = LCD_RAM & 0x00FF;

	while (--ReadSize) {
		data <<= 8;
		data |= (LCD_RAM & 0x00FF);
	}
	return (uint32_t)data;
}

#endif
#endif
