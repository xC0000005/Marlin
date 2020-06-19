/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if !defined(STM32F4) && !defined(STM32F4xx)
  #error "Oops! Select an STM32F4 board in 'Tools > Board.'"
#elif HOTENDS > 2 || E_STEPPERS > 2
  #error "LERDGE S supports up to 2 hotends / E-steppers."
#endif

#define BOARD_INFO_NAME      "Lerdge S"
#define DEFAULT_MACHINE_NAME "LERDGE"

#define STEP_TIMER 4
#define TEMP_TIMER 2

//#define I2C_EEPROM

//
// Servos
//
//#define SERVO0_PIN                        PD12 //unchecked, most likely wrong, board has a servo connector
//#define SERVO1_PIN                        -1

//
// Limit Switches
//
#define X_STOP_PIN                          PB12 //unchecked, most likely wrong
#define Y_STOP_PIN                          PB13 //unchecked, most likely wrong
#define Z_STOP_PIN                          PB14 //unchecked, most likely wrong

//
// Filament runout
//
#define FIL_RUNOUT_PIN                      PE1 //unchecked, most likely wrong

//
// Z Probe (when not Z_MIN_PIN)
//
//#ifndef Z_MIN_PROBE_PIN
//  #define Z_MIN_PROBE_PIN  PB15 //unchecked, most likely wrong, board has a probe switch connector
//#endif

//
// Steppers
//
#define X_STEP_PIN                          PF7 //confirmed 
#define X_DIR_PIN                           PF8 //confirmed 
#define X_ENABLE_PIN                        PF6 //confirmed 
//#ifndef X_CS_PIN
//  #define X_CS_PIN                        -1 //no idea
//#endif

#define Y_STEP_PIN                          PF10 //confirmed 
#define Y_DIR_PIN                           PF11 //confirmed 
#define Y_ENABLE_PIN                        PF9  //confirmed 
//#ifndef Y_CS_PIN
//  #define Y_CS_PIN                        -1 //no idea
//#endif

#define Z_STEP_PIN                          PF13 //confirmed 
#define Z_DIR_PIN                           PF14 //confirmed 
#define Z_ENABLE_PIN                        PF12 //confirmed 
//#ifndef Z_CS_PIN
//  #define Z_CS_PIN                        -1 //no idea
//#endif

#define E0_STEP_PIN                         PG0 //confirmed
#define E0_DIR_PIN                          PG1 //confirmed
#define E0_ENABLE_PIN                       PF15 //confirmed
//#ifndef E0_CS_PIN
//  #define E0_CS_PIN                       -1 //no idea
//#endif

#define E1_STEP_PIN                         PG3 //confirmed
#define E1_DIR_PIN                          PG4 //confirmed
#define E1_ENABLE_PIN                       PG2 //confirmed
//#ifndef E1_CS_PIN
//  #define E1_CS_PIN                       -1 //no idea
//#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC3   // Analog Input //guessing
#define TEMP_1_PIN                          PC4    // Analog Input //guessing
#define TEMP_BED_PIN                        PC5   // Analog Input //guessing

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA0 //confirmed
#define HEATER_1_PIN                        PA1 //confirmed
#define HEATER_BED_PIN                      PA3 //confirmed

#ifndef FAN_PIN
  //#define FAN_PIN                         PA15
#endif
#define FAN1_PIN                            PA15 //heater 0 fan 1 //confirmed
#define FAN2_PIN                            PB10 //heater 1 fan 2 //confirmed
#define FAN3_PIN                            PF5  //heater 0 fan 2 and heater 1 fan 2 (two sockets, switched together) //confirmed

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                   PF5  // FAN3_PIN
#endif

//
// Prusa i3 MK2 Multi Material Multiplexer Support
//
//#define E_MUX0_PIN                        -1
//#define E_MUX1_PIN                        -1

//
// LED / Lighting
//
//#define CASE_LIGHT_PIN_CI                 -1 //has two LED strip connectors (one RGB and one white only) - need to check those pins next
//#define CASE_LIGHT_PIN_DO                 -1
//#define NEOPIXEL_PIN                      -1

//
// Misc. Functions
//
#define SDSS                                PC11 //unchecked, will test sd access later
#define LED_PIN                             PC6   //confirmed
#define PS_ON_PIN                           -1    //board has a power monitor socket, usage unclear
#define KILL_PIN                            -1
#define POWER_LOSS_PIN                      -1    // Power-loss / nAC_FAULT

#define SCK_PIN                             PC12 //unchecked, will test sd access later
#define MISO_PIN                            PC8  //unchecked, will test sd access later
#define MOSI_PIN                            PD2 //unchecked, will test sd access later
#define SS_PIN                              PC11 //unchecked, will test sd access later

//
// SD support
//
//#define SDIO_SUPPORT
#define SD_DETECT_PIN                       -1 //unchecked, will test sd access later

//
// LCD / Controller
//

// The LCD is initialized in FSMC mode
#define BEEPER_PIN                          PD13 //confirmed

#define BTN_EN1                             PE3 //unchecked, most likely wrong
#define BTN_EN2                             PE4 //unchecked, most likely wrong
#define BTN_ENC                             PE2 //unchecked, most likely wrong

#define TFT_RESET_PIN                       PD6 //unchecked, most likely wrong
#define TFT_BACKLIGHT_PIN                   PD3  //confirmed (well, the LCD is off, but I cannot see if it is only the backlight)

#define TFT_CS_PIN                         PD7 //TFT works, so all ok?
#define TFT_RS_PIN                         PD11 //TFT works, so all ok?

#define TOUCH_CS_PIN                        PB6 //there is touch, but calibration is off
#define TOUCH_SCK_PIN                       PB3 //there is touch, but calibration is off
#define TOUCH_MOSI_PIN                      PB5 //there is touch, but calibration is off
#define TOUCH_MISO_PIN                      PB4 //there is touch, but calibration is off

//
// ST7920 Delays
//
#if HAS_GRAPHICAL_LCD
  #ifndef BOARD_ST7920_DELAY_1
    #define BOARD_ST7920_DELAY_1 DELAY_NS(96)
  #endif
  #ifndef BOARD_ST7920_DELAY_2
    #define BOARD_ST7920_DELAY_2 DELAY_NS(48)
  #endif
  #ifndef BOARD_ST7920_DELAY_3
    #define BOARD_ST7920_DELAY_3 DELAY_NS(715)
  #endif
#endif
