/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
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

#ifndef _HAL_TIMERS_H
#define _HAL_TIMERS_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define FORCE_INLINE __attribute__((always_inline)) inline

#define hal_timer_t uint32_t  // TODO: One is 16-bit, one 32-bit - does this need to be checked?
#define HAL_TIMER_TYPE_MAX 0xFFFF

#define STEP_TIMER_NUM 0  // index of timer to use for stepper
#define TEMP_TIMER_NUM 1  // index of timer to use for temperature

// TODO: Set all of these via calculation - the prescale seems to be the Clock Frequence divded by 1000.

#ifdef STM32F0
#define HAL_TIMER_RATE         (HAL_RCC_GetSysClockFreq() / 2)  // frequency of timer peripherals
#define STEPPER_TIMER_PRESCALE 24            // to match system clock

#define STEP_TIMER_DEV TIM4
#define STEP_TIMER_CALLBACK TIM45_IRQHandler
#define STEP_TIMER_IRQ_NAME TIM4_IRQn
#define ENABLE_STEP_TIMER() __HAL_RCC_TIM4_CLK_ENABLE()

#define TEMP_TIMER_DEV TIM5
#define TEMP_TIMER_CALLBACK TIM5_IRQHandler
#define TEMP_TIMER_IRQ_NAME TIM5_IRQn
#define ENABLE_TEMP_TIMER() __HAL_RCC_TIM5_CLK_ENABLE()

#endif

#ifdef STM32F1

#define HAL_TIMER_RATE         (HAL_RCC_GetPCLK2Freq())  // frequency of timer peripherals
#define STEPPER_TIMER_PRESCALE 36            // prescaler for setting stepper timer, 2Mhz. I

#define STEP_TIMER_DEV TIM4
#define STEP_TIMER_CALLBACK TIM4_IRQHandler
#define STEP_TIMER_IRQ_NAME TIM4_IRQn
#define ENABLE_STEP_TIMER() __HAL_RCC_TIM4_CLK_ENABLE()

#define TEMP_TIMER_DEV TIM3
#define TEMP_TIMER_CALLBACK TIM3_IRQHandler
#define TEMP_TIMER_IRQ_NAME TIM3_IRQn
#define ENABLE_TEMP_TIMER() __HAL_RCC_TIM3_CLK_ENABLE()
#endif

#ifdef STM32F4 || #defined STM32F7
#define HAL_TIMER_RATE         (HAL_RCC_GetSysClockFreq() / 2)  // frequency of timer peripherals
#define STEPPER_TIMER_PRESCALE 54            // was 40,prescaler for setting stepper timer, 2Mhz

#define STEP_TIMER_DEV TIM5
#define STEP_TIMER_CALLBACK TIM5_IRQHandler
#define STEP_TIMER_IRQ_NAME TIM5_IRQn
#define ENABLE_STEP_TIMER() __HAL_RCC_TIM5_CLK_ENABLE()

#define TEMP_TIMER_DEV TIM7
#define TEMP_TIMER_CALLBACK TIM7_IRQHandler
#define TEMP_TIMER_IRQ_NAME TIM7_IRQn
#define ENABLE_TEMP_TIMER() __HAL_RCC_TIM3_CLK_ENABLE()

#endif

#define STEP_TIMER_PRIORITY 1
#define TEMP_TIMER_PRIORITY 2

#define HAL_STEPPER_TIMER_RATE (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)   // frequency of stepper timer (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)
#define HAL_TICKS_PER_US       ((HAL_STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs

#define PULSE_TIMER_NUM STEP_TIMER_NUM
#define PULSE_TIMER_PRESCALE STEPPER_TIMER_PRESCALE

#define TEMP_TIMER_PRESCALE     1000 // prescaler for setting Temp timer, 72Khz
#define TEMP_TIMER_FREQUENCY    1000 // temperature interrupt frequency


#define STEP_TIMER_MIN_INTERVAL    8 // minimum time in µs between stepper interrupts
#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

// The way this works is that these define the function names for the ISR routines themselves.
// They are stored in an array and called from timer specific callbacks implemented by
// the core itself (weak symbols in the vector table)

extern void temp_isr_handler();
extern void step_isr_handler();
#define HAL_STEP_TIMER_ISR  void temp_isr_handler()
#define HAL_TEMP_TIMER_ISR  void step_isr_handler()

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef struct {
  TIM_HandleTypeDef timerdef;
  IRQn_Type   IRQ_Id;
  uint32_t callback;
} tTimerConfig;

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

//extern const tTimerConfig timerConfig[];

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

void HAL_timer_set_compare(const uint8_t timer_num, const uint32_t compare);
hal_timer_t HAL_timer_get_compare(const uint8_t timer_num);
uint32_t HAL_timer_get_count(const uint8_t timer_num);
void HAL_timer_restrain(const uint8_t timer_num, const uint16_t interval_ticks);

void HAL_timer_isr_prologue(const uint8_t timer_num);
#define HAL_timer_isr_epilogue(TIMER_NUM)

#endif // _HAL_TIMERS
