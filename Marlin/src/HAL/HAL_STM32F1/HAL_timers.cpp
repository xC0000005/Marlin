/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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

#ifdef STM32F1

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "HAL.h"

#include "HAL_timers.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 2

//#define PRESCALER 1
// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

tTimerConfig timerConfig[NUM_HARDWARE_TIMERS];

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

bool timers_initialised[NUM_HARDWARE_TIMERS] = {false};

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {

  if (!timers_initialised[timer_num]) {
    switch (timer_num) {
    case STEP_TIMER_NUM:
      // STEPPER TIMER
      ENABLE_STEP_TIMER();
      timerConfig[STEP_TIMER_NUM].timerdef.Instance            = STEP_TIMER_DEV;
      timerConfig[STEP_TIMER_NUM].timerdef.Init.Prescaler      = (STEPPER_TIMER_PRESCALE);
      timerConfig[STEP_TIMER_NUM].timerdef.Init.CounterMode    = TIM_COUNTERMODE_UP;
      timerConfig[STEP_TIMER_NUM].timerdef.Init.ClockDivision  = TIM_CLOCKDIVISION_DIV1;
      timerConfig[STEP_TIMER_NUM].IRQ_Id = STEP_TIMER_IRQ_NAME;
      timerConfig[STEP_TIMER_NUM].callback = (uint32_t)temp_isr_handler;
      HAL_NVIC_SetPriority(timerConfig[STEP_TIMER_NUM].IRQ_Id, STEP_TIMER_PRIORITY, 0);
      break;
    case TEMP_TIMER_NUM:
      // TEMPERATURE TIMER
      ENABLE_TEMP_TIMER();
      timerConfig[TEMP_TIMER_NUM].timerdef.Instance            = TEMP_TIMER_DEV;
      timerConfig[TEMP_TIMER_NUM].timerdef.Init.Prescaler      = (TEMP_TIMER_PRESCALE);
      timerConfig[TEMP_TIMER_NUM].timerdef.Init.CounterMode    = TIM_COUNTERMODE_UP;
      timerConfig[TEMP_TIMER_NUM].timerdef.Init.ClockDivision  = TIM_CLOCKDIVISION_DIV1;
      timerConfig[TEMP_TIMER_NUM].IRQ_Id = TEMP_TIMER_IRQ_NAME;
      timerConfig[TEMP_TIMER_NUM].callback = (uint32_t)step_isr_handler;
      HAL_NVIC_SetPriority(timerConfig[TEMP_TIMER_NUM].IRQ_Id, TEMP_TIMER_PRIORITY, 0);
      break;
    }
    timers_initialised[timer_num] = true;
  }

  timerConfig[timer_num].timerdef.Init.Period = (((HAL_TIMER_RATE) / timerConfig[timer_num].timerdef.Init.Prescaler) / frequency) - 1;

  if (HAL_TIM_Base_Init(&timerConfig[timer_num].timerdef) == HAL_OK)
    HAL_TIM_Base_Start_IT(&timerConfig[timer_num].timerdef);
}

//forward the interrupt
extern "C" void STEP_TIMER_CALLBACK() {
  ((void(*)(void))timerConfig[STEP_TIMER_NUM].callback)();
}
extern "C" void TEMP_TIMER_CALLBACK() {
  ((void(*)(void))timerConfig[TEMP_TIMER_NUM].callback)();
}

void HAL_timer_set_compare(const uint8_t timer_num, const uint32_t compare) {
  __HAL_TIM_SET_AUTORELOAD(&timerConfig[timer_num].timerdef, compare);
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  HAL_NVIC_EnableIRQ(timerConfig[timer_num].IRQ_Id);
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  HAL_NVIC_DisableIRQ(timerConfig[timer_num].IRQ_Id);
}

hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  return __HAL_TIM_GET_AUTORELOAD(&timerConfig[timer_num].timerdef);
}

uint32_t HAL_timer_get_count(const uint8_t timer_num) {
  return __HAL_TIM_GET_COUNTER(&timerConfig[timer_num].timerdef);
}

void HAL_timer_restrain(const uint8_t timer_num, const uint16_t interval_ticks) {
  const hal_timer_t mincmp = HAL_timer_get_count(timer_num) + interval_ticks;
  if (HAL_timer_get_compare(timer_num) < mincmp) HAL_timer_set_compare(timer_num, mincmp);
}

void HAL_timer_isr_prologue(const uint8_t timer_num) {
  if (__HAL_TIM_GET_FLAG(&timerConfig[timer_num].timerdef, TIM_FLAG_UPDATE) == SET) {
    __HAL_TIM_CLEAR_FLAG(&timerConfig[timer_num].timerdef, TIM_FLAG_UPDATE);
  }
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  if (NVIC->ISER[(uint32_t)((int32_t)timerConfig[timer_num].IRQ_Id) >> 5] & (uint32_t)(1 << ((uint32_t)((int32_t)timerConfig[timer_num].IRQ_Id) & (uint32_t)0x1F))) {
    return true;
  }
  else {
    return false;
  }
}

#endif // STM32F4
