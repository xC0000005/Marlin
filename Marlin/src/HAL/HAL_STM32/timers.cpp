/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "HAL.h"

#include "timers.h"

// ------------------------
// Local defines
// ------------------------
#define AUTO_TIMER_SELECT

#define NUM_HARDWARE_TIMERS 2
#if ENABLED(AUTO_TIMER_SELECT)
TIM_TypeDef* STEP_TIMER_DEV;
TIM_TypeDef* TEMP_TIMER_DEV;
#else
#define __TIMER_DEV(X) TIM##X
#define _TIMER_DEV(X) __TIMER_DEV(X)
#define STEP_TIMER_DEV _TIMER_DEV(STEP_TIMER)
#define TEMP_TIMER_DEV _TIMER_DEV(TEMP_TIMER)
#endif

// ------------------------
// Private Variables
// ------------------------

HardwareTimer *timer_instance[NUM_HARDWARE_TIMERS] = { NULL };
bool timer_enabled[NUM_HARDWARE_TIMERS] = { false };
bool timer_available[TIMER_NUM] = { false };
int selected_step_timer = -1;
int selected_temp_timer = -1;
// ------------------------
// Public functions
// ------------------------

// frequency is in Hertz
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  if (!HAL_timer_initialized(timer_num)) {
    switch (timer_num) {
      case STEP_TIMER_NUM: // STEPPER TIMER - use a 32bit timer if possible
        timer_instance[timer_num] = new HardwareTimer(STEP_TIMER_DEV);
        /* Set the prescaler to the final desired value.
         * This will change the effective ISR callback frequency but when
         * HAL_timer_start(timer_num=0) is called in the core for the first time
         * the real frequency isn't important as long as, after boot, the ISR
         * gets called with the correct prescaler and count register. So here
         * we set the prescaler to the correct, final value and ignore the frequency
         * asked. We will call back the ISR in 1 second to start at full speed.
         *
         * The proper fix, however, would be a correct initialization OR a
         * HAL_timer_change(const uint8_t timer_num, const uint32_t frequency)
         * which changes the prescaler when an IRQ frequency change is needed
         * (for example when steppers are turned on)
         */
        timer_instance[timer_num]->setPrescaleFactor(STEPPER_TIMER_PRESCALE); //the -1 is done internally
        timer_instance[timer_num]->setOverflow(_MIN(hal_timer_t(HAL_TIMER_TYPE_MAX), (HAL_TIMER_RATE) / (STEPPER_TIMER_PRESCALE) /* /frequency */), TICK_FORMAT);
        break;
      case TEMP_TIMER_NUM: // TEMP TIMER - any available 16bit timer
        timer_instance[timer_num] = new HardwareTimer(TEMP_TIMER_DEV);
        // The prescale factor is computed automatically for HERTZ_FORMAT
        timer_instance[timer_num]->setOverflow(frequency, HERTZ_FORMAT);
        break;
    }

    HAL_timer_enable_interrupt(timer_num);

    /*
     * Initializes (and unfortunately starts) the timer.
     * This is needed to set correct IRQ priority at the moment but causes
     * no harm since every call to HAL_timer_start() is actually followed by
     * a call to HAL_timer_enable_interrupt() which means that there isn't
     * a case in which you want the timer to run without a callback.
     */
    timer_instance[timer_num]->resume(); // First call to resume() MUST follow the attachInterrupt()

    // This is fixed in Arduino_Core_STM32 1.8.
    // These calls can be removed and replaced with
    // timer_instance[timer_num]->setInterruptPriority
    switch (timer_num) {
      case STEP_TIMER_NUM:
        HAL_NVIC_SetPriority(STEP_TIMER_IRQ_NAME, STEP_TIMER_IRQ_PRIO, 0);
        break;
      case TEMP_TIMER_NUM:
        HAL_NVIC_SetPriority(TEMP_TIMER_IRQ_NAME, TEMP_TIMER_IRQ_PRIO, 0);
        break;
    }
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num) && !timer_enabled[timer_num]) {
    timer_enabled[timer_num] = true;
    switch (timer_num) {
      case STEP_TIMER_NUM:
      timer_instance[timer_num]->attachInterrupt(Step_Handler);
      break;
    case TEMP_TIMER_NUM:
      timer_instance[timer_num]->attachInterrupt(Temp_Handler);
      break;
    }
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_interrupt_enabled(timer_num)) {
    timer_instance[timer_num]->detachInterrupt();
    timer_enabled[timer_num] = false;
  }
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) && timer_enabled[timer_num];
}

void mark_timer_unavailable(pin_t pin) {
  PinName name = digitalPinToPinName(pin);
  TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(name, PinMap_PWM);
  uint32_t index = get_timer_index(instance);
  timer_available[index] = false;
}

TIM_TypeDef* get_timer_by_index(timer_index_t index)
{
  switch (index) {
#if defined(TIM1_BASE)
  case TIMER1_INDEX: return TIM1;
#endif
#if defined(TIM2_BASE)
  case TIMER2_INDEX: return TIM2;
#endif
#if defined(TIM3_BASE)
  case TIMER3_INDEX: return TIM3;
#endif
#if defined(TIM4_BASE)
  case TIMER4_INDEX: return TIM4;
#endif
#if defined(TIM5_BASE)
  case TIMER5_INDEX: return TIM5;
#endif
#if defined(TIM6_BASE)
  case TIMER6_INDEX: return TIM6;
#endif
#if defined(TIM7_BASE)
  case TIMER7_INDEX: return TIM7;
#endif
#if defined(TIM8_BASE)
  case TIMER8_INDEX: return TIM8;
#endif
#if defined(TIM9_BASE)
  case TIMER9_INDEX: return TIM9;
#endif
#if defined(TIM10_BASE)
  case TIMER10_INDEX: return TIM10;
#endif
#if defined(TIM11_BASE)
  case TIMER11_INDEX: return TIM11;
#endif
#if defined(TIM12_BASE)
  case TIMER12_INDEX: return TIM12;
#endif
#if defined(TIM13_BASE)
  case TIMER13_INDEX: return TIM13;
#endif
#if defined(TIM14_BASE)
  case TIMER14_INDEX: return TIM14;
#endif
#if defined(TIM15_BASE)
  case TIMER15_INDEX: return TIM15;
#endif
#if defined(TIM16_BASE)
  case TIMER16_INDEX: return TIM16;
#endif
#if defined(TIM17_BASE)
  case TIMER17_INDEX: return TIM17;
#endif
#if defined(TIM18_BASE)
  case TIMER18_INDEX: return TIM18;
#endif
#if defined(TIM19_BASE)
  case TIMER19_INDEX: return TIM19;
#endif
#if defined(TIM20_BASE)
  case TIMER20_INDEX: return TIM20;
#endif
#if defined(TIM21_BASE)
  case TIMER21_INDEX: return TIM21;
#endif
#if defined(TIM22_BASE)
  case TIMER2_INDEX: return TIM22;
#endif
  }

  // call error handler
  return TIM1;
}

// Determine which timers are available.
void select_timers() {
  // For each PWM pin (bed, heaters, fans if not soft PWM), mark the timers as unavailable
  // then select one for temp and stepper
  for (int j = 0; j < TIMER_NUM; j++) timer_available[j] = true;

#if HAS_HEATER_0 
  mark_timer_unavailable(HEATER_0_PIN);
#endif

#if HAS_HEATER_1
  mark_timer_unavailable(HEATER_1_PIN);
#endif

#if HAS_HEATER_2 
  mark_timer_unavailable(HEATER_2_PIN);
#endif

#if HAS_HEATER_3
  mark_timer_unavailable(HEATER_3_PIN);
#endif

#if HAS_HEATER_4
  mark_timer_unavailable(HEATER_4_PIN);
#endif

#if HAS_HEATER_5
  mark_timer_unavailable(HEATER_5_PIN);
#endif

#if HAS_HEATER_BED
  mark_timer_unavailable(HEATER_BED_PIN);
#endif

#if ENABLED(FAN_SOFT_PWM)
  #if HAS_FAN0
    mark_timer_unavailable(FAN0_PIN);
  #endif

  #if HAS_FAN1
    mark_timer_unavailable(FAN1_PIN);
  #endif

  #if HAS_FAN2
    mark_timer_unavailable(FAN2_PIN);
  #endif
#endif

  // Now select the first available for temperature
  for (int i = 0; i < TIMER_NUM; i++) {
    if (!timer_available[i]) continue;

    if (selected_step_timer == -1) {
      selected_step_timer = i;
      timer_available[i] = false;
      continue;
    }

    if (selected_temp_timer == -1) {
      selected_temp_timer = i;
      timer_available[i] = false;
      continue;
    }

    STEP_TIMER_DEV = get_timer_by_index((timer_index_t)selected_step_timer);
    TEMP_TIMER_DEV = get_timer_by_index((timer_index_t)selected_temp_timer);

    // Validate we were able to find timers
    if (selected_temp_timer == -1 || selected_step_timer == -1) {
      //kill(PSTR("TIMER ERROR"));
    }
  }
}

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
