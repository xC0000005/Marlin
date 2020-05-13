/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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

#include "../../inc/MarlinConfig.h"

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../gcode.h"

extern int selected_step_timer;
extern int selected_temp_timer;
extern IRQn_Type STEP_TIMER_IRQ_NAME;
extern IRQn_Type TEMP_TIMER_IRQ_NAME;

/**
 * M575 - Change serial baud rate
 *
 *   P<index>    - Serial port index. Omit for all.
 *   B<baudrate> - Baud rate (bits per second)
 */
void GcodeSuite::M576() {
      SERIAL_ECHO_START();
      SERIAL_ECHOLNPAIR("Selected Temp Timer:", selected_temp_timer);
      SERIAL_ECHOLNPAIR("Temp Timer IRQn:", TEMP_TIMER_IRQ_NAME);
      SERIAL_ECHOLNPAIR("Selected Step Timer:", selected_step_timer);
      SERIAL_ECHOLNPAIR("Step Timer IRQn:", STEP_TIMER_IRQ_NAME);

      SERIAL_FLUSH();
}

#endif // (ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)
