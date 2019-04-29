/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../gcode.h"
#include "../../module/configuration_store.h"
#include "../../core/serial.h"
#include "../../inc/MarlinConfig.h"
#include "../../module/memoryserial.h"

#if ENABLED(WIFI_PRINTING)

extern void MALYANLCD_set_wifi_ssd(char *ssd);

extern void MALYANLCD_set_wifi_password(char *password);

void ShowOneMemorySerial(MemorySerial *ms) {
  char message[1024];
  sprintf(message, "TXBUFF: %08x", (int)(ms->transmit_buffer));
  SERIAL_ECHOLN(message);

  sprintf(message, "RXBUFF: %08x", (int)(ms->receive_buffer));
  SERIAL_ECHOLN(message);
  SERIAL_ECHOLNPAIR("Available:", ms->available());
}

void ShowPTS(PassthroughSerialPair *pts) {
  char message[1024];
  sprintf(message, "RB0: %08x", (int)(&(pts->buffer0)));
  SERIAL_ECHOLN(message);

  sprintf(message, "RB1: %08x", (int)(&(pts->buffer1)));
  SERIAL_ECHOLN(message);

  SERIAL_ECHOLN("SerialA");
  ShowOneMemorySerial(&pts->SerialA);

  SERIAL_ECHOLN("SerialB");
  ShowOneMemorySerial(&pts->SerialB);
}

/**
 * M550: Store settings in EEPROM
 */
void GcodeSuite::M550() {
  //ShowPTS(&PassThroughSerial);
  MALYANLCD_set_wifi_ssd(parser.string_arg);
}

/**
 * M551: Set WIFI SSID
 */
void GcodeSuite::M551() {
  MALYANLCD_set_wifi_password(parser.string_arg);
//  SERIAL_ECHOLNPAIR("Passing:", parser.string_arg);
//  for (int i = 0; i < strlen(parser.string_arg); i++)
//    MYSERIAL1.write(parser.string_arg[i]);
}

#endif
