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
#pragma once

#include <stdarg.h>
#include <stdio.h>
#include "Stream.h"

/**
 * Generic RingBuffer
 * T type of the buffer array
 * S size of the buffer (must be power of 2)
 *
 */
template <typename T, uint32_t S> class RingBuffer {
public:
  RingBuffer() { index_read = index_write = 0; }
  uint32_t available() volatile {
    uint32_t result = index_write-index_read;
    if (result < 0) result += buffer_size;
    return result;
  }
  uint32_t free() volatile      {
    return buffer_size - available();
  }

  bool empty() volatile         { return index_read == index_write; }
  bool full() volatile          { return available() == buffer_size; }
  void clear() volatile         { index_read = index_write = 0; }

  bool peek(T *value) volatile {
    if (value == 0 || available() == 0)
      return false;
    *value = buffer[mask(index_read)];
    return true;
  }

  int read() volatile {
    int value = -1;
    if (empty()) return value;
    value = buffer[mask(index_read++)];
    index_read %= buffer_size;
    return value;
  }

  bool write(T value) volatile {
    if (full()) return false;
    buffer[mask(index_write++)] = value;
    index_write %= buffer_size;
    return true;
  }

private:
  uint32_t mask(uint32_t val) volatile {
    return buffer_mask & val;
  }

  static const uint32_t buffer_size = S;
  static const uint32_t buffer_mask = buffer_size - 1;
  volatile T buffer[buffer_size];
  volatile uint32_t index_write;
  volatile uint32_t index_read;
};

class MemorySerial : public Stream  {
public:

  MemorySerial() { host_connected = true; }

  void begin(int32_t baud) {
  }

  int available() {
    return (uint16_t)receive_buffer->available();
  }

  int peek() {
    uint8_t value;
    return receive_buffer->peek(&value) ? value : -1;
  }

  int read() { return receive_buffer->read(); }

  int availableForWrite(void){
    return transmit_buffer->free() > 255 ? 255 : (uint8_t)transmit_buffer->free();
  }

  void flush() { /*receive_buffer->clear();*/ }

  size_t write(uint8_t c) {
//    if (!host_connected) return 0;
//    while (!transmit_buffer.free());
    return transmit_buffer->write(c);
  }

  operator bool() { return host_connected; }


  void flushTX(void){
//    if (host_connected)
//      while (transmit_buffer.available()) { /* nada */ }
  }

  using Print::write; // pull in write(str) and write(buf, size) from Print

  void set_receive_buffer(RingBuffer<uint8_t, 128> *recv_buffer) {
    this->receive_buffer = recv_buffer;
  }

  void set_transmit_buffer(RingBuffer<uint8_t, 128> *trans_buffer) {
    this->transmit_buffer = trans_buffer;
  }

  RingBuffer<uint8_t, 128> *receive_buffer;
  RingBuffer<uint8_t, 128> *transmit_buffer;
  bool host_connected;
};

class PassthroughSerialPair {
public:
  RingBuffer<uint8_t, 128> buffer0;
  RingBuffer<uint8_t, 128> buffer1;

  MemorySerial SerialA;
  MemorySerial SerialB;

  PassthroughSerialPair() {
    this->SerialA.set_transmit_buffer(&this->buffer0);
    this->SerialB.set_receive_buffer(&this->buffer0);

    this->SerialB.set_transmit_buffer(&this->buffer1);
    this->SerialA.set_receive_buffer(&this->buffer1);
  }
};

extern PassthroughSerialPair PassThroughSerial;
