/*******************************************************************************
 * Copyright (c) 2020, 2022  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include "CaBotHandle.h"

// #define DEBUG 1

namespace cabot
{
Handle::Handle() { mBaudRate = 0; }

Handle::~Handle() {}

void Handle::setBaudRate(unsigned long rate) { mBaudRate = rate; }

void Handle::init()
{
  mConnected = false;
  mConnecting = false;
  mTimeOffset = 0;
  diff = 0;
  state = 0;
  header_count = 0;
  size = 0;
  size_count = 0;
  cmd = 0;
  count = 0;

  if (mBaudRate == 0) {
    return;
  }
  Serial.begin(mBaudRate);
  while (!Serial) {
    ;
  }
  mConnected = true;
}

bool Handle::connected() { return mConnected; }

void Handle::spinOnce()
{
  static uint8_t cmd = 0;
  uint8_t *data;
  int count = readCommand(&cmd, &data);
  if (count == 0)
    return;

  if (cmd == 0x01) { // time sync
    uint32_t temp = (millis() - mSyncTime) / 2UL;
    diff = diff * 0.9f + temp * 0.1f;

#ifdef DEBUG
    char buff[32];
    snprintf(buff, sizeof(buff), "diff=");
    dtostrf(diff, 4, 2, buff + strlen(buff));
    logdebug(buff);
#endif

    mTimeOffset = mSyncTime + diff;
    mTime.sec = parseUInt32(data);
    mTime.nsec = parseUInt32(data + 4);
  }
  if (0x20 <= cmd && cmd <= 0x23) {
    for (int i = 0; i < 4; i++) {
      if (callbacks[i].cmd == cmd) {
        callbacks[i].callback(data[0]);
      }
    }
  }
  cmd = 0; // reset
}

void Handle::subscribe(uint8_t cmd, void (*callback)(const uint8_t))
{
  Callback temp;
  temp.cmd = cmd;
  temp.callback = callback;
  callbacks[callback_count++] = temp;
}

void Handle::logdebug(char *text)
{
  // code的にはこれで問題ないと思います。
  // sendCommandの引数でchar*で受けるものを作って、そちらでcastするのが良いと思います。
  sendCommand(0x02, text, strlen(text));
}

void Handle::loginfo(char *text) { sendCommand(0x03, text, strlen(text)); }

void Handle::logwarn(char *text) { sendCommand(0x04, text, strlen(text)); }

bool Handle::getParam(char *name, int *out, size_t num, int timeout_ms)
{
  sendCommand(0x08, name, strlen(name));
  uint8_t cmd = 0x08;
  uint8_t *ptr;
  int count = 0;
  while (readCommand(&cmd, &ptr) == 0) {
    count += 1;
    delay(1);
    if (count > timeout_ms) {
      // loginfo("timeout");
      return false;
    }
  }
  for (size_t i = 0; i < num; i++) {
    out[i] = parseUInt32(ptr + i * 4);
  }

  return true;
}

void Handle::publish(uint8_t cmd, char *data, size_t num) // prevent from no wrapping
{
  sendCommand(cmd, reinterpret_cast<uint8_t *>(data), num);
}

void Handle::publish(uint8_t cmd, uint8_t *data, size_t num) // prevent from no wrapping
{
  sendCommand(cmd, data, num);
}

void Handle::publish(uint8_t cmd, int8_t *data, size_t num)
{
  sendCommand(cmd, reinterpret_cast<uint8_t *>(data), num);
}

void Handle::publish(uint8_t cmd, float *data, size_t num)
{
  uint8_t temp[128];
  for (int i = 0; i < num; i++) {
    toBytes(data[i], temp + i * 4);
  }
  sendCommand(cmd, temp, num * 4);
}

void Handle::publish(uint8_t cmd, int8_t data)
{
  uint8_t buff[1];
  toBytes(data, buff, 1);
  sendCommand(cmd, buff, 1);
}

void Handle::publish(uint8_t cmd, int16_t data)
{
  uint8_t buff[2];
  toBytes(data, buff, 2);
  sendCommand(cmd, buff, 2);
}

void Handle::publish(uint8_t cmd, float data)
{
  uint8_t buff[4];
  toBytes(data, buff);
  sendCommand(cmd, buff, 4);
}

void Handle::sync()
{
  uint8_t buff[8];

  Time current = now();
  toBytes(current.sec, buff, 4);
  toBytes(current.nsec, buff + 4, 4);
  mSyncTime = millis();
  sendCommand(0x01, buff, 8);
}

bool Handle::is_synchronized() { return mTimeOffset > 0; }

Time Handle::now()
{
  Time current;
  uint32_t diff = (millis() - mTimeOffset);

  current.sec = mTime.sec + (diff / 1000UL);
  current.nsec = mTime.nsec + (diff % 1000UL) * 1000000UL;
  if (current.nsec > 1000000000UL) {
    current.nsec -= 1000000000UL;
    current.sec += 1;
  }
  return current;
}

#define SIZE_BYTES_LENGTH (1)

/* private */
size_t Handle::readCommand(uint8_t *expect, uint8_t **ptr)
{
  static uint8_t buffer[256];

  int received = Serial.read();
  if (received < 0)
    return 0;
#ifdef DEBUG
  static char buff[48];
  snprintf(buff, 48, "%d %d %x %x %d %d %d", state, header_count, cmd, *expect, size, size_count, count);
  loginfo(buff);
#endif
  if (state == 0) {
    if (received == 0xAA) {
      header_count += 1;
    } else {
      header_count = 0;
    }
    if (header_count == 2) {
      header_count = 0;
      state = 1;
    }
    cmd = 0;
  } else if (state == 1) {
    cmd = received;
    if (*expect == 0) {
      *expect = cmd;
    } else if (cmd != *expect) {
      state = 0;
      return 0;
    }
    state = 2;
    size = 0;
    size_count = 0;
  } else if (state == 2) {
    size += (received & 0xFF) << size_count * 8;
    size_count += 1;
    if (size_count == SIZE_BYTES_LENGTH) {
      if (size < 0 || 256 < size) {
        state = 0;
      } else if (size == 0) {
        state = 4;
      } else {
        state = 3;
      }
    }
    count = 0;
  } else if (state == 3) {
    buffer[count] = received;
    count += 1;
    if (count == size) {
      state = 4;
    }
  } else if (state == 4) {
    state = 0;
    if (received == checksum(buffer, size)) {
      *ptr = buffer;
      return size;
    }
  }
  return 0;
}

bool Handle::sendCommand(uint8_t type, uint8_t *data, size_t num)
{
  static uint8_t buffer[256 + 6];
  if (num < 0 || 256 < num) {
    return false;
  }

  buffer[0] = 0xAA;
  buffer[1] = 0xAA;
  buffer[2] = type;
  buffer[3] = num & 0xFF;
  buffer[4] = (num >> 8) & 0xFF;
  for (size_t i = 0; i < num; i++) {
    buffer[i + 5] = data[i];
  }

  buffer[num + 5] = checksum(data, num);

  size_t written = 0;
  while (written < num + 6) {
    if (Serial.availableForWrite() > 0) {
      written += Serial.write(buffer + written, num + 6 - written);
    }
  }
  return true;
}

bool Handle::sendCommand(uint8_t type, char *data, size_t num)
{
  return sendCommand(type, reinterpret_cast<uint8_t *>(data), num);
}

uint8_t Handle::checksum(uint8_t *data, size_t num)
{
  uint8_t temp = 0;
  for (size_t i = 0; i < num; i++) {
    temp += data[i];
  }
  return 0xFF - (0xFF & temp);
}

typedef struct {
  uint8_t c1;
  uint8_t c2;
  uint8_t c3;
  uint8_t c4;
} convert_t;

// bit shift over 16bit does not work
uint32_t Handle::parseUInt32(uint8_t *ptr)
{
  convert_t temp = {ptr[0], ptr[1], ptr[2], ptr[3]};
  return *reinterpret_cast<uint32_t *>(&temp);
}

void Handle::toBytes(uint32_t v, uint8_t *ptr, size_t num)
{
  for (int i = 0; i < num; i++) {
    ptr[i] = (v >> 8 * i) & 0xFF;
  }
}

void Handle::toBytes(float v, uint8_t *ptr)
{
  convert_t *temp = reinterpret_cast<convert_t *>(&v);
  ptr[0] = temp->c1;
  ptr[1] = temp->c2;
  ptr[2] = temp->c3;
  ptr[3] = temp->c4;
}

} // namespace cabot
