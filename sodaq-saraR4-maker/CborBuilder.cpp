/*    _   _ _ _____ _    _              _____     _ _     ___ ___  _  __
 *   /_\ | | |_   _| |_ (_)_ _  __ _ __|_   _|_ _| | |__ / __|   \| |/ /
 *  / _ \| | | | | | ' \| | ' \/ _` (_-< | |/ _` | | / / \__ \ |) | ' <
 * /_/ \_\_|_| |_| |_||_|_|_||_\__, /__/ |_|\__,_|_|_\_\ |___/___/|_|\_\
 *                             |___/
 *
 * Copyright 2018 AllThingsTalk
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *  This file has been modified to
 *   1) Remove any dependencies to ATT Device Class.
 *   2) Expose some methods needed by the app.
 *   3) Change addNumber to use templates.
 *   4) Add Timestamp and change GPS
 *
 */

#include "CborBuilder.h"
#include "Arduino.h"
#include <stdlib.h>

CborBuilder::CborBuilder()
{
  init(256);
}

CborBuilder::CborBuilder(const uint32_t initialCapacity)
{
  init(initialCapacity);
}

CborBuilder::~CborBuilder() {
  delete buffer;
}

void CborBuilder::init(unsigned int initialCapacity)
{
  this->capacity = initialCapacity;
  this->buffer = new unsigned char[initialCapacity];
  this->offset = 0;
}

void CborBuilder::reset()
{
  this->offset = 0;
}

void CborBuilder::addBoolean(bool value, const String asset)
{
  writeString(asset);
  if(value)
    writeSpecial(21);
  else
    writeSpecial(20);
}

void CborBuilder::addInteger(int value, const String asset)
{
  writeString(asset);
  writeInt(value);
}

void CborBuilder::addGps(double latitude, double longitude, int16_t altitude, const String asset)
{
  writeString(asset);
  writeTag(103);
  writeArray(3);
  addNumber(latitude);
  addNumber(longitude);
  writeInt(altitude);
}

void CborBuilder::addTimestamp(uint32_t timestamp)
{
    writeTag(1);
    writeInt(timestamp);
}

void CborBuilder::addString(const String value, const String asset)
{
  writeString(asset);
  writeString(value);
}

unsigned char *CborBuilder::getData()
{
  return buffer;
}

unsigned int CborBuilder::getSize()
{
  return offset;
}

void CborBuilder::putByte(unsigned char value)
{
  if(offset < capacity) {
    buffer[offset++] = value;
  } else {
    // Serial.print("buffer overflow error");
  }
}

void CborBuilder::putBytes(const unsigned char *data, const unsigned int size)
{
  if(offset + size - 1 < capacity) {
    memcpy(buffer + offset, data, size);
    offset += size;
  } else {
    // Serial.print("buffer overflow error");
  }
}

// Writer functions

void CborBuilder::writeTypeAndValue(uint8_t majorType, const uint32_t value)
{
  majorType <<= 5;
  if(value < 24) {
    putByte(majorType | value);
  } else if(value < 256) {
    putByte(majorType | 24);
    putByte(value);
  } else if(value < 65536) {
    putByte(majorType | 25);
    putByte(value >> 8);
    putByte(value);
  } else {
    putByte(majorType | 26);
    putByte(value >> 24);
    putByte(value >> 16);
    putByte(value >> 8);
    putByte(value);
  }
}

void CborBuilder::writeTypeAndValue(uint8_t majorType, const uint64_t value)
{
  majorType <<= 5;
  if(value < 24ULL) {
    putByte(majorType | value);
  } else if(value < 256ULL) {
    putByte(majorType | 24);
    putByte(value);
  } else if(value < 65536ULL) {
    putByte(majorType | 25);
    putByte(value >> 8);
  } else if(value < 4294967296ULL) {
    putByte(majorType | 26);
    putByte(value >> 24);
    putByte(value >> 16);
    putByte(value >> 8);
    putByte(value);
  } else {
    putByte(majorType | 27);
    putByte(value >> 56);
    putByte(value >> 48);
    putByte(value >> 40);
    putByte(value >> 32);
    putByte(value >> 24);
    putByte(value >> 16);
    putByte(value >> 8);
    putByte(value);
  }
}

void CborBuilder::writeInt(const int value)
{
  if(value < 0) {
    writeTypeAndValue(1, (uint32_t) -(value+1));
  } else {
    writeTypeAndValue(0, (uint32_t) value);
  }
}

void CborBuilder::writeBytes(const unsigned char *data, const unsigned int size)
{
  writeTypeAndValue(2, (uint32_t)size);
  putBytes(data, size);
}

void CborBuilder::writeString(const char *data, const unsigned int size)
{
  writeTypeAndValue(3, (uint32_t)size);
  putBytes((const unsigned char *)data, size);
}

void CborBuilder::writeString(const String str)
{
  writeTypeAndValue(3, (uint32_t)str.length());
  putBytes((const unsigned char *)str.c_str(), str.length());
}

void CborBuilder::writeArray(const unsigned int size)
{
  writeTypeAndValue(4, (uint32_t)size);
}

void CborBuilder::map(const unsigned int size)
{
  writeTypeAndValue(5, (uint32_t)size);
}

void CborBuilder::writeTag(const uint32_t tag)
{
  writeTypeAndValue(6, tag);
}

void CborBuilder::writeSpecial(const uint32_t special)
{
  writeTypeAndValue(7, special);
}