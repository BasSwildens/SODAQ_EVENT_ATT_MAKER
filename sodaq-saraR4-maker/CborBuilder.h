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
 
#ifndef CBOREN_H
#define CBOREN_H

#include "Arduino.h"

class CborBuilder {

  public:
    CborBuilder();
    CborBuilder(uint32_t initialCapacity);
    ~CborBuilder();

    unsigned char *getData();
    unsigned int getSize();

    void reset();

    // Method to construct a custom cbor payload
    void addBoolean(bool value, const String asset);
    void addInteger(int value, const String asset);
    
    template<typename T> void addNumber(T value, const String asset) {
        writeString(asset);
        addNumber(value);
    }

    void addString(const String value, const String asset);
    void addGps(double latitude, double longitude, int16_t altitude, const String asset);
    void addTimestamp(uint32_t timestamp);

    void map(const unsigned int size);
    void writeArray(const unsigned int size);
    void writeTag(const uint32_t tag);

    void init(unsigned int initalCapacity);
    unsigned char *buffer;
    unsigned int capacity;
    unsigned int offset;
 
  private: 
    virtual void putByte(unsigned char value);
    virtual void putBytes(const unsigned char *data, const unsigned int size);
  
    void writeTypeAndValue(uint8_t majorType, const uint32_t value);
    void writeTypeAndValue(uint8_t majorType, const uint64_t value);

    template<typename T> void addNumber(T value) {
        const int size = sizeof(value);

        // Convert double to bytes array
        union {
            T a;
            unsigned char bytes[size];
        } thing;

        if (size == 4)
            putByte(0xFA);
        else if (size == 8)
            putByte(0xFB);
        else
            return;  // Unknown float size

        thing.a = value;
        int ii;
        for (ii = (size - 1); ii >= 0; ii--)
        {
            putByte(thing.bytes[ii]);
        }
    }

    void writeInt(const int value);
    void writeBytes(const unsigned char *data, const unsigned int size);
    void writeString(const char *data, const unsigned int size);
    void writeString(const String str);
    void writeSpecial(const uint32_t special);
};

#endif