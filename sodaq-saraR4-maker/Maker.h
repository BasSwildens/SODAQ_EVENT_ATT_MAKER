/*
Copyright (c) 2018, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MAKER_H
#define MAKER_H

#include "Sodaq_LSM303AGR.h"
#include "ublox.h"
#include "Config.h"

#define CONSOLE_STREAM SerialUSB
#define DEBUG_STREAM   SerialUSB

// #define ENABLE_WATCHDOG

#define CONSOLE_BAUD   115200
#define DEBUG_BAUD     115200 // only used when CONSOLE is different than debug, otherwise console baud is used only
#define STARTUP_DELAY  5000

#define GPS_TIME_VALIDITY      0b00000011 // date and time (but not fully resolved)
#define GPS_FIX_FLAGS          0b00000001 // just gnssFixOK
#define GPS_COMM_CHECK_TIMEOUT 3 // seconds
#define GPS_SCALING            (float)pow(10, 7)
#define MAX_RTC_EPOCH_OFFSET   25

#define debugPrint(x)   if (params.getIsDebugOn()) { DEBUG_STREAM.print(x);   }
#define debugPrintln(x) if (params.getIsDebugOn()) { DEBUG_STREAM.println(x); }

#define consolePrint(x)   { CONSOLE_STREAM.print(x);   if ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM) { debugPrint(x);   } }
#define consolePrintln(x) { CONSOLE_STREAM.println(x); if ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM) { debugPrintln(x); } }

// macro to do compile time sanity checks / assertions
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

enum LogMessageType
{
    UnknownMessageType,
    NormalMessageType,
    ErrorMessageType,
    FullScreenMessageType,
};

struct Position
{
    uint32_t Timestamp;
    int32_t  Lat;
    int32_t  Lon;
    int16_t  Alt;
    uint8_t  NumberOfSatellites;
    uint8_t  TimeToFix;
} __attribute__((packed));

extern volatile bool pendingSensorRead;

extern Sodaq_LSM303AGR accelerometer;
extern bool            isGpsInitialized;
extern Position        tmpPosition;
extern void (*displayLine_func)(const char* message, LogMessageType messageType);

void fatal();
bool handleGpsFixSequence();
void postLoop();
void preLoop();
void displayLine(const char* message, LogMessageType messageType);
void setupBoot();
void setupFinal();
void setupInitial();
void uploadData(const uint8_t* buffer, uint8_t size, bool needHeader, bool needBlink);

#endif // MAKER_H
