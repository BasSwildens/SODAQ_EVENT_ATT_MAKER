/*
Copyright (c) 2016, SODAQ
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

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <Arduino.h>
#include "Command.h"

typedef void(*VoidCallbackMethodPtr)(void);

struct ConfigParams
{
    uint16_t _header;

    uint16_t _sensorReadInterval;
    uint16_t _gpsFixTimeout;

    uint8_t _isGpsEnabled;

    uint8_t _accelerationPercentage;
    uint8_t _accelerationDuration;
    uint8_t _accelerationSensitivity;
    uint8_t _onTheMoveBackoffTime;

    char _attToken[32 + 1];
    char _apn[32 + 1];
    char _forceOperator[32 + 1];

    char _endpointUrl[128 + 1];
    uint16_t _endpointPort;
    uint8_t _urat;
    uint8_t _telco;

    int8_t _temperatureOffset;

    uint8_t _gpsMinSatelliteCount;

    uint8_t _isDebugOn;

    uint16_t _crc16;

public:
    void read();
    bool commit();
    void reset();

    bool execCommand(const char* line);

    uint16_t getSensorReadInterval() const { return _sensorReadInterval; }
    uint16_t getGpsFixTimeout() const { return _gpsFixTimeout; }

    uint8_t getIsGpsEnabled() const { return _isGpsEnabled; }

    uint8_t getAccelerationPercentage() const { return _accelerationPercentage; }
    uint8_t getAccelerationDuration() const { return _accelerationDuration; }
    uint8_t getAccelerationSensitivity() const { return _accelerationSensitivity; }
    uint8_t getOnTheMoveBackoffTime() const { return _onTheMoveBackoffTime; }

    const char* getAttToken() const { return _attToken; }

    const char* getApn() const { return _apn; }
    const char* getForceOperator() const { return _forceOperator; }

    const char* getEndpointUrl() const { return _endpointUrl; }
    uint16_t getEndpointPort() const { return _endpointPort; }
    uint8_t getUrat() const { return _urat; }
    uint8_t getTelco() const { return _telco; }

    int8_t getTemperatureOffset() const { return _temperatureOffset; }

    uint8_t getGpsMinSatelliteCount() const{ return _gpsMinSatelliteCount; }

    uint8_t getIsDebugOn() const { return _isDebugOn; }

    static void showConfig(Stream* stream);
    bool checkConfig(Stream& stream);
    void setConfigResetCallback(VoidCallbackMethodPtr callback);
};

extern ConfigParams params;

#endif
