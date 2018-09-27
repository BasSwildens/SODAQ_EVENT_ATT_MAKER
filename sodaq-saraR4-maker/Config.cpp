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

#include "FlashStorage.h"
#include "Modem.h"
#include "SodaqMaker.h"

#define DEFAULT_HEADER 0xBEEF

ConfigParams params;

FlashStorage(flash, ConfigParams);
static bool needsCommit;
static VoidCallbackMethodPtr configResetCallback;

static uint16_t crc16ccitt(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (*buf++ << 8);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void ConfigParams::read()
{
    flash.read(this);

    // check header and CRC
    uint16_t calcCRC16 = crc16ccitt((uint8_t*)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);
    if (_header != DEFAULT_HEADER || _crc16 != calcCRC16) {
        reset();
    }
}

void ConfigParams::reset()
{
    _gpsFixTimeout           = 120;
    _sensorReadInterval      = 5;

    _attToken[0]             = '\0';
    _apn[0]                  = '\0';
    _forceOperator[0]        = '\0';
    _endpointUrl[0]          = '\0';
    _endpointPort            = 0;
    _urat                    = 8;
    _telco                   = 1;

    _temperatureOffset       = 0;

    _gpsMinSatelliteCount    = 6;
    _isGpsEnabled            = 1;

    _accelerationPercentage  = 25;
    _accelerationDuration    = 0;
    _accelerationSensitivity = 10;
    _onTheMoveBackoffTime    = 10; // seconds

    _isDebugOn               = 1;

    if (configResetCallback) {
        configResetCallback();
    }

    needsCommit = false;
}

/*
 * Write the configuration parameters to NVM / Dataflash
 */
bool ConfigParams::commit()
{
    if (!needsCommit) {
        return false;
    }

    _header = DEFAULT_HEADER;
    _crc16 = crc16ccitt((uint8_t*)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);

    flash.write(*this);

    needsCommit = false;

    return true;
}

static const Command args[] = {
    { "IMEI                                            ", 0,      0,                    Command::show_string, &modem.imei },
    { "Telco (tmobile=0, vf-n=1, vf-m=2, kpn=3, mono=4)", "tlc=", setTelco, Command::show_uint8,  &params._telco },
    { "All Things Talk Token                           ", "att=", Command::set_string,  Command::show_string, params._attToken, sizeof(params._attToken) },
    { "Upload Interval (min)                           ", "sri=", Command::set_uint16,  Command::show_uint16, &params._sensorReadInterval },
    { "GPS Fix Timeout (sec)                           ", "gft=", Command::set_uint16,  Command::show_uint16, &params._gpsFixTimeout },
    { "Minimum sat count                               ", "sat=", Command::set_uint8,   Command::show_uint8,  &params._gpsMinSatelliteCount },
    { "APN                                             ", "apn=", Command::set_string,  Command::show_string, params._apn, sizeof(params._apn) },
    { "Force Operator                                  ", "opr=", Command::set_string,  Command::show_string, params._forceOperator, sizeof(params._forceOperator) },
    { "Network (LTE-M=7, NB-Iot=8, 2G=9)               ", "rat=", Command::set_uint8,   Command::show_uint8,  &params._urat },
    { "Server URL/IP                                   ", "url=", Command::set_string,  Command::show_string, params._endpointUrl, sizeof(params._endpointUrl) },
    { "Server port                                     ", "prt=", Command::set_uint16,  Command::show_uint16, &params._endpointPort },
    { "Temperature offset                              ", "tof=", Command::set_int8,    Command::show_int8,   &params._temperatureOffset },
    { "GPS (OFF=0 / ON=1)                              ", "gps=", Command::set_uint8,   Command::show_uint8,  &params._isGpsEnabled },
    { "Debug (OFF=0 / ON=1)                            ", "dbg=", Command::set_uint8,   Command::show_uint8,  &params._isDebugOn },
    { "On-the-move Functionality                       ", 0,      0,                    Command::show_title,  0 },
    { "Acceleration% (100% = 8g)                       ", "acc=", Command::set_uint8,   Command::show_uint8,  &params._accelerationPercentage },
    { "Acceleration Duration                           ", "acd=", Command::set_uint8,   Command::show_uint8,  &params._accelerationDuration },
    { "Accelerator Trigger",                              0,      0,                    Command::show_text,   0 },
    { "  Sensitivity (1-50)                            ", "act=", Command::set_uint8,   Command::show_uint8,  &params._accelerationSensitivity },
    { "  Back-off time (sec)                           ", "bot=", Command::set_uint8,   Command::show_uint8,  &params._onTheMoveBackoffTime }
};

void ConfigParams::showConfig(Stream* stream)
{
    stream->println();
    stream->println("Settings:");
    for (size_t i = 0; i < sizeof(args) / sizeof(args[0]); ++i) {
        const Command* a = &args[i];
        if (a->show_func) {
            a->show_func(a, stream);
        }
    }
}

/*
 * Execute a command from the commandline
 *
 * Return true if it was a valid command
 */
bool ConfigParams::execCommand(const char* line)
{
    bool done = Command::execCommand(args, sizeof(args) / sizeof(args[0]), line);
    if (done) {
        needsCommit = true;
    }

    return done;
}

/*
 * Check if all required config parameters are filled in
 */
bool ConfigParams::checkConfig(Stream& stream)
{
    bool fail = false;

    if (_telco != 0 && (_attToken == NULL || _attToken[0] == '\0')) {
        stream.println("All Things Talk Token must not be empty");
        fail = true;
    }

    if (_isDebugOn > 1) {
        stream.println("Debug must be either 0 or 1");
        fail = true;
    }

    if (_isGpsEnabled > 1) {
        stream.println("GPS ON/OFF must be either 0 or 1");
        fail = true;
    }

    if (_accelerationPercentage > 100) {
        stream.println("Acceleration% must be 0-100");
        fail = true;
    }

    if (_accelerationSensitivity < 1 || _accelerationSensitivity > 50) {
        stream.println("Acceleration Trigger Sensitivity must be 1-50");
        fail = true;
    }

    if (_urat < 7 || _urat > 9) {
        stream.println("Network code must be 7, 8 or 9");
        fail = true;
    }

    if (_telco > 4) {
        stream.println("Telco must be 0-4");
        fail = true;
    }

    return !fail;
}

void ConfigParams::setConfigResetCallback(VoidCallbackMethodPtr callback)
{
    configResetCallback = callback;
}
