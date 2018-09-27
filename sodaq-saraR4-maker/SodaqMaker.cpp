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

#include "SodaqMaker.h"

/**
* Callback from Config.reset(), used to override default values.
*/
void onConfigReset()
{
    resetConfigNetwork();

    params._temperatureOffset = DEFAULT_TEMPERATURE_OFFSET;
}

bool setTelco(const Command *s, const char *line)
{
    bool b = Command::set_uint8(s, line);
    if (b) {
        resetConfigNetwork();
    }
    return b;
}

void resetConfigNetwork()
{
    BUILD_BUG_ON(sizeof(TMOBILE_APN)              > sizeof(params._apn));
    BUILD_BUG_ON(sizeof(TMOBILE_FORCE_OPERATOR)   > sizeof(params._forceOperator));
    BUILD_BUG_ON(sizeof(TMOBILE_UDP_ENDPOINT_URL) > sizeof(params._endpointUrl));
    BUILD_BUG_ON(sizeof(VFN_APN)                  > sizeof(params._apn));
    BUILD_BUG_ON(sizeof(VFN_FORCE_OPERATOR)       > sizeof(params._forceOperator));
    BUILD_BUG_ON(sizeof(VFN_UDP_ENDPOINT_URL)     > sizeof(params._endpointUrl));
    BUILD_BUG_ON(sizeof(VFM_APN)                  > sizeof(params._apn));
    BUILD_BUG_ON(sizeof(VFM_FORCE_OPERATOR)       > sizeof(params._forceOperator));
    BUILD_BUG_ON(sizeof(VFM_UDP_ENDPOINT_URL)     > sizeof(params._endpointUrl));
    BUILD_BUG_ON(sizeof(KPN_APN)                  > sizeof(params._apn));
    BUILD_BUG_ON(sizeof(KPN_FORCE_OPERATOR)       > sizeof(params._forceOperator));
    BUILD_BUG_ON(sizeof(KPN_UDP_ENDPOINT_URL)     > sizeof(params._endpointUrl));
    BUILD_BUG_ON(sizeof(MONO_APN)                 > sizeof(params._apn));
    BUILD_BUG_ON(sizeof(MONO_FORCE_OPERATOR)      > sizeof(params._forceOperator));
    BUILD_BUG_ON(sizeof(MONO_UDP_ENDPOINT_URL)    > sizeof(params._endpointUrl));

    switch (params.getTelco()) {
        case 0:
            strcpy(params._apn,           TMOBILE_APN);
            strcpy(params._forceOperator, TMOBILE_FORCE_OPERATOR);
            strcpy(params._endpointUrl,   TMOBILE_UDP_ENDPOINT_URL);
            params._endpointPort        = TMOBILE_UDP_ENDPOINT_PORT;
            params._urat                = TMOBILE_URAT;
            break;
        case 1:
            strcpy(params._apn,           VFN_APN);
            strcpy(params._forceOperator, VFN_FORCE_OPERATOR);
            strcpy(params._endpointUrl,   VFN_UDP_ENDPOINT_URL);
            params._endpointPort        = VFN_UDP_ENDPOINT_PORT;
            params._urat                = VFN_URAT;
            break;
        case 2:
            strcpy(params._apn,           VFM_APN);
            strcpy(params._forceOperator, VFM_FORCE_OPERATOR);
            strcpy(params._endpointUrl,   VFM_UDP_ENDPOINT_URL);
            params._endpointPort        = VFM_UDP_ENDPOINT_PORT;
            params._urat                = VFM_URAT;
            break;
        case 3:
            strcpy(params._apn,           KPN_APN);
            strcpy(params._forceOperator, KPN_FORCE_OPERATOR);
            strcpy(params._endpointUrl,   KPN_UDP_ENDPOINT_URL);
            params._endpointPort        = KPN_UDP_ENDPOINT_PORT;
            params._urat                = KPN_URAT;
            break;
        case 4:
            strcpy(params._apn,           MONO_APN);
            strcpy(params._forceOperator, MONO_FORCE_OPERATOR);
            strcpy(params._endpointUrl,   MONO_UDP_ENDPOINT_URL);
            params._endpointPort        = MONO_UDP_ENDPOINT_PORT;
            params._urat                = MONO_URAT;
            break;
    }
}
