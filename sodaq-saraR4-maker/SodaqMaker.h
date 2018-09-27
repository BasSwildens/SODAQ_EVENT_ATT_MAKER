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

#ifndef SODAQ_MAKER_H
#define SODAQ_MAKER_H

#include "Maker.h"

#define DEFAULT_TEMPERATURE_OFFSET 0

#define TMOBILE_APN                "cdp.iot.t-mobile.nl"
#define TMOBILE_FORCE_OPERATOR     "20416"
#define TMOBILE_UDP_ENDPOINT_PORT  15683
#define TMOBILE_UDP_ENDPOINT_URL   "172.27.131.100"
#define TMOBILE_URAT               8

#define VFN_APN                    "nb.inetd.gdsp"
#define VFN_FORCE_OPERATOR         "20404"
#define VFN_UDP_ENDPOINT_PORT      8891
#define VFN_UDP_ENDPOINT_URL       "40.68.172.187"
#define VFN_URAT                   8

#define VFM_APN                    "live.vodafone.com"
#define VFM_FORCE_OPERATOR         "20404"
#define VFM_UDP_ENDPOINT_PORT      8891
#define VFM_UDP_ENDPOINT_URL       "40.68.172.187"
#define VFM_URAT                   7

#define KPN_APN                    "m2mc.webtrial"
#define KPN_FORCE_OPERATOR         "20408"
#define KPN_UDP_ENDPOINT_PORT      8891
#define KPN_UDP_ENDPOINT_URL       "40.68.172.187"
#define KPN_URAT                   7

#define MONO_APN                   "data.mono"
#define MONO_FORCE_OPERATOR        "20408"
#define MONO_UDP_ENDPOINT_PORT     8891
#define MONO_UDP_ENDPOINT_URL      "40.68.172.187"
#define MONO_URAT                  7

void onConfigReset();
bool setTelco(const Command *s, const char *line);
void resetConfigNetwork();

#endif // SODAQ_MAKER_H
