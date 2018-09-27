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

#include "Sodaq_nbIOT.h"
#include "Sodaq_wdt.h"
#include "Maker.h"
#include "Modem.h"

#define MODEM_STREAM       Serial1

#define DEFAULT_CID        1
#define MODEM_ENABLE       SARA_ENABLE
#define MODEM_TX_ENABLE    SARA_TX_ENABLE
#define MODEM_TOGGLE       SARA_R4XX_TOGGLE

#define ATCOMMANDS_COUNT   26
#define INITCOMMANDS_COUNT 8
#define MODEM_TIMEOUT      3 * 60 * 1000

Modem modem;

static Sodaq_nbIOT nbiot;

void Modem::initStream()
{
    MODEM_STREAM.begin(nbiot.getSaraR4Baudrate());
}

void Modem::initNBIOT()
{
    nbiot.init(MODEM_STREAM, MODEM_ENABLE, MODEM_TX_ENABLE, MODEM_TOGGLE, DEFAULT_CID);
}

bool Modem::initModem()
{
    nbiot.setDiag(DEBUG_STREAM);
    initNBIOT();
    nbiot.overrideNconfigParam("CR_0354_0338_SCRAMBLING", true);
    nbiot.setUrat(params.getUrat());

    bool result = connectNBIOT();

    if (!result) {
        Sodaq_nbIOT::SimStatuses simStatus = nbiot.getSimStatus();

        if (simStatus == Sodaq_nbIOT::SimMissing) {
            debugPrintln("Please insert the SIM Card and restart the device");
            displayLine("Please insert", ErrorMessageType);
            displayLine("the SIM Card",  ErrorMessageType);
            displayLine("and restart",   ErrorMessageType);
            displayLine("the device",    ErrorMessageType);
        }
        else if (simStatus == Sodaq_nbIOT::SimNeedsPin) {
            debugPrintln("Please configure the SIM pin");
            displayLine("Please configure", ErrorMessageType);
            displayLine("the SIM pin",      ErrorMessageType);
        }
        else {
            // the problem is not related to the SIM
            debugPrintln("Could not connect to the network");
            displayLine("Could not",   ErrorMessageType);
            displayLine("connect to",  ErrorMessageType);
            displayLine("the network", ErrorMessageType);
        }
    }

    return result;
}

void Modem::flush()
{
    MODEM_STREAM.flush();
}

void Modem::readIMEI()
{
    if (nbiot.getIMEI(imei, sizeof(imei))) {
        cachedImei = atoll(imei);
    } else {
        imei[0] = '\0';
        cachedImei = 0;
        consolePrintln("Failed to get IMEI!");
    }
}

void Modem::off()
{
    nbiot.off();
}

void Modem::on()
{
    nbiot.on();
}

uint8_t Modem::transmit(const uint8_t* buffer, uint8_t size)
{
    sodaq_wdt_reset();

    if (!nbiot.isConnected()) {
        if (!connectNBIOT()) {
            off();
            sodaq_wdt_safe_delay(450);
            on();
            sodaq_wdt_safe_delay(450);

            // try just one last time
            connectNBIOT();
        }
    }

    debugPrintln("Sending message through UDP...");
    int socketID = nbiot.createSocket();

    if (socketID < 0) {
        debugPrintln("Error: failed to create socket!");
        return false;
    }

    size_t lengthSent = nbiot.socketSend(socketID, params.getEndpointUrl(), params.getEndpointPort(), buffer, size);
    debugPrint("Sent ");
    debugPrint(lengthSent);
    debugPrintln(" bytes");

    nbiot.closeSocket(socketID);

    return lengthSent;
}

void Modem::executeAtInit()
{
    nbiot.setDiag(DEBUG_STREAM);

    consolePrintln();

    for (int i = 1; i <= INITCOMMANDS_COUNT; i++) {
        if (i == 6) {
            printInitializingMessage();
        }

        putInitCommand(CONSOLE_STREAM, i);
        putInitCommand(MODEM_STREAM  , i);

        printResponse();
    }

    consolePrintln("Finished...\r\n");

    sodaq_wdt_safe_delay(2000);
}

bool Modem::handleAtCommand(char *buffer)
{
    int i = atoi(buffer);

    if (i > 0 && i <= ATCOMMANDS_COUNT) {
        if (i == 14) {
            printInitializingMessage();
        }

        putAtCommand(CONSOLE_STREAM, i);
        putAtCommand(MODEM_STREAM,   i);

        printResponse();

        return false;
    }

    if (strncasecmp(buffer, "AT", 2) == 0) {
        if (strncasecmp(buffer, "AT+COPS=", 8) == 0) {
            printInitializingMessage();
        }

        MODEM_STREAM.println(buffer);

        printResponse();

        return false;
    }

    return true;
}

void Modem::showAtCommands()
{
    nbiot.setDiag(DEBUG_STREAM);

    consolePrintln("\r\nAT Command Mode\r\n");

    for (int i = 1; i <= ATCOMMANDS_COUNT; i++) {
        consolePrint(i);
        consolePrint(i <= 9 ? "   " : "  ");
        putAtCommand(CONSOLE_STREAM, i);
    }

    consolePrintln("\r\nYou can use one of the above numbers, or type 'exit' for quit AT Command Mode\r\n");
}

bool Modem::connectNBIOT()
{
    return nbiot.connect(params.getApn(), NULL, params.getForceOperator());
}

void Modem::printInitializingMessage()
{
    consolePrintln("Initializing module, this may take 3 minutes...");
}

void Modem::printResponse()
{
    nbiot.readResponse(NULL, MODEM_TIMEOUT);

    while (MODEM_STREAM.available()) {
        consolePrint(MODEM_STREAM.read());
    }
}

void Modem::putAtCommand(Stream& stream, uint16_t index)
{
    switch (index) {
        case 1:
            stream.println("ATE0");
            break;
        case 2:
            stream.print("AT+URAT=");
            stream.println(params.getUrat());
            break;
        case 3:
            stream.println("AT+CCID");
            break;
        case 4:
            stream.println("AT+CGSN");
            break;
        case 5:
            stream.println("AT+CIMI");
            break;
        case 6:
            stream.println("AT+CGMR");
            break;
        case 7:
            stream.println("AT+UBANDMASK?");
            break;
        case 8:
            stream.println("AT+UMNOPROF?");
            break;
        case 9:
            stream.println("AT+URAT?");
            break;
        case 10:
            stream.println("AT+CFUN=0");
            break;
        case 11:
            stream.print("AT+CGDCONT=1,\"IP\",\"");
            stream.print(params.getApn());
            stream.println('"');
            break;
        case 12:
            stream.println("AT+CFUN=1");
            break;
        case 13:
            stream.println("AT+CGDCONT?");
            break;
        case 14:
            stream.print("AT+COPS=1,2,\"");
            stream.print(params.getForceOperator());
            stream.println('"');
            break;
        case 15:
            stream.println("AT+CSQ");
            break;
        case 16:
            stream.println("AT+CREG=2");
            break;
        case 17:
            stream.println("AT+CREG?");
            break;
        case 18:
            stream.println("AT+CGPADDR");
            break;
        case 19:
            stream.println("AT+UDCONF=1,0");
            break;
        case 20:
            stream.println("AT+USOCR=17");
            break;
        case 21:
            stream.print("AT+USOST=0,\"");
            stream.print(params.getEndpointUrl());
            stream.print("\",");
            stream.print(params.getEndpointPort());
            stream.println(",4,\"Data\"");
            break;
        case 22:
            stream.println("AT+UDCONF=1,1");
            break;
        case 23:
            stream.print("AT+USOST=0,\"");
            stream.print(params.getEndpointUrl());
            stream.print("\",");
            stream.print(params.getEndpointPort());
            stream.println(",4,\"44617461\"");
            break;
        case 24:
            stream.println("AT+USOCL=0");
            break;
        case 25:
            stream.println("AT+URAT=7");
            break;
        case 26:
            stream.println("AT+CFUN=15");
            break;
    }
}

void Modem::putInitCommand(Stream& stream, uint16_t index)
{
    switch (index) {
        case 1:
            stream.println("ATE0");
            break;
        case 2:
            stream.print("AT+URAT=");
            stream.println(params.getUrat());
            break;
        case 3:
            stream.println("AT+CFUN=0");
            break;
        case 4:
            stream.print("AT+CGDCONT=1,\"IP\",\"");
            stream.print(params.getApn());
            stream.println('"');
            break;
        case 5:
            stream.println("AT+CFUN=1");
            break;
        case 6:
            stream.print("AT+COPS=1,2,\"");
            stream.print(params.getForceOperator());
            stream.println('"');
            break;
        case 7:
            stream.println("AT+CSQ");
            break;
        case 8:
            stream.println("AT+CFUN=15");
            break;
    }
}
