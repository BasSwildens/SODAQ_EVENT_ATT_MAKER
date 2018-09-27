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

#include <Wire.h>
#include "BootMenu.h"
#include "LedColor.h"
#include "MyTime.h"
#include "RTCTimer.h"
#include "RTCZero.h"
#include "Sodaq_wdt.h"
#include "Maker.h"
#include "Modem.h"

union UdpHeader
{
    uint8_t Raw[16];

    struct
    {
        uint8_t Version;
        uint8_t Imei[7];
        char    Token[8];
    } __attribute__((packed));
};

volatile bool     isTmpPositionAltered; // this is set to true only when tmpPosition is written by the gps delegate
volatile uint32_t lastOnTheMoveActivationTimestamp;
volatile bool     minuteFlag;
volatile bool     pendingSensorRead = true;
volatile bool     updateOnTheMoveTimestampFlag;

Sodaq_LSM303AGR   accelerometer;
bool              isGpsInitialized;
Position          tmpPosition;
void (*displayLine_func)(const char* message, LogMessageType messageType) = NULL;

static double     accel3D = 0;
static Time       time;
static RTCTimer   timer;
static RTCZero    rtc;
static int64_t    rtcEpochDelta; // set in setNow() and used in getGpsFixAndHandle() for correcting time in loop
static UBlox      ublox;
static UdpHeader  defaultUdpHeader;

static double accelX = 0;
static double accelY = 0;
static double accelZ = 0;

static uint32_t getNow();
static bool     initGps();
static void     initOnTheMove();
static void     resetRtcTimerEvents();
static void     rtcAlarmHandler();
static void     runSensorReadEvent(uint32_t now);
static void     saveAccelerometerPosition();
static void     setGpsActive(bool on);
static void     setNow(uint32_t newEpoch);

/**
* Runs every time acceleration is over the limits
* set by the user (if enabled).
*/
void accelerometerInt1Handler()
{
    if (digitalRead(ACCEL_INT1)) {
        updateOnTheMoveTimestampFlag = true;
    }
}

void checkTimer()
{
    if (minuteFlag) {
        timer.update(); // handle scheduled events
        minuteFlag = false;
    }
}

/**
*  Checks validity of data, adds valid points to the points list, syncs the RTC
*/
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt)
{
    sodaq_wdt_reset();

    if (!isGpsInitialized) {
        debugPrintln("delegateNavPvt exiting because GPS is not initialized.");
        return;
    }

    // note: db_printf gets enabled/disabled according to the "DEBUG" define (ublox.cpp)
    ublox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\r\n",
        NavPvt->year, NavPvt->month, NavPvt->day,
        NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
        NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

    // sync the RTC time
    if ((NavPvt->valid & GPS_TIME_VALIDITY) == GPS_TIME_VALIDITY) {
        uint32_t epoch = time.mktime(NavPvt->year, NavPvt->month, NavPvt->day, NavPvt->hour, NavPvt->minute, NavPvt->seconds);

        // check if there is an actual offset before setting the RTC
        if (abs((int64_t)getNow() - (int64_t)epoch) > MAX_RTC_EPOCH_OFFSET) {
            setNow(epoch);
        }
    }

    // check that the fix is OK and that it is a 3d fix or GNSS + dead reckoning combined
    if (((NavPvt->flags & GPS_FIX_FLAGS) == GPS_FIX_FLAGS) && ((NavPvt->fixType == 3) || (NavPvt->fixType == 4))) {
        tmpPosition.Lat = NavPvt->lat;
        tmpPosition.Lon = NavPvt->lon;
        tmpPosition.Alt = (int16_t)constrain(NavPvt->hMSL / 1000, INT16_MIN, INT16_MAX); // mm to m
        tmpPosition.NumberOfSatellites = NavPvt->numSV;

        isTmpPositionAltered = true;
    }
}

void fatal()
{
    setLedColor(RED);

    while (true) {
        sodaq_wdt_reset();
    }
}

/**
* Returns the current datetime (seconds since unix epoch).
*/
static uint32_t getNow()
{
    return rtc.getEpoch();
}


/**
* Shows and handles the boot up commands.
*/
static void handleBootUpCommands()
{
    do {
        showBootMenu(CONSOLE_STREAM);
    } while (!params.checkConfig(CONSOLE_STREAM));

    params.showConfig(&CONSOLE_STREAM);

    debugPrintln(params.commit() ? "Configuration changes committed" : "Configuration not changed");
}

/**
* Tries to get a GPS fix and handles the data.
* Times-out after params.getGpsFixTimeout seconds.
*/
bool handleGpsFixSequence()
{
    if (!isGpsInitialized) {
        debugPrintln("GPS is not initialized, not used in this sequence.");

        return false;
    }

    setLedColor(MAGENTA);

    debugPrintln("GPS Fix sequence started...");

    bool isFixSuccessful = false;
    setGpsActive(true);

    tmpPosition.NumberOfSatellites = 0; // reset satellites to use them as a quality metric in the loop
    uint32_t startTime = getNow();
    while ((getNow() - startTime <= params.getGpsFixTimeout())
        && (tmpPosition.NumberOfSatellites < params.getGpsMinSatelliteCount()))
    {
        sodaq_wdt_reset();
        uint16_t bytes = ublox.available();

        if (bytes) {
            rtcEpochDelta = 0;
            isTmpPositionAltered = false;
            ublox.GetPeriodic(bytes); // calls the delegate method for passing results

            startTime += rtcEpochDelta; // just in case the clock was changed (by the delegate in ublox.GetPeriodic)

            // isPendingReportDataRecordNew guarantees at least a 3d fix or GNSS + dead reckoning combined
            // and is good enough to keep, but the while loop should keep trying until timeout or sat count larger than set
            if (isTmpPositionAltered) {
                isFixSuccessful = true;
            }
        }
    }

    setGpsActive(false); // turn off gps as soon as it is not needed

    // populate all fields of the report record
    tmpPosition.Timestamp = getNow();

    if (isFixSuccessful) {
        tmpPosition.TimeToFix = tmpPosition.Timestamp - startTime;
    }
    else {
        tmpPosition.TimeToFix = 0xFF;

        tmpPosition.Lat = 0;
        tmpPosition.Lon = 0;

        setLedColor(RED);
        sodaq_wdt_safe_delay(5000);
    }

    setLedColor(NONE);

    return isFixSuccessful;
}

/**
* Initializes the GPS and leaves it on if succesful.
* Returns true if successful.
*/
static bool initGps()
{
    pinMode(GPS_ENABLE, OUTPUT);
    pinMode(GPS_TIMEPULSE, INPUT);

    // attempt to turn on and communicate with the GPS
    ublox.enable();
    digitalWrite(GPS_ENABLE, HIGH);
    ublox.flush();

    uint32_t startTime = getNow();
    bool found = false;
    while (!found && (getNow() - startTime <= GPS_COMM_CHECK_TIMEOUT)) {
        sodaq_wdt_reset();
        found = ublox.exists();
    }

    // check for success
    if (found) {
        setGpsActive(true); // properly turn on before returning
        return true;
    }

    debugPrintln("*** GPS not found!");

    // turn off before returning in case of failure
    setGpsActive(false);

    return false;
}

static bool initNetwork()
{
    setLedColor(BLUE);

    bool result = modem.initModem();

    if (!result) {
        setLedColor(RED);
        sodaq_wdt_safe_delay(5000);
    }

    setLedColor(NONE);

    return result;
}

/**
* Initializes the on-the-move functionality (interrupt on acceleration).
*/
static void initOnTheMove()
{
    pinMode(ACCEL_INT1, INPUT);
    attachInterrupt(ACCEL_INT1, accelerometerInt1Handler, CHANGE);

    // Configure EIC to use GCLK1 which uses XOSC32K, XOSC32K is already running in standby
    // This has to be done after the first call to attachInterrupt()
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
        GCLK_CLKCTRL_GEN_GCLK1 |
        GCLK_CLKCTRL_CLKEN;

    accelerometer.enableAccelerometer(
        Sodaq_LSM303AGR::LowPowerMode,
        Sodaq_LSM303AGR::HrNormalLowPower10Hz,
        Sodaq_LSM303AGR::XYZ,
        Sodaq_LSM303AGR::Scale8g,
        true);
    sodaq_wdt_safe_delay(100);

    accelerometer.enableInterrupt1(
        Sodaq_LSM303AGR::XHigh
        | Sodaq_LSM303AGR::XLow
        | Sodaq_LSM303AGR::YHigh
        | Sodaq_LSM303AGR::YLow
        | Sodaq_LSM303AGR::ZHigh
        | Sodaq_LSM303AGR::ZLow,
        params.getAccelerationPercentage() * 8.0 / 100.0,
        params.getAccelerationDuration(),
        Sodaq_LSM303AGR::MovementRecognition);
}

/**
* Initializes the RTC.
*/
void initRtc()
{
    rtc.begin();

    // Schedule the wakeup interrupt for every minute
    // Alarm is triggered 1 cycle after match
    rtc.setAlarmSeconds(59);
    rtc.enableAlarm(RTCZero::MATCH_SS);   // alarm every minute
    rtc.attachInterrupt(rtcAlarmHandler); // attach handler

    // This sets it to 2000-01-01
    rtc.setEpoch(0);
}

/**
* Initializes the RTC Timer and schedules the default events.
*/
void initRtcTimer()
{
    timer.setNowCallback(getNow); // set how to get the current time
    timer.allowMultipleEvents();

    resetRtcTimerEvents();
}

void initUdpHeader()
{
    defaultUdpHeader.Version = 1; // always version=1
    memcpy(defaultUdpHeader.Imei, &modem.cachedImei, sizeof(defaultUdpHeader.Imei));
    // reverse the array
    // (the exaple shows that IMEI = 391855893742972 should be 01 64 64 0F 59 85 7C
    size_t n = sizeof(defaultUdpHeader.Imei);
    for (size_t i = 0; i < n / 2; ++i) {
        int tmp = defaultUdpHeader.Imei[i];
        defaultUdpHeader.Imei[i] = defaultUdpHeader.Imei[n - 1 - i];
        defaultUdpHeader.Imei[n - 1 - i] = tmp;
    }

    strncpy(defaultUdpHeader.Token, params.getAttToken(), sizeof(defaultUdpHeader.Token));
}

void postLoop()
{
    if (pendingSensorRead) {
        pendingSensorRead                = false;
        lastOnTheMoveActivationTimestamp = getNow();
        updateOnTheMoveTimestampFlag     = false;

        saveAccelerometerPosition();
    }
}

void preLoop()
{
    if (sodaq_wdt_flag) {
        sodaq_wdt_reset();
        sodaq_wdt_flag = false;
    }

    if (!pendingSensorRead && !updateOnTheMoveTimestampFlag) {
        double t = (double)params.getAccelerationSensitivity() / 10;
        updateOnTheMoveTimestampFlag =
            abs(accelerometer.getX() - accelX) >= t ||
            abs(accelerometer.getY() - accelY) >= t ||
            abs(accelerometer.getZ() - accelZ) >= t;
    }

    if (updateOnTheMoveTimestampFlag) {
        setLedColor(BLUE);

        uint32_t i = getNow();
        if (i - lastOnTheMoveActivationTimestamp > params.getOnTheMoveBackoffTime()) {
            pendingSensorRead = true;
            lastOnTheMoveActivationTimestamp = i;
            debugPrintln("Acceleration handled, pending sensor read...");
        } else {
            debugPrintln("Acceleration handled, no action required");
            sodaq_wdt_safe_delay(500);
            setLedColor(NONE);
        }

        updateOnTheMoveTimestampFlag = false;
        saveAccelerometerPosition();
    }

    checkTimer();
}

void displayLine(const char* message, LogMessageType messageType)
{
    if (displayLine_func) {
        displayLine_func(message, messageType);
    }
}

/**
* Clears the RTC Timer events and schedules the default events.
*/
void resetRtcTimerEvents()
{
    timer.clearAllEvents();

    // Schedule the default fix event (if applicable)
    if (params.getSensorReadInterval() > 0) {
        timer.every(params.getSensorReadInterval() * 60, runSensorReadEvent);
    }
}

/**
* Runs every minute by the rtc alarm.
*/
void rtcAlarmHandler()
{
    minuteFlag = true;
}

void runSensorReadEvent(uint32_t now)
{
    pendingSensorRead = true;
}

/**
* Turns the GPS on or off.
*/
static void setGpsActive(bool on)
{
    sodaq_wdt_reset();

    if (on) {
        ublox.enable();
        digitalWrite(GPS_ENABLE, HIGH);
        ublox.flush();

        sodaq_wdt_safe_delay(80);

        PortConfigurationDDC pcd;

        uint8_t maxRetries = 6;
        int8_t retriesLeft;

        retriesLeft = maxRetries;
        while (!ublox.getPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
            debugPrintln("Retrying ublox.getPortConfigurationDDC(&pcd)...");
            sodaq_wdt_safe_delay(15);
        }
        if (retriesLeft == -1) {
            debugPrintln("ublox.getPortConfigurationDDC(&pcd) failed!");
            return;
        }

        pcd.outProtoMask = 1; // Disable NMEA
        retriesLeft = maxRetries;
        while (!ublox.setPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
            debugPrintln("Retrying ublox.setPortConfigurationDDC(&pcd)...");
            sodaq_wdt_safe_delay(15);
        }
        if (retriesLeft == -1) {
            debugPrintln("ublox.setPortConfigurationDDC(&pcd) failed!");
            return;
        }

        ublox.CfgMsg(UBX_NAV_PVT, 1); // Navigation Position Velocity TimeSolution
        ublox.funcNavPvt = delegateNavPvt;
    }
    else {
        ublox.disable();
        digitalWrite(GPS_ENABLE, LOW);
    }
}

static void saveAccelerometerPosition()
{
    accelX = accelerometer.getX();
    accelY = accelerometer.getY();
    accelZ = accelerometer.getZ();
}

/**
* Sets the RTC epoch and "rtcEpochDelta".
*/
static void setNow(uint32_t newEpoch)
{
    uint32_t currentEpoch = getNow();

    debugPrint("Setting RTC from ");
    debugPrint(currentEpoch);
    debugPrint(" to ");
    debugPrintln(newEpoch);

    rtcEpochDelta = newEpoch - currentEpoch;
    rtc.setEpoch(newEpoch);

    timer.adjust(currentEpoch, newEpoch);
}

void setupBoot()
{
    // preliminary modem init, to get the IMEI
    modem.initStream();
    modem.initNBIOT();
    modem.on();
    modem.readIMEI();

    pinMode(USB_DETECT, INPUT);
    if (digitalRead(USB_DETECT)) {
        #ifdef ENABLE_WATCHDOG
        sodaq_wdt_disable();
        #endif
        displayLine("Showing menu...", NormalMessageType);
        handleBootUpCommands();
        #ifdef ENABLE_WATCHDOG
        sodaq_wdt_enable(WDT_PERIOD_8X);
        #endif
    }

    modem.off();
}

void setupFinal()
{
    // make sure the debug option is honored
    if (params.getIsDebugOn() && ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM)) {
        DEBUG_STREAM.begin(DEBUG_BAUD);
    }

    if (params.getIsGpsEnabled()) {
        debugPrintln("Initializing GPS...");
        displayLine("Init GPS...", NormalMessageType);

        isGpsInitialized = initGps();
        if (!isGpsInitialized) {
            debugPrintln("Failed to initialize GPS!");
            displayLine("Failed to", ErrorMessageType);
            displayLine("init GPS",  ErrorMessageType);
            fatal();
        }
    }

    accelerometer.disableMagnetometer();
    pinMode(MAG_INT, OUTPUT);
    digitalWrite(MAG_INT, LOW);
    if (params.getAccelerationPercentage() > 0) {
        initOnTheMove();
    }

    debugPrintln("Connecting...");
    displayLine("Connecting...", NormalMessageType);
    while (!initNetwork());
    debugPrintln("Connected!");
    displayLine("Connected!", NormalMessageType);

    initRtcTimer();
    initUdpHeader();

    // disable the USB if not needed for debugging
    if (!params.getIsDebugOn() || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        consolePrintln("The USB is going to be disabled now.");

        SerialUSB.flush();
        sodaq_wdt_safe_delay(3000);
        SerialUSB.end();
        USBDevice.detach();
        USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
    }

    // disable the debug stream if it is not disabled by the above
    if (!params.getIsDebugOn() && ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        DEBUG_STREAM.flush();
        DEBUG_STREAM.end();
    }

    // disable the console stream if it is not disabled by the above,
    // and only if it is different than the debug stream
    if ((long)&CONSOLE_STREAM != (long)&SerialUSB && ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM)) {
        CONSOLE_STREAM.flush();
        CONSOLE_STREAM.end();
    }

    // wait for 2s before starting first sensor sequence
    sodaq_wdt_safe_delay(2000);

    saveAccelerometerPosition();
}

void setupInitial()
{
    sodaq_wdt_disable();

    setLedColor(YELLOW);

    CONSOLE_STREAM.begin(CONSOLE_BAUD);
    while (!CONSOLE_STREAM && millis() < STARTUP_DELAY);

    #ifdef ENABLE_WATCHDOG
    sodaq_wdt_enable(WDT_PERIOD_8X);
    #endif
    sodaq_wdt_reset();

    if ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM) {
        DEBUG_STREAM.begin(DEBUG_BAUD);
    }

    initRtc();

    Wire.begin();
}

void showUploadStatus(bool result, bool needBlink)
{
    debugPrintln(result ? "Upload Successful" : "Upload Failed");

    if (needBlink) {
      setLedColor(result ? GREEN : RED);
      sodaq_wdt_safe_delay(1000);
      setLedColor(NONE);
    }

    debugPrintln();
}

void uploadData(const uint8_t* data, uint8_t size, bool needHeader, bool needBlink)
{
    bool result;

    if (needHeader) {
        const size_t headerSize = sizeof(defaultUdpHeader.Raw);

        uint8_t buffer[headerSize + size];
        memcpy(buffer, defaultUdpHeader.Raw, headerSize);
        memcpy(buffer + headerSize, data, size);

        result = modem.transmit(buffer, headerSize + size) == headerSize + size;
    } else {
        result = modem.transmit(data, size) == size;
    }

    showUploadStatus(result, needBlink);
}
