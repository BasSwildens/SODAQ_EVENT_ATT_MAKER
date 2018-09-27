#include "CborBuilder.h"
#include "SodaqMaker.h"

#define PROJECT_VERSION "SODAQ SARAR4 MAKER 20180921.5"

#define ADC_AREF   3.3f
#define BATVOLT_R1 4.7f
#define BATVOLT_R2 10.0f

static CborBuilder cbor;

bool  initSensor();
float getSensorValue();
void  updateCborPayload(float sensorValue, uint16_t batteryValue);

void setup()
{
    setupInitial();

    consolePrintln(PROJECT_VERSION "\r\nBooting up...");

    params.setConfigResetCallback(onConfigReset);
    params.read();

    setupBoot();

    debugPrintln("Init Sensor...");
    if (!initSensor()) {
        debugPrintln("Failed to init sensor");
        fatal();
    }

    setupFinal();

    handleSensorReadProcedure(false);
}

void loop()
{
    preLoop();

    if (pendingSensorRead) {
        handleSensorReadProcedure(true);
    }

    postLoop();
}

void updateCborPayload(float sensorValue, uint16_t batteryValue)
{
    cbor.reset();

    cbor.writeTag(120);
    cbor.writeArray(1);
    cbor.map(isGpsInitialized ? 4 : 3);

    cbor.addNumber (sensorValue,  "t");
    cbor.addInteger(batteryValue, "v");
    cbor.addString (params.getForceOperator(), "o");

    if (isGpsInitialized) {
        cbor.addGps(tmpPosition.Lat / GPS_SCALING, tmpPosition.Lon / GPS_SCALING,
                    tmpPosition.Alt, "g");
    }
}

void handleSensorReadProcedure(bool isFromLoop)
{
    debugPrintln("Sensor read sequence started...");

    bool gpsSuccess = isFromLoop && handleGpsFixSequence();

    float sensorValue = getSensorValue();
    debugPrint("Temperature: ");
    debugPrint(sensorValue);

    if (gpsSuccess) {
        debugPrint(" C, Lat: ");
        debugPrint(tmpPosition.Lat);
        debugPrint(" Lon: ");
        debugPrintln(tmpPosition.Lon);
    }
    else {
        debugPrintln(" C");
    }

    uint16_t batteryValue = getBatteryVoltageMV();
    debugPrint("Battery voltage: ");
    debugPrint(batteryValue);
    debugPrintln(" mV");

    debugPrintln("Uploading...");

    updateCborPayload(sensorValue, batteryValue);

    uploadData(cbor.getData(), cbor.getSize(), params.getTelco() > 0, isFromLoop);
}

float getSensorValue()
{
    return accelerometer.getTemperature() + params.getTemperatureOffset();
}

/*
 * Compute the battery value in milliVolt
 *
 * The SODAQ boards have two resistors (R1 and R2) between VCC and ground.
 * The middle point is connected to an analog port.
 */
uint16_t getBatteryVoltageMV()
{
    return (uint16_t)((ADC_AREF * 1000 / ((1 << 10) - 1)) *
           (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BAT_VOLT));
}

bool initSensor()
{
    return accelerometer.checkWhoAmI();
}
