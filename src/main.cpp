#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <LeifHomieLib.h>
#include <SoftwareSerial.h>
#include <bsec.h>
#include <SDS011.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <FS.h>

#include "config.h"

const uint8_t bsec_config_iaq[] = {
#include <config/generic_33v_3s_28d/bsec_iaq.txt>
};

#define BSEC_STATE_FILENAME "/bsec_state.bin"
#define BSEC_STATE_WRITE_INTERVAL_MS (2 * 60 * 60 * 1000)

uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint8_t prevBsecAccuracy = 0;
unsigned long lastWriteBsecState = millis();

bool otaRunning = false;

// SDS011
SoftwareSerial sdsSerial(SDS_RX, SDS_TX);
SDS011 sds(&sdsSerial);

HomieDevice homie;

// BME680
HomieNode *homieNodeBme680 = nullptr;

HomieProperty *homiePropRawTemperature = nullptr;
HomieProperty *homiePropPressure = nullptr;
HomieProperty *homiePropRawHumidity = nullptr;
HomieProperty *homiePropGasResistance = nullptr;
HomieProperty *homiePropIaq = nullptr;
HomieProperty *homiePropIaqAccuracy = nullptr;
HomieProperty *homiePropTemperature = nullptr;
HomieProperty *homiePropHumidity = nullptr;
HomieProperty *homiePropStaticIaq = nullptr;
HomieProperty *homiePropStaticIaqAccuracy = nullptr;
HomieProperty *homiePropCo2Equivalent = nullptr;
HomieProperty *homiePropCo2EquivalentAccuracy = nullptr;
HomieProperty *homiePropBreathVocEquivalent = nullptr;
HomieProperty *homiePropBreathVocEquivalentAccuracy = nullptr;
HomieProperty *homiePropPowerOnStabStatus = nullptr;
HomieProperty *homiePropStabStatus = nullptr;

HomieProperty *homiePropBsecStatus = nullptr;
HomieProperty *homiePropBme680Status = nullptr;

// SDS011
HomieNode *homieNodeSds011 = nullptr;

HomieProperty *homiePropPm10 = nullptr;
HomieProperty *homiePropPm25 = nullptr;

// BSEC crap
Bsec bsec;
int16_t lastBmeStatus = 0x7FFF;
int16_t lastBsecStatus = 0x7FFF;

bsec_virtual_sensor_t bsecSensorList[] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_STABILIZATION_STATUS
};

void setupHomieTree() {
    homieNodeBme680 = homie.NewNode();
    homieNodeBme680->strID = "air-quality";
    homieNodeBme680->strFriendlyName = "Air quality sensor";
    homieNodeBme680->strType = "BME680";

    homiePropRawTemperature = homieNodeBme680->NewProperty();
    homiePropRawTemperature->SetRetained(true);
    homiePropRawTemperature->SetSettable(false);
    homiePropRawTemperature->strID = "raw-temperature";
    homiePropRawTemperature->strFriendlyName = "Raw temperature";
    homiePropRawTemperature->datatype = homieFloat;
    homiePropRawTemperature->SetUnit("°C");

    homiePropTemperature = homieNodeBme680->NewProperty();
    homiePropTemperature->SetRetained(true);
    homiePropTemperature->SetSettable(false);
    homiePropTemperature->strID = "temperature";
    homiePropTemperature->strFriendlyName = "Temperature";
    homiePropTemperature->datatype = homieFloat;
    homiePropTemperature->SetUnit("°C");

    homiePropPressure = homieNodeBme680->NewProperty();
    homiePropPressure->SetRetained(true);
    homiePropPressure->SetSettable(false);
    homiePropPressure->strID = "pressure";
    homiePropPressure->strFriendlyName = "Pressure";
    homiePropPressure->datatype = homieFloat;
    homiePropPressure->SetUnit("Pa");

    homiePropRawHumidity = homieNodeBme680->NewProperty();
    homiePropRawHumidity->SetRetained(true);
    homiePropRawHumidity->SetSettable(false);
    homiePropRawHumidity->strID = "raw-humidity";
    homiePropRawHumidity->strFriendlyName = "Raw humidity";
    homiePropRawHumidity->datatype = homieFloat;
    homiePropRawHumidity->SetUnit("%");
    homiePropRawHumidity->strFormat = "0:100";

    homiePropHumidity = homieNodeBme680->NewProperty();
    homiePropHumidity->SetRetained(true);
    homiePropHumidity->SetSettable(false);
    homiePropHumidity->strID = "humidity";
    homiePropHumidity->strFriendlyName = "Humidity";
    homiePropHumidity->datatype = homieFloat;
    homiePropHumidity->SetUnit("%");
    homiePropHumidity->strFormat = "0:100";

    homiePropGasResistance = homieNodeBme680->NewProperty();
    homiePropGasResistance->SetRetained(true);
    homiePropGasResistance->SetSettable(false);
    homiePropGasResistance->strID = "gas-resistance";
    homiePropGasResistance->strFriendlyName = "Gas resistance";
    homiePropGasResistance->datatype = homieFloat;
    homiePropGasResistance->SetUnit("Ω");

    homiePropIaq = homieNodeBme680->NewProperty();
    homiePropIaq->SetRetained(true);
    homiePropIaq->SetSettable(false);
    homiePropIaq->strID = "iaq";
    homiePropIaq->strFriendlyName = "IAQ";
    homiePropIaq->strFormat = "0:500";
    homiePropIaq->datatype = homieFloat;

    homiePropIaqAccuracy = homieNodeBme680->NewProperty();
    homiePropIaqAccuracy->SetRetained(true);
    homiePropIaqAccuracy->SetSettable(false);
    homiePropIaqAccuracy->strID = "iaq-accuracy";
    homiePropIaqAccuracy->strFriendlyName = "IAQ accuracy";
    homiePropIaqAccuracy->datatype = homieInteger;

    homiePropStaticIaq = homieNodeBme680->NewProperty();
    homiePropStaticIaq->SetRetained(true);
    homiePropStaticIaq->SetSettable(false);
    homiePropStaticIaq->strID = "static-iaq";
    homiePropStaticIaq->strFriendlyName = "Static IAQ";
    homiePropStaticIaq->datatype = homieFloat;

    homiePropStaticIaqAccuracy = homieNodeBme680->NewProperty();
    homiePropStaticIaqAccuracy->SetRetained(true);
    homiePropStaticIaqAccuracy->SetSettable(false);
    homiePropStaticIaqAccuracy->strID = "static-iaq-accuracy";
    homiePropStaticIaqAccuracy->strFriendlyName = "Static IAQ accuracy";
    homiePropStaticIaqAccuracy->datatype = homieInteger;

    homiePropCo2Equivalent = homieNodeBme680->NewProperty();
    homiePropCo2Equivalent->SetRetained(true);
    homiePropCo2Equivalent->SetSettable(false);
    homiePropCo2Equivalent->strID = "co2-equivalent";
    homiePropCo2Equivalent->strFriendlyName = "CO₂ equivalent";
    homiePropCo2Equivalent->datatype = homieFloat;
    homiePropCo2Equivalent->SetUnit("ppm");

    homiePropCo2EquivalentAccuracy = homieNodeBme680->NewProperty();
    homiePropCo2EquivalentAccuracy->SetRetained(true);
    homiePropCo2EquivalentAccuracy->SetSettable(false);
    homiePropCo2EquivalentAccuracy->strID = "co2-equivalent-accuracy";
    homiePropCo2EquivalentAccuracy->strFriendlyName = "CO₂ equivalent accuracy";
    homiePropCo2EquivalentAccuracy->datatype = homieInteger;

    homiePropBreathVocEquivalent = homieNodeBme680->NewProperty();
    homiePropBreathVocEquivalent->SetRetained(true);
    homiePropBreathVocEquivalent->SetSettable(false);
    homiePropBreathVocEquivalent->strID = "breath-voc-equivalent";
    homiePropBreathVocEquivalent->strFriendlyName = "Breath VOC equivalent";
    homiePropBreathVocEquivalent->datatype = homieFloat;
    homiePropBreathVocEquivalent->SetUnit("ppm");

    homiePropBreathVocEquivalentAccuracy = homieNodeBme680->NewProperty();
    homiePropBreathVocEquivalentAccuracy->SetRetained(true);
    homiePropBreathVocEquivalentAccuracy->SetSettable(false);
    homiePropBreathVocEquivalentAccuracy->strID = "breath-voc-equivalent-accuracy";
    homiePropBreathVocEquivalentAccuracy->strFriendlyName = "Breath VOC equivalent accuracy";
    homiePropBreathVocEquivalentAccuracy->datatype = homieInteger;

    homiePropBsecStatus = homieNodeBme680->NewProperty();
    homiePropBsecStatus->SetRetained(true);
    homiePropBsecStatus->SetSettable(false);
    homiePropBsecStatus->strID = "bsec-status";
    homiePropBsecStatus->strFriendlyName = "BSEC status";
    homiePropBsecStatus->datatype = homieInteger;

    homiePropBme680Status = homieNodeBme680->NewProperty();
    homiePropBme680Status->SetRetained(true);
    homiePropBme680Status->SetSettable(false);
    homiePropBme680Status->strID = "bme680-status";
    homiePropBme680Status->strFriendlyName = "BME680 status";
    homiePropBme680Status->datatype = homieInteger;

    homiePropPowerOnStabStatus = homieNodeBme680->NewProperty();
    homiePropPowerOnStabStatus->SetRetained(true);
    homiePropPowerOnStabStatus->SetSettable(false);
    homiePropPowerOnStabStatus->strID = "power-on-stabilization-done";
    homiePropPowerOnStabStatus->strFriendlyName = "Power-on stabilization status";
    homiePropPowerOnStabStatus->datatype = homieBool;

    homiePropStabStatus = homieNodeBme680->NewProperty();
    homiePropStabStatus->SetRetained(true);
    homiePropStabStatus->SetSettable(false);
    homiePropStabStatus->strID = "stabilization-done";
    homiePropStabStatus->strFriendlyName = "Stabilization status";
    homiePropStabStatus->datatype = homieBool;

    homieNodeSds011 = homie.NewNode();
    homieNodeSds011->strID = "particulate";
    homieNodeSds011->strFriendlyName = "Particulate sensor";
    homieNodeSds011->strType = "SDS011";

    homiePropPm10 = homieNodeSds011->NewProperty();
    homiePropPm10->SetRetained(true);
    homiePropPm10->SetSettable(false);
    homiePropPm10->strID = "pm10";
    homiePropPm10->strFriendlyName = "PM10";
    homiePropPm10->datatype = homieFloat;
    homiePropPm10->SetUnit("μg/m³");

    homiePropPm25 = homieNodeSds011->NewProperty();
    homiePropPm25->SetRetained(true);
    homiePropPm25->SetSettable(false);
    homiePropPm25->strID = "pm25";
    homiePropPm25->strFriendlyName = "PM2.5";
    homiePropPm25->datatype = homieFloat;
    homiePropPm25->SetUnit("μg/m³");
}

void saveBsecState() {
    bsec.getState(bsecState);
    File file = SPIFFS.open(BSEC_STATE_FILENAME, "w");
    file.write(bsecState, sizeof(bsecState));
    file.close();
    lastWriteBsecState = millis();
    Serial.println("BSEC state persisted");
}

void setup() {
    Serial.begin(74880);
    sdsSerial.begin(9600);
    Wire.begin(BME_SDA, BME_SCL);

    Serial.print(F("\r\n\r\nConnecting to "));
    Serial.print(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    pinMode(LED_BUILTIN, OUTPUT);
    uint8_t led = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(F("."));
        digitalWrite(LED_BUILTIN, led);
        led = !led;
        delay(500);
    }
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println();
    Serial.print(F("Connected: "));
    Serial.println(WiFi.localIP());

    MDNS.begin(WIFI_HOSTNAME);
    if (OTA_PASSWORD[0] != '\0') {
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.setRebootOnSuccess(true);
        ArduinoOTA.setHostname(WIFI_HOSTNAME);
        ArduinoOTA.setPort(OTA_PORT);

        ArduinoOTA.onStart([]() {
            saveBsecState();
            otaRunning = true;
            Serial.println(F("OTA upgrade started"));
        });
        ArduinoOTA.onEnd([]() {
            Serial.println(F("OTA upgrade successfully completed"));
        });
        ArduinoOTA.onError([](ota_error_t err) {
            Serial.print("OTA error: ");
            switch (err) {
                case OTA_AUTH_ERROR:
                    Serial.println("AUTH");
                    break;
                case OTA_BEGIN_ERROR:
                    Serial.println("BEGIN");
                    break;
                case OTA_CONNECT_ERROR:
                    Serial.println("CONNECT");
                    break;
                case OTA_RECEIVE_ERROR:
                    Serial.println("RECEIVE");
                    break;
                case OTA_END_ERROR:
                    Serial.println("END");
                    break;
            }
            ESP.reset();
        });
        ArduinoOTA.begin();
        Serial.println("OTA server up");
    }

    HomieLibRegisterDebugPrintCallback([](const char *szText) {
        Serial.print(szText);
    });

    Serial.println(F("Bringing up Homie"));
    setupHomieTree();
    homie.strID = "air-sensor";
    homie.strFriendlyName = "Air quality sensor";
    homie.strMqttServerIP = MQTT_IP;
    homie.Init();
    Serial.println(F("Homie is running"));

    SPIFFSConfig fsConfig;
    fsConfig.setAutoFormat(true);
    SPIFFS.setConfig(fsConfig);
    SPIFFS.begin();

    bsec.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    bsec.setConfig(bsec_config_iaq);

    if (SPIFFS.exists(BSEC_STATE_FILENAME)) {
        File file = SPIFFS.open(BSEC_STATE_FILENAME, "r");
        file.read(bsecState, sizeof(bsecState));
        file.close();
        bsec.setState(bsecState);
        Serial.println("Loaded BSEC state");
    }

    bsec.updateSubscription(bsecSensorList, sizeof(bsecSensorList), BSEC_SAMPLE_RATE_LP);

    String output = "BSEC version " + String(bsec.version.major) + "." + String(bsec.version.minor) + "." +
                    String(bsec.version.major_bugfix) + "." + String(bsec.version.minor_bugfix);
    Serial.println(output);

    sds011_dev_info_t sdsInfo;
    if (!sds.getInfo(&sdsInfo)) {
        Serial.println("Fatal: failed to retrieve SDS011 device info");
        panic();
    }
    if (!sds.setWorkingPeriod(1)) { // once per minute
        Serial.println("Fatal: failed to set SDS011 working period");
        panic();
    }
    if (!sds.setDataReporting(SDS011_REPORT_MODE_ACTIVE)) {
        Serial.println("Fatal: failed to set SDS011 data reporting mode");
        panic();
    }
    if (!sds.setSleepMode(SDS011_SLEEP_MODE_WORK)) {
        Serial.println("Fatal: failed to set SDS011 sleep mode");
        panic();
    }

    output = "SDS011 version " + String(sdsInfo.year) + "-" + String(sdsInfo.month) + "-" + String(sdsInfo.day) +
             ", sensor ID: " + String(sdsInfo.deviceId, HEX);
    Serial.println(output);

    // Query the sensor once on start to avoid publishing 0.0
    sds011_pm_data_t pmData;
    if (sds.query(&pmData)) {
        homiePropPm25->SetValue(String(pmData.pm25));
        homiePropPm10->SetValue(String(pmData.pm10));
    }
}

void checkBsecStatus() {
    String output;
    if (bsec.status != BSEC_OK) {
        if (bsec.status < BSEC_OK) {
            output = "BSEC error code : " + String(bsec.status);
            Serial.println(output);
            panic();
        } else {
            output = "BSEC warning code : " + String(bsec.status);
            Serial.println(output);
        }
    }

    if (bsec.bme680Status != BME680_OK) {
        if (bsec.bme680Status < BME680_OK) {
            output = "BME680 error code : " + String(bsec.bme680Status);
            Serial.println(output);
            panic();
        } else {
            output = "BME680 warning code : " + String(bsec.bme680Status);
            Serial.println(output);
        }
    }
}

void loop() {
    ArduinoOTA.handle();
    if (otaRunning) {
        return;
    }

    homie.Loop();

    if (lastBmeStatus != bsec.bme680Status) {
        homiePropBme680Status->SetValue(String(bsec.bme680Status));
        lastBmeStatus = (unsigned char) bsec.bme680Status;
    }
    if (lastBsecStatus != bsec.status) {
        homiePropBsecStatus->SetValue(String(bsec.status));
        lastBsecStatus = bsec.status;
    }

    if (bsec.run()) {
        homiePropRawTemperature->SetValue(String(bsec.rawTemperature));
        homiePropTemperature->SetValue(String(bsec.temperature));
        homiePropPressure->SetValue(String(bsec.pressure));
        homiePropRawHumidity->SetValue(String(bsec.rawHumidity));
        homiePropHumidity->SetValue(String(bsec.humidity));
        homiePropGasResistance->SetValue(String(bsec.gasPercentageAcccuracy));
        homiePropIaq->SetValue(String(bsec.iaq));
        homiePropIaqAccuracy->SetValue(String(bsec.iaqAccuracy));
        homiePropStaticIaq->SetValue(String(bsec.staticIaq));
        homiePropStaticIaqAccuracy->SetValue(String(bsec.staticIaqAccuracy));
        homiePropCo2Equivalent->SetValue(String(bsec.co2Equivalent));
        homiePropCo2EquivalentAccuracy->SetValue(String(bsec.co2Accuracy));
        homiePropBreathVocEquivalent->SetValue(String(bsec.breathVocEquivalent));
        homiePropBreathVocEquivalentAccuracy->SetValue(String(bsec.breathVocAccuracy));
        homiePropPowerOnStabStatus->SetBool((bool) bsec.runInStatus);
        homiePropStabStatus->SetBool((bool) bsec.stabStatus);

        if (bsec.iaqAccuracy > prevBsecAccuracy || (millis() - lastWriteBsecState) > BSEC_STATE_WRITE_INTERVAL_MS) {
            saveBsecState();
        }
        prevBsecAccuracy = bsec.iaqAccuracy;

    } else {
        checkBsecStatus();
    }

    sds011_pm_data_t pmData;
    if (sds.read(&pmData)) {
        homiePropPm25->SetValue(String(pmData.pm25));
        homiePropPm10->SetValue(String(pmData.pm10));
    }
}
