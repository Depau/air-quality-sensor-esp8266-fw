//
// Created by depau on 4/30/21.
//
// Little-endianness is assumed. The device ID will therefore be swapped compared to what is reported in the
// datasheet. Other 16 bit data will be in the correct byte order.
//
// Datasheet fetched on 2021-04-30 from https://nettigo.pl/attachments/415
//

#ifndef AIR_SENSORS_SENDER_SDS011_H
#define AIR_SENSORS_SENDER_SDS011_H

#include <HardwareSerial.h>

#define SDS011_ANY 0xFFFF

#define SDS011_ERR -1

// Enums

typedef enum sds011_operation {
    SDS011_OPERATION_QUERY = 0,
    SDS011_OPERATION_SET = 1,
} sds011_operation_t;

typedef enum sds011_reporting_mode {
    SDS011_REPORT_MODE_ACTIVE = 0,
    SDS011_REPORT_MODE_QUERY = 1,
} sds011_reporting_mode_t;

typedef enum sds011_sleep_mode {
    SDS011_SLEEP_MODE_SLEEP = 0,
    SDS011_SLEEP_MODE_WORK = 1,
} sds011_sleep_mode_t;

typedef enum sds011_command {
    SDS011_CMD_DATA_REPORTING = 2,
    SDS011_CMD_QUERY = 4,
    SDS011_CMD_DEV_ID = 5,
    SDS011_CMD_SLEEP_WORK = 6,
    SDS011_CMD_WORK_PERIOD = 8,
    SDS011_CMD_VERSION = 7,
} sds011_command_t;


// Command payloads

typedef struct __attribute__((packed)) sds011_command_raw {
    uint8_t bytes[19];
} sds011_command_raw_t;

typedef struct __attribute__((packed)) sds011_command_common {
    uint8_t head;
    uint8_t commandId;
    uint8_t data[13];
    uint16_t deviceId;
    uint8_t checksum;
    uint8_t tail;
} sds011_command_common_t;

typedef struct __attribute__((packed)) sds011_command_setting {
    uint8_t head;
    uint8_t commandId;
    uint8_t command;
    uint8_t operation;
    uint8_t setting;
    uint8_t data[10];
    uint16_t sensorID;
    uint8_t checksum;
    uint8_t tail;
} sds011_command_setting_t;

typedef struct __attribute__((packed)) sds011_command_set_dev_id {
    uint8_t head;
    uint8_t commandId;
    uint8_t command;
    uint8_t data[10];
    uint16_t newDeviceIdBE;
    uint16_t deviceId;
    uint8_t checksum;
    uint8_t tail;
} sds011_command_set_dev_id_t;

typedef union __attribute__((packed)) sds011_command_u {
    sds011_command_raw raw;
    sds011_command_common_t common;
    sds011_command_setting_t setting;
    sds011_command_set_dev_id set_dev_id;
} sds011_command_u;


// Response payloads

typedef struct __attribute__((packed)) sds011_response_raw {
    uint8_t bytes[10];
} sds011_response_raw_t;

typedef struct __attribute__((packed)) sds011_response_common {
    uint8_t head;
    uint8_t commandId;
    uint8_t data[4];
    uint16_t deviceId;
    uint8_t checksum;
    uint8_t tail;
} sds011_response_common_t;

typedef struct __attribute__((packed)) sds011_response_setting {
    uint8_t head;
    uint8_t commandId;
    uint8_t command;
    uint8_t operation;
    uint8_t setting;
    uint8_t reserved;
    uint16_t deviceId;
    uint8_t checksum;
    uint8_t tail;
} sds011_response_setting_t;

typedef struct __attribute__((packed)) sds011_response_query {
    uint8_t head;
    uint8_t commandId;
    uint16_t pm25;
    uint16_t pm10;
    uint16_t deviceId;
    uint8_t checksum;
    uint8_t tail;
} sds011_response_query_t;

typedef struct __attribute__((packed)) sds011_response_version {
    uint8_t head;
    uint8_t commandId;
    uint8_t command;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint16_t deviceId;
    uint8_t checksum;
    uint8_t tail;
} sds011_response_version_t;

typedef union __attribute__((packed)) sds011_response_u {
    sds011_response_raw_t raw;
    sds011_response_common_t common;
    sds011_response_setting_t setting;
    sds011_response_query_t query;
    sds011_response_version_t version;
} sds011_response_u;


// High-level API
typedef struct sds011_dev_info {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint16_t deviceId;
} sds011_dev_info_t;

typedef struct sds011_pm_data {
    float pm25;
    float pm10;
    uint16_t deviceId;
} sds011_pm_data_t;

class SDS011 {
protected:
    Stream *_serial;
    uint32_t _timeout = 1000;

    static void fillBoilerplate(sds011_command_u *cmd);

    bool sendCommand(sds011_command_u *cmd);

    bool readResponse(sds011_response_u *response, uint32_t timeout);

    bool readPmData(sds011_pm_data_t *data, uint32_t timeout);

    bool seekRespStart();

    static bool checkChecksumResp(sds011_response_u *response);

    static uint8_t genChecksum(uint8_t *data, size_t len);

    bool getSetSetting(uint16_t sensorID, sds011_operation_t operation, sds011_command_t setting, uint8_t value, sds011_response_u *response);

public:
    explicit SDS011(Stream *serial) : _serial{serial} {};

    bool setDataReporting(sds011_reporting_mode_t mode, uint16_t sensorID);

    bool setDataReporting(sds011_reporting_mode_t mode) { return setDataReporting(mode, SDS011_ANY); }

    bool getDataReporting(uint16_t sensorID, uint8_t *dataReporting);

    bool getDataReporting(uint8_t *dataReporting) { return getDataReporting(SDS011_ANY, dataReporting); }

    bool setSleepMode(sds011_sleep_mode_t mode, uint16_t sensorID);

    bool setSleepMode(sds011_sleep_mode_t mode) { return setSleepMode(mode, SDS011_ANY); }

    bool getSleepMode(uint16_t sensorID, uint8_t *sleepMode);

    bool getSleepMode(uint8_t *sleepMode) { return getSleepMode(SDS011_ANY, sleepMode); }

    bool setWorkingPeriod(uint8_t workPeriod, uint16_t sensorID);

    bool setWorkingPeriod(uint8_t workPeriod) { return setWorkingPeriod(workPeriod, SDS011_ANY); }

    bool getWorkingPeriod(uint16_t sensorID, uint8_t *workingPeriod);

    bool getWorkingPeriod(uint8_t *workingPeriod) { return getWorkingPeriod(SDS011_ANY, workingPeriod); }

    bool getInfo(uint16_t sensorID, sds011_dev_info_t *devInfo);

    bool getInfo(sds011_dev_info_t *devInfo) { return getInfo(SDS011_ANY, devInfo); };

    bool query(uint16_t sensorID, sds011_pm_data_t *data);

    bool query(sds011_pm_data_t *data) { return query(SDS011_ANY, data); };

    bool read(sds011_pm_data_t *data);

    void setTimeout(uint32_t timeout) { _timeout = timeout; }
};


#endif //AIR_SENSORS_SENDER_SDS011_H
