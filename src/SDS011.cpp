//
// Created by depau on 4/30/21.
//

#include <Arduino.h>
#include <HomieLogger.h>
#include "SDS011.h"

bool SDS011::setDataReporting(sds011_reporting_mode_t mode, uint16_t sensorID) {
    sds011_response_u resp;
    return getSetSetting(sensorID, SDS011_OPERATION_SET, SDS011_CMD_DATA_REPORTING, mode, &resp);
}

bool SDS011::getDataReporting(uint16_t sensorID, uint8_t *dataReporting) {
    sds011_response_u resp;
    if (!getSetSetting(sensorID, SDS011_OPERATION_QUERY, SDS011_CMD_DATA_REPORTING, 0, &resp)) {
        return false;
    }
    *dataReporting = resp.setting.setting;
    return true;
}

bool SDS011::setSleepMode(sds011_sleep_mode_t mode, uint16_t sensorID) {
    sds011_response_u resp;
    return getSetSetting(sensorID, SDS011_OPERATION_SET, SDS011_CMD_SLEEP_WORK, mode, &resp);
}

bool SDS011::getSleepMode(uint16_t sensorID, uint8_t *sleepMode) {
    sds011_response_u resp;
    if (!getSetSetting(sensorID, SDS011_OPERATION_QUERY, SDS011_CMD_SLEEP_WORK, 0, &resp)) {
        return false;
    }
    *sleepMode = resp.setting.setting;
    return true;
}

bool SDS011::setWorkingPeriod(uint8_t workPeriod, uint16_t sensorID) {
    if (workPeriod > 30) {
        HLogger.println("Invalid SDS011 working period! Must be from 0 to 30");
        return false;
    }
    sds011_response_u resp;
    return getSetSetting(sensorID, SDS011_OPERATION_SET, SDS011_CMD_WORK_PERIOD, workPeriod, &resp);
}

bool SDS011::getWorkingPeriod(uint16_t sensorID, uint8_t *workingPeriod) {
    sds011_response_u resp;
    if (!getSetSetting(sensorID, SDS011_OPERATION_QUERY, SDS011_CMD_WORK_PERIOD, 0, &resp)) {
        return false;
    }
    *workingPeriod = resp.setting.setting;
    return true;
}


bool SDS011::getSetSetting(uint16_t sensorID, sds011_operation_t operation, sds011_command_t setting, uint8_t value,
                           sds011_response_u *response) {
    sds011_command_u cmd = {{0}};
    cmd.setting.sensorID = sensorID;
    cmd.setting.operation = operation;
    cmd.setting.command = setting;
    cmd.setting.setting = value;
    fillBoilerplate(&cmd);
    if (!sendCommand(&cmd)) {
        return false;
    }
    if (!readResponse(response, _timeout)) {
        return false;
    }
    if (!checkChecksumResp(response)) {
        return false;
    }
    if (cmd.setting.setting != response->setting.setting) {
        return false;
    }
    return true;
}


bool SDS011::getInfo(uint16_t sensorID, sds011_dev_info_t *devInfo) {
    sds011_command_u cmd = {{0}};
    cmd.setting.command = SDS011_CMD_VERSION;
    cmd.setting.sensorID = sensorID;
    fillBoilerplate(&cmd);
    if (!sendCommand(&cmd)) {
        return false;
    }
    sds011_response_u resp;
    if (!readResponse(&resp, _timeout)) {
        return false;
    }
    if (!checkChecksumResp(&resp)) {
        return false;
    }
    devInfo->deviceId = resp.common.deviceId;
    devInfo->year = resp.version.year;
    devInfo->month = resp.version.month;
    devInfo->day = resp.version.day;
    return true;
}

bool SDS011::query(uint16_t sensorID, sds011_pm_data_t *data) {
    sds011_command_u cmd = {{0}};
    cmd.setting.command = SDS011_CMD_QUERY;
    cmd.setting.sensorID = sensorID;
    fillBoilerplate(&cmd);

    if (!sendCommand(&cmd)) {
        return false;
    }
    return readPmData(data, _timeout);
}

bool SDS011::read(sds011_pm_data_t *data) {
    return readPmData(data, 0);
}


bool SDS011::readPmData(sds011_pm_data_t *data, uint32_t timeout) {
    sds011_response_u resp;
    if (!readResponse(&resp, timeout)) {
        return false;
    }
    if (!checkChecksumResp(&resp)) {
        return false;
    }
    data->pm25 = ((float) resp.query.pm25) / 10;
    data->pm10 = ((float) resp.query.pm10) / 10;
    data->deviceId = resp.common.deviceId;
    return true;
}


void SDS011::fillBoilerplate(sds011_command_u *cmd) {
    cmd->common.head = 0xAA;
    cmd->common.commandId = 0xB4;
    cmd->common.checksum = genChecksum(cmd->raw.bytes + 2, 15);
    cmd->common.tail = 0xAB;
}

bool SDS011::sendCommand(sds011_command_u *cmd) {
    unsigned long start = millis();

    do {
        size_t written = _serial->write(cmd->raw.bytes, sizeof(cmd->raw.bytes));
        if (written == 0) {
            yield();
            continue;
        }
        HLogger.print("SDS011: send ");
        for (unsigned char byte : cmd->raw.bytes) {
            HLogger.print(byte, HEX);
            HLogger.print(" ");
        }
        HLogger.println();
        return true;

    } while (millis() < start + _timeout);

    HLogger.println("SDS011: Failed to send command");
    return false;
}

bool SDS011::readResponse(sds011_response_u *response, uint32_t timeout) {
    unsigned long start = millis();

    do {
        if (!seekRespStart()) {
            yield();
            continue;
        }
        if (_serial->available() < 10) {
            yield();
            continue;
        }

        _serial->readBytes(response->raw.bytes, 10);
        HLogger.print("SDS011: recv ");
        for (unsigned char byte : response->raw.bytes) {
            HLogger.print(byte, HEX);
            HLogger.print(" ");
        }
        HLogger.println();
        return true;

    } while (millis() < start + timeout);
    return false;
}

bool SDS011::seekRespStart() {
    while (_serial->available()) {
        if (_serial->peek() == 0xAA) {
            return true;
        }
        _serial->read();
    }
    return false;
}

bool SDS011::checkChecksumResp(sds011_response_u *response) {
    uint8_t checksum = genChecksum(response->raw.bytes + 2, 6);
    return checksum == response->common.checksum;
}

uint8_t SDS011::genChecksum(uint8_t *data, size_t len) {
    uint16_t accum = 0;
    for (uint8_t *ptr = data; ptr < data + len; ptr++) {
        accum += *ptr;
    }
    return accum & 0xFF;
}