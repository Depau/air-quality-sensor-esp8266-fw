//
// Created by depau on 5/5/21.
//

#ifndef AIR_SENSORS_SENDER_HOMIELOGGER_H
#define AIR_SENSORS_SENDER_HOMIELOGGER_H


#include <Stream.h>
#include <HomieNode.h>

class HomieLogger : public Stream {
protected:
    HomieProperty *_homieProp;
    Stream *_serial;
    String _printBuf = String();

public:
    HomieLogger(Stream *serial, HomieProperty *homieProperty) : _homieProp{homieProperty}, _serial{serial} {};

    size_t write(const uint8_t *buffer, size_t size) override {
        size_t ret = 0;
        bool bufInHeap = false;

        if (_printBuf.length() > 0) {
            bufInHeap = true;
            size_t len = _printBuf.length() + 1;
            uint8_t *buf = static_cast<uint8_t *>(malloc((len + size) * sizeof(char)));

            _printBuf.toCharArray(reinterpret_cast<char *>(buf), len);
            memcpy(buf + _printBuf.length(), buffer, size);
            buf[_printBuf.length() + size] = 0;

            buffer = buf;
            size += _printBuf.length();
            _printBuf = String();
        }

        if (size > 0) {
            if (_serial != nullptr) {
                ret = _serial->write(buffer, size);
            }
            if (_homieProp != nullptr) {
                _homieProp->SetValue(String((char *) buffer));
                if (ret != 0) {
                    ret = size;
                }
            }
        }

        if (bufInHeap) {
            free((void *) buffer);
        }

        return ret;
    }

    void setSerial(Stream *serial) { _serial = serial; };

    void setHomieProp(HomieProperty *prop) { _homieProp = prop; }

    int available() override {
        if (_serial != nullptr) {
            return _serial->available();
        }
        return 0;
    }

    int read() override {
        if (_serial != nullptr) {
            return _serial->read();
        }
        return 0;
    }

    int peek() override {
        if (_serial != nullptr) {
            return _serial->peek();
        }
        return 0;
    };

    size_t write(uint8_t uint81) override {
        _printBuf += (char) uint81;
        if (uint81 == '\n') {
            write((uint8_t *) nullptr, 0);
        }
        return 1;
    }

    using Print::write; // Import other write() methods to support things like write(0) properly
};


extern HomieLogger HLogger;

#endif //AIR_SENSORS_SENDER_HOMIELOGGER_H
