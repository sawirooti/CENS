#pragma once

#include <Arduino.h>
#include "I2C.h"

class MPU6050
{
public:
    static const uint8_t DEFAULT_ADDRESS = 0x68;

    explicit MPU6050(uint8_t address = DEFAULT_ADDRESS, TwoWire *wire = &Wire);

    bool begin();
    bool readGyroZDegreesPerSecond(float *gyroZ);
    uint8_t lastError() const;

private:
    static const uint8_t REG_SMPLRT_DIV = 0x19;
    static const uint8_t REG_CONFIG = 0x1A;
    static const uint8_t REG_GYRO_CONFIG = 0x1B;
    static const uint8_t REG_GYRO_ZOUT_H = 0x47;
    static const uint8_t REG_PWR_MGMT_1 = 0x6B;
    static const uint8_t REG_WHO_AM_I = 0x75;

    I2C _i2c;
    uint8_t _address;
    uint8_t _lastError;
};
