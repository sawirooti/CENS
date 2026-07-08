#pragma once

#include <Arduino.h>
#include "I2C.h"

enum class MPU6050RunMode : uint8_t
{
    Continuous,
    FixedRate
};

enum class MPU6050GyroUnit : uint8_t
{
    DegreesPerSecond,
    RadiansPerSecond
};

class MPU6050
{
public:
    enum GyroRange
    {
        GYRO_RANGE_250_DPS = 0,
        GYRO_RANGE_500_DPS = 1,
        GYRO_RANGE_1000_DPS = 2,
        GYRO_RANGE_2000_DPS = 3
    };

    static const uint8_t DEFAULT_ADDRESS = 0x68;
    static const uint8_t ALTERNATIVE_ADDRESS = 0x69;

    explicit MPU6050(uint8_t address = DEFAULT_ADDRESS, TwoWire *wire = &Wire);

    bool begin(MPU6050RunMode mode = MPU6050RunMode::Continuous,
               float sampleRateHz = 0.0f,
               MPU6050GyroUnit unit = MPU6050GyroUnit::DegreesPerSecond);
    bool begin(TwoWire *wire,
               GyroRange gyroRange = GYRO_RANGE_250_DPS,
               uint8_t address = DEFAULT_ADDRESS);

    void setRunMode(MPU6050RunMode mode, float sampleRateHz = 0.0f);
    void setGyroUnit(MPU6050GyroUnit unit);
    void setWire(TwoWire *wire);
    bool setGyroRange(GyroRange gyroRange);
    void setGyroZOffset(float offsetDegreesPerSecond);
    float getGyroZOffset() const;

    bool update();

    bool readGyroZDegreesPerSecond(float *gyroZ);
    bool readGyroZRadiansPerSecond(float *gyroZ);
    bool readGyroZ(float *gyroZ);

    bool available() const;
    float lastGyroZDegreesPerSecond() const;
    float lastGyroZRadiansPerSecond() const;
    uint8_t lastError() const;
    bool isInitialized() const;
    uint8_t getLastStatus() const;

private:
    static const uint8_t REG_SMPLRT_DIV = 0x19;
    static const uint8_t REG_CONFIG = 0x1A;
    static const uint8_t REG_GYRO_CONFIG = 0x1B;
    static const uint8_t REG_GYRO_ZOUT_H = 0x47;
    static const uint8_t REG_PWR_MGMT_1 = 0x6B;
    static const uint8_t REG_WHO_AM_I = 0x75;

    bool readRawGyroZ(int16_t *rawZ);
    bool readAndCache();
    bool shouldSample(unsigned long now) const;
    float convertRawToDps(int16_t rawZ) const;
    static float scaleForRange(GyroRange range);

    I2C _i2c;
    uint8_t _address;
    GyroRange _gyroRange;
    float _gyroZOffset;
    MPU6050RunMode _mode;
    MPU6050GyroUnit _unit;
    unsigned long _sampleIntervalMs;
    unsigned long _lastSampleMs;
    float _lastGyroZDps;
    bool _hasSample;
    bool _initialized;
    uint8_t _lastError;
};
