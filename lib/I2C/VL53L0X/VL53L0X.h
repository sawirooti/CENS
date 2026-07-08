#pragma once

#include <Arduino.h>
#include "I2C.h"

enum class VL53L0XRunMode : uint8_t
{
    Continuous,
    FixedRate
};

class VL53L0X
{
public:
    static const uint8_t DEFAULT_ADDRESS = 0x29;

    explicit VL53L0X(uint8_t address = DEFAULT_ADDRESS, TwoWire *wire = &Wire);

    bool begin(VL53L0XRunMode mode = VL53L0XRunMode::Continuous,
               float sampleRateHz = 0.0f);

    void setRunMode(VL53L0XRunMode mode, float sampleRateHz = 0.0f);
    void setWire(TwoWire *wire);
    void setTimeout(uint16_t timeoutMs);

    bool update();

    bool readDistanceMillimeters(uint16_t *distanceMm);
    bool readDistanceMeters(float *distanceM);

    bool available() const;
    uint16_t lastDistanceMillimeters() const;
    bool lastDistanceValid() const;
    bool timeoutOccurred() const;
    uint8_t lastError() const;
    uint16_t getDistanceMillimeters() const;
    bool isInitialized() const;

private:
    static const uint8_t REG_SYSRANGE_START = 0x00;
    static const uint8_t REG_SYSTEM_SEQUENCE_CONFIG = 0x01;
    static const uint8_t REG_SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
    static const uint8_t REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    static const uint8_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0B;
    static const uint8_t REG_RESULT_INTERRUPT_STATUS = 0x13;
    static const uint8_t REG_RESULT_RANGE_STATUS = 0x14;
    static const uint8_t REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    static const uint8_t REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    static const uint8_t REG_IDENTIFICATION_MODEL_ID = 0xC0;
    static const uint8_t REG_OSC_CALIBRATE_VAL = 0xF8;

    bool initSensor();
    bool getSpadInfo(uint8_t *count, bool *typeIsAperture);
    bool configureSpads(uint8_t spadCount, bool spadTypeIsAperture);
    bool applyDefaultTuning();
    bool performSingleRefCalibration(uint8_t vhvInitByte);
    bool startContinuous(uint16_t periodMs);
    bool readContinuous(uint16_t *distanceMm);
    bool readAndCache();
    bool shouldSample(unsigned long now) const;
    bool waitForRegisterBit(uint8_t reg, uint8_t mask, bool set);

    uint8_t write8(uint8_t reg, uint8_t value);
    uint8_t write16(uint8_t reg, uint16_t value);
    uint8_t write32(uint8_t reg, uint32_t value);
    uint8_t read8(uint8_t reg, uint8_t *value);
    uint8_t read16(uint8_t reg, uint16_t *value);
    uint8_t readBlock(uint8_t reg, uint8_t *data, uint8_t length);

    I2C _i2c;
    uint8_t _address;
    VL53L0XRunMode _mode;
    unsigned long _sampleIntervalMs;
    unsigned long _lastSampleMs;
    uint16_t _lastDistanceMm;
    bool _hasSample;
    bool _lastDistanceIsValid;
    bool _initialized;
    uint16_t _timeoutMs;
    bool _timeoutOccurred;
    uint8_t _stopVariable;
    uint8_t _lastError;
};
