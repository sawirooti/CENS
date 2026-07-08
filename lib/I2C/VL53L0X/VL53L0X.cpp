#include "VL53L0X.h"

#if defined(__AVR__)
#include <avr/pgmspace.h>
#define VL53L0X_TUNING_STORAGE PROGMEM
#define VL53L0X_READ_TUNING_BYTE(value) pgm_read_byte(&(value))
#else
#define VL53L0X_TUNING_STORAGE
#define VL53L0X_READ_TUNING_BYTE(value) (value)
#endif

VL53L0X::VL53L0X(uint8_t address, TwoWire *wire)
    : _i2c(wire),
      _address(address),
      _mode(VL53L0XRunMode::Continuous),
      _sampleIntervalMs(0),
      _lastSampleMs(0),
      _lastDistanceMm(0),
      _hasSample(false),
      _lastDistanceIsValid(false),
      _initialized(false),
      _timeoutMs(100),
      _timeoutOccurred(false),
      _stopVariable(0),
      _lastError(0)
{
}

bool VL53L0X::begin(VL53L0XRunMode mode, float sampleRateHz)
{
    _i2c.begin();
    setRunMode(mode, sampleRateHz);

    if (!_i2c.isDeviceConnected(_address))
    {
        _lastError = 4;
        _initialized = false;
        return false;
    }

    if (!initSensor())
    {
        _initialized = false;
        return false;
    }

    const uint16_t periodMs = (mode == VL53L0XRunMode::FixedRate && _sampleIntervalMs > 0)
                                  ? (uint16_t)_sampleIntervalMs
                                  : 0;

    if (!startContinuous(periodMs))
    {
        _initialized = false;
        return false;
    }

    _initialized = true;
    return readAndCache();
}

void VL53L0X::setRunMode(VL53L0XRunMode mode, float sampleRateHz)
{
    _mode = mode;
    if (mode == VL53L0XRunMode::FixedRate && sampleRateHz > 0.0f)
    {
        _sampleIntervalMs = (unsigned long)(1000.0f / sampleRateHz);
        if (_sampleIntervalMs == 0)
        {
            _sampleIntervalMs = 1;
        }
    }
    else
    {
        _sampleIntervalMs = 0;
    }
}

void VL53L0X::setWire(TwoWire *wire)
{
    _i2c.setWire(wire);
}

void VL53L0X::setTimeout(uint16_t timeoutMs)
{
    _timeoutMs = timeoutMs;
}

bool VL53L0X::update()
{
    if (_mode == VL53L0XRunMode::Continuous)
    {
        return readAndCache();
    }

    const unsigned long now = millis();
    if (!_hasSample || shouldSample(now))
    {
        return readAndCache();
    }

    return true;
}

bool VL53L0X::readDistanceMillimeters(uint16_t *distanceMm)
{
    if (distanceMm == 0)
    {
        _lastError = 4;
        return false;
    }

    if (_mode == VL53L0XRunMode::Continuous)
    {
        if (!readAndCache())
        {
            return false;
        }
    }
    else if (!_hasSample || shouldSample(millis()))
    {
        if (!readAndCache())
        {
            return false;
        }
    }

    *distanceMm = _lastDistanceMm;
    return _lastDistanceIsValid;
}

bool VL53L0X::readDistanceMeters(float *distanceM)
{
    if (distanceM == 0)
    {
        _lastError = 4;
        return false;
    }

    uint16_t distanceMm = 0;
    const bool valid = readDistanceMillimeters(&distanceMm);
    *distanceM = (float)distanceMm / 1000.0f;
    return valid;
}

bool VL53L0X::available() const
{
    return _hasSample;
}

uint16_t VL53L0X::lastDistanceMillimeters() const
{
    return _lastDistanceMm;
}

bool VL53L0X::lastDistanceValid() const
{
    return _lastDistanceIsValid;
}

bool VL53L0X::timeoutOccurred() const
{
    return _timeoutOccurred;
}

uint8_t VL53L0X::lastError() const
{
    return _lastError;
}

uint16_t VL53L0X::getDistanceMillimeters() const
{
    return _lastDistanceMm;
}

bool VL53L0X::isInitialized() const
{
    return _initialized;
}

bool VL53L0X::initSensor()
{
    uint8_t modelId = 0;
    _lastError = read8(REG_IDENTIFICATION_MODEL_ID, &modelId);
    if (_lastError != 0 || modelId != 0xEE)
    {
        _lastError = _lastError == 0 ? 4 : _lastError;
        return false;
    }

    if (write8(0x88, 0x00) != 0 ||
        write8(0x80, 0x01) != 0 ||
        write8(0xFF, 0x01) != 0 ||
        write8(0x00, 0x00) != 0 ||
        read8(0x91, &_stopVariable) != 0 ||
        write8(0x00, 0x01) != 0 ||
        write8(0xFF, 0x00) != 0 ||
        write8(0x80, 0x00) != 0)
    {
        return false;
    }

    uint8_t msrc = 0;
    if (read8(0x60, &msrc) != 0 || write8(0x60, msrc | 0x12) != 0)
    {
        return false;
    }

    // 0.25 MCPS оставляет запас по темным поверхностям в рабочей зоне до 1.5 м.
    if (write16(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32) != 0)
    {
        return false;
    }

    if (write8(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF) != 0)
    {
        return false;
    }

    uint8_t spadCount = 0;
    bool spadTypeIsAperture = false;
    if (!getSpadInfo(&spadCount, &spadTypeIsAperture) ||
        !configureSpads(spadCount, spadTypeIsAperture) ||
        !applyDefaultTuning())
    {
        return false;
    }

    uint8_t gpio = 0;
    if (write8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04) != 0 ||
        read8(REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio) != 0 ||
        write8(REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio & ~0x10) != 0 ||
        write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != 0)
    {
        return false;
    }

    if (write8(REG_SYSTEM_SEQUENCE_CONFIG, 0x01) != 0 ||
        !performSingleRefCalibration(0x40) ||
        write8(REG_SYSTEM_SEQUENCE_CONFIG, 0x02) != 0 ||
        !performSingleRefCalibration(0x00) ||
        write8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8) != 0)
    {
        return false;
    }

    return true;
}

bool VL53L0X::getSpadInfo(uint8_t *count, bool *typeIsAperture)
{
    if (count == 0 || typeIsAperture == 0)
    {
        _lastError = 4;
        return false;
    }

    if (write8(0x80, 0x01) != 0 ||
        write8(0xFF, 0x01) != 0 ||
        write8(0x00, 0x00) != 0 ||
        write8(0xFF, 0x06) != 0)
    {
        return false;
    }

    uint8_t value = 0;
    if (read8(0x83, &value) != 0 ||
        write8(0x83, value | 0x04) != 0 ||
        write8(0xFF, 0x07) != 0 ||
        write8(0x81, 0x01) != 0 ||
        write8(0x80, 0x01) != 0 ||
        write8(0x94, 0x6B) != 0 ||
        write8(0x83, 0x00) != 0)
    {
        return false;
    }

    if (!waitForRegisterBit(0x83, 0xFF, true))
    {
        return false;
    }

    if (write8(0x83, 0x01) != 0 ||
        read8(0x92, &value) != 0)
    {
        return false;
    }

    *count = value & 0x7F;
    *typeIsAperture = (value & 0x80) != 0;

    return write8(0x81, 0x00) == 0 &&
           write8(0xFF, 0x06) == 0 &&
           read8(0x83, &value) == 0 &&
           write8(0x83, value & ~0x04) == 0 &&
           write8(0xFF, 0x01) == 0 &&
           write8(0x00, 0x01) == 0 &&
           write8(0xFF, 0x00) == 0 &&
           write8(0x80, 0x00) == 0;
}

bool VL53L0X::configureSpads(uint8_t spadCount, bool spadTypeIsAperture)
{
    uint8_t refSpadMap[6] = {0, 0, 0, 0, 0, 0};
    if (readBlock(0xB0, refSpadMap, 6) != 0)
    {
        return false;
    }

    const uint8_t firstSpadToEnable = spadTypeIsAperture ? 12 : 0;
    uint8_t enabledSpads = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < firstSpadToEnable || enabledSpads == spadCount)
        {
            refSpadMap[i / 8] &= ~(1 << (i % 8));
        }
        else if ((refSpadMap[i / 8] >> (i % 8)) & 0x01)
        {
            enabledSpads++;
        }
    }

    return _i2c.writeRegisters(_address, 0xB0, refSpadMap, 6) == 0;
}

bool VL53L0X::applyDefaultTuning()
{
    static const uint8_t settings[][2] VL53L0X_TUNING_STORAGE = {
        {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
        {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
        {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
        {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
        {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
        {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
        {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
        {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
        {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
        {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
        {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
        {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
        {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
        {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
        {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
        {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
        {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
        {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
        {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
        {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}};

    for (uint8_t i = 0; i < sizeof(settings) / sizeof(settings[0]); i++)
    {
        const uint8_t reg = VL53L0X_READ_TUNING_BYTE(settings[i][0]);
        const uint8_t value = VL53L0X_READ_TUNING_BYTE(settings[i][1]);
        if (write8(reg, value) != 0)
        {
            return false;
        }
    }

    return true;
}

bool VL53L0X::performSingleRefCalibration(uint8_t vhvInitByte)
{
    if (write8(REG_SYSRANGE_START, 0x01 | vhvInitByte) != 0)
    {
        return false;
    }

    if (!waitForRegisterBit(REG_RESULT_INTERRUPT_STATUS, 0x07, true))
    {
        return false;
    }

    return write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) == 0 &&
           write8(REG_SYSRANGE_START, 0x00) == 0;
}

bool VL53L0X::startContinuous(uint16_t periodMs)
{
    if (write8(0x80, 0x01) != 0 ||
        write8(0xFF, 0x01) != 0 ||
        write8(0x00, 0x00) != 0 ||
        write8(0x91, _stopVariable) != 0 ||
        write8(0x00, 0x01) != 0 ||
        write8(0xFF, 0x00) != 0 ||
        write8(0x80, 0x00) != 0)
    {
        return false;
    }

    if (periodMs > 0)
    {
        uint16_t oscCalibrate = 0;
        if (read16(REG_OSC_CALIBRATE_VAL, &oscCalibrate) != 0)
        {
            return false;
        }
        uint32_t periodRegister = periodMs;
        if (oscCalibrate != 0)
        {
            periodRegister *= oscCalibrate;
        }

        return write32(REG_SYSTEM_INTERMEASUREMENT_PERIOD, periodRegister) == 0 &&
               write8(REG_SYSRANGE_START, 0x04) == 0;
    }

    return write8(REG_SYSRANGE_START, 0x02) == 0;
}

bool VL53L0X::readContinuous(uint16_t *distanceMm)
{
    if (distanceMm == 0)
    {
        _lastError = 4;
        return false;
    }

    if (!waitForRegisterBit(REG_RESULT_INTERRUPT_STATUS, 0x07, true))
    {
        return false;
    }

    uint16_t range = 0;
    if (read16(REG_RESULT_RANGE_STATUS + 10, &range) != 0 ||
        write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != 0)
    {
        return false;
    }

    *distanceMm = range;
    return true;
}

bool VL53L0X::readAndCache()
{
    uint16_t distance = 0;
    if (!readContinuous(&distance))
    {
        return false;
    }

    _lastError = 0;
    _lastDistanceMm = distance;
    _lastDistanceIsValid = true;
    _lastSampleMs = millis();
    _hasSample = true;
    return true;
}

bool VL53L0X::shouldSample(unsigned long now) const
{
    return _sampleIntervalMs == 0 || (unsigned long)(now - _lastSampleMs) >= _sampleIntervalMs;
}

bool VL53L0X::waitForRegisterBit(uint8_t reg, uint8_t mask, bool set)
{
    const unsigned long start = millis();
    _timeoutOccurred = false;

    while (true)
    {
        uint8_t value = 0;
        if (read8(reg, &value) != 0)
        {
            return false;
        }

        const bool matched = (value & mask) != 0;
        if (matched == set)
        {
            return true;
        }

        if (_timeoutMs > 0 && (unsigned long)(millis() - start) > _timeoutMs)
        {
            _timeoutOccurred = true;
            _lastError = 4;
            return false;
        }
    }
}

uint8_t VL53L0X::write8(uint8_t reg, uint8_t value)
{
    _lastError = _i2c.writeRegister(_address, reg, value);
    return _lastError;
}

uint8_t VL53L0X::write16(uint8_t reg, uint16_t value)
{
    uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    _lastError = _i2c.writeRegisters(_address, reg, data, 2);
    return _lastError;
}

uint8_t VL53L0X::write32(uint8_t reg, uint32_t value)
{
    uint8_t data[4] = {
        (uint8_t)(value >> 24),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF)};
    _lastError = _i2c.writeRegisters(_address, reg, data, 4);
    return _lastError;
}

uint8_t VL53L0X::read8(uint8_t reg, uint8_t *value)
{
    _lastError = _i2c.readRegister(_address, reg, value);
    return _lastError;
}

uint8_t VL53L0X::read16(uint8_t reg, uint16_t *value)
{
    if (value == 0)
    {
        _lastError = 4;
        return _lastError;
    }

    uint8_t data[2] = {0, 0};
    _lastError = _i2c.readRegisters(_address, reg, data, 2);
    if (_lastError == 0)
    {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    return _lastError;
}

uint8_t VL53L0X::readBlock(uint8_t reg, uint8_t *data, uint8_t length)
{
    _lastError = _i2c.readRegisters(_address, reg, data, length);
    return _lastError;
}
