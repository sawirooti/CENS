#include "MPU6050.h"

static const float MPU6050_DEG_TO_RAD = 0.017453292519943295f;
static const float MPU6050_GYRO_250DPS_SCALE = 131.0f;

MPU6050::MPU6050(uint8_t address, TwoWire *wire)
    : _i2c(wire),
      _address(address),
      _gyroRange(GYRO_RANGE_250_DPS),
      _gyroZOffset(0.0f),
      _mode(MPU6050RunMode::Continuous),
      _unit(MPU6050GyroUnit::DegreesPerSecond),
      _sampleIntervalMs(0),
      _lastSampleMs(0),
      _lastGyroZDps(0.0f),
      _hasSample(false),
      _initialized(false),
      _lastError(0)
{
}

bool MPU6050::begin(MPU6050RunMode mode, float sampleRateHz, MPU6050GyroUnit unit)
{
    _i2c.begin();
    _mode = mode;
    _unit = unit;
    setRunMode(mode, sampleRateHz);

    if (!_i2c.isDeviceConnected(_address))
    {
        _lastError = 4;
        _initialized = false;
        return false;
    }

    uint8_t whoAmI = 0;
    _lastError = _i2c.readRegister(_address, REG_WHO_AM_I, &whoAmI);
    if (_lastError != 0)
    {
        return false;
    }

    // У разных модулей AD0 может быть подтянут по-разному; сравниваем только ID датчика.
    if ((whoAmI & 0x7E) != 0x68)
    {
        _lastError = 4;
        _initialized = false;
        return false;
    }

    _lastError = _i2c.writeRegister(_address, REG_PWR_MGMT_1, 0x00);
    if (_lastError != 0)
    {
        return false;
    }
    delay(100);

    // DLPF около 44 Гц для гироскопа: меньше шума, без лишней задержки для маленькой машины.
    _lastError = _i2c.writeRegister(_address, REG_CONFIG, 0x03);
    if (_lastError != 0)
    {
        return false;
    }

    if (!setGyroRange(_gyroRange))
    {
        return false;
    }

    // Внутренняя частота при включенном DLPF равна 1 кГц.
    _lastError = _i2c.writeRegister(_address, REG_SMPLRT_DIV, 0x00);
    if (_lastError != 0)
    {
        return false;
    }

    _initialized = true;
    return readAndCache();
}

bool MPU6050::begin(TwoWire *wire, GyroRange gyroRange, uint8_t address)
{
    _i2c.setWire(wire);
    _address = address;
    _gyroRange = gyroRange;
    const float sampleRateHz = _sampleIntervalMs > 0 ? 1000.0f / (float)_sampleIntervalMs : 0.0f;
    return begin(_mode, sampleRateHz, _unit);
}

void MPU6050::setRunMode(MPU6050RunMode mode, float sampleRateHz)
{
    _mode = mode;
    if (mode == MPU6050RunMode::FixedRate && sampleRateHz > 0.0f)
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

void MPU6050::setGyroUnit(MPU6050GyroUnit unit)
{
    _unit = unit;
}

void MPU6050::setWire(TwoWire *wire)
{
    _i2c.setWire(wire);
}

bool MPU6050::setGyroRange(GyroRange gyroRange)
{
    if (gyroRange < GYRO_RANGE_250_DPS || gyroRange > GYRO_RANGE_2000_DPS)
    {
        _lastError = 4;
        return false;
    }

    _lastError = _i2c.writeRegister(_address, REG_GYRO_CONFIG, ((uint8_t)gyroRange) << 3);
    if (_lastError != 0)
    {
        return false;
    }

    _gyroRange = gyroRange;
    return true;
}

void MPU6050::setGyroZOffset(float offsetDegreesPerSecond)
{
    _gyroZOffset = offsetDegreesPerSecond;
}

float MPU6050::getGyroZOffset() const
{
    return _gyroZOffset;
}

bool MPU6050::update()
{
    if (_mode == MPU6050RunMode::Continuous)
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

bool MPU6050::readGyroZDegreesPerSecond(float *gyroZ)
{
    if (gyroZ == 0)
    {
        _lastError = 4;
        return false;
    }

    if (_mode == MPU6050RunMode::Continuous)
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

    *gyroZ = _lastGyroZDps;
    return true;
}

bool MPU6050::readGyroZRadiansPerSecond(float *gyroZ)
{
    if (gyroZ == 0)
    {
        _lastError = 4;
        return false;
    }

    float dps = 0.0f;
    if (!readGyroZDegreesPerSecond(&dps))
    {
        return false;
    }

    *gyroZ = dps * MPU6050_DEG_TO_RAD;
    return true;
}

bool MPU6050::readGyroZ(float *gyroZ)
{
    if (_unit == MPU6050GyroUnit::RadiansPerSecond)
    {
        return readGyroZRadiansPerSecond(gyroZ);
    }
    return readGyroZDegreesPerSecond(gyroZ);
}

bool MPU6050::available() const
{
    return _hasSample;
}

float MPU6050::lastGyroZDegreesPerSecond() const
{
    return _lastGyroZDps;
}

float MPU6050::lastGyroZRadiansPerSecond() const
{
    return _lastGyroZDps * MPU6050_DEG_TO_RAD;
}

uint8_t MPU6050::lastError() const
{
    return _lastError;
}

bool MPU6050::isInitialized() const
{
    return _initialized;
}

uint8_t MPU6050::getLastStatus() const
{
    return _lastError;
}

bool MPU6050::readRawGyroZ(int16_t *rawZ)
{
    if (rawZ == 0)
    {
        _lastError = 4;
        return false;
    }

    _lastError = _i2c.readInt16BE(_address, REG_GYRO_ZOUT_H, rawZ);
    return _lastError == 0;
}

bool MPU6050::readAndCache()
{
    int16_t rawZ = 0;
    if (!readRawGyroZ(&rawZ))
    {
        return false;
    }

    _lastGyroZDps = convertRawToDps(rawZ);
    _lastSampleMs = millis();
    _hasSample = true;
    return true;
}

bool MPU6050::shouldSample(unsigned long now) const
{
    return _sampleIntervalMs == 0 || (unsigned long)(now - _lastSampleMs) >= _sampleIntervalMs;
}

float MPU6050::convertRawToDps(int16_t rawZ) const
{
    return ((float)rawZ / scaleForRange(_gyroRange)) - _gyroZOffset;
}

float MPU6050::scaleForRange(GyroRange range)
{
    switch (range)
    {
    case GYRO_RANGE_250_DPS:
        return 131.0f;
    case GYRO_RANGE_500_DPS:
        return 65.5f;
    case GYRO_RANGE_1000_DPS:
        return 32.8f;
    case GYRO_RANGE_2000_DPS:
        return 16.4f;
    default:
        return MPU6050_GYRO_250DPS_SCALE;
    }
}
