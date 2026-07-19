#include "MPU6050.h"

namespace
{
const float GYRO_SCALE_250_DPS = 131.0f;
}

MPU6050::MPU6050(uint8_t address, TwoWire *wire)
    : _i2c(wire),
      _address(address),
      _lastError(0)
{
}

bool MPU6050::begin()
{
    _i2c.begin();

    if (!_i2c.isDeviceConnected(_address))
    {
        _lastError = 4;
        return false;
    }

    uint8_t whoAmI = 0;
    _lastError = _i2c.readRegister(_address, REG_WHO_AM_I, &whoAmI);
    const bool supportedDevice =
        (whoAmI & 0x7E) == 0x68 || whoAmI == 0x70;
    if (_lastError != 0 || !supportedDevice)
    {
        if (_lastError == 0)
        {
            _lastError = 4;
        }
        return false;
    }

    _lastError = _i2c.writeRegister(_address, REG_PWR_MGMT_1, 0x00);
    if (_lastError != 0)
    {
        return false;
    }
    delay(100);

    // Фильтр 44 Гц уменьшает шум гироскопа.
    _lastError = _i2c.writeRegister(_address, REG_CONFIG, 0x03);
    if (_lastError != 0)
    {
        return false;
    }

    // Диапазон ±250 град/с, чувствительность 131 LSB/(град/с).
    _lastError = _i2c.writeRegister(_address, REG_GYRO_CONFIG, 0x00);
    if (_lastError != 0)
    {
        return false;
    }

    // При включённом фильтре внутренняя частота измерений равна 1 кГц.
    _lastError = _i2c.writeRegister(_address, REG_SMPLRT_DIV, 0x00);
    return _lastError == 0;
}

bool MPU6050::readGyroZDegreesPerSecond(float *gyroZ)
{
    if (gyroZ == 0)
    {
        _lastError = 4;
        return false;
    }

    int16_t rawZ = 0;
    _lastError = _i2c.readInt16BE(_address, REG_GYRO_ZOUT_H, &rawZ);
    if (_lastError != 0)
    {
        return false;
    }

    *gyroZ = (float)rawZ / GYRO_SCALE_250_DPS;
    return true;
}

uint8_t MPU6050::lastError() const
{
    return _lastError;
}
