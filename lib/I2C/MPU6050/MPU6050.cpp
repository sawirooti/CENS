#include "MPU6050.h"

MPU6050::MPU6050(uint8_t address) : _address(address)
{
}

bool MPU6050::begin(TwoWire *wire, GyroRange gyroRange, uint8_t address)
{
    if (wire == 0)
    {
        // Без I2C-шины датчик недоступен.
        _lastStatus = 4;
        return false;
    }

    // Привязываем общий I2C-класс к нужной шине и запускаем ее.
    _i2c.setWire(wire);
    _i2c.begin();
    _address = address;
    _initialized = false;

    // Проверяем, что по адресу действительно отвечает MPU6050.
    uint8_t whoAmI = 0;
    _lastStatus = _i2c.readRegister(_address, REG_WHO_AM_I, &whoAmI);
    if (_lastStatus != 0 || whoAmI != WHO_AM_I_VALUE)
    {
        return false;
    }

    // Выводим датчик из сна и выбираем PLL от гироскопа X как источник тактирования.
    _lastStatus = _i2c.writeRegister(_address, REG_PWR_MGMT_1, CLOCK_PLL_XGYRO);
    if (_lastStatus != 0)
    {
        return false;
    }

    delay(100);

    // Включаем цифровой фильтр, чтобы уменьшить шум гироскопа.
    _lastStatus = _i2c.writeRegister(_address, REG_CONFIG, DLPF_44HZ);
    if (_lastStatus != 0)
    {
        return false;
    }

    if (!setGyroRange(gyroRange))
    {
        return false;
    }

    _initialized = true;
    return true;
}

bool MPU6050::setGyroRange(GyroRange gyroRange)
{
    if (gyroRange < GYRO_RANGE_250_DPS || gyroRange > GYRO_RANGE_2000_DPS)
    {
        // Значение вне enum считаем ошибкой настройки.
        _lastStatus = 4;
        return false;
    }

    // Поле FS_SEL находится в битах 4:3 регистра GYRO_CONFIG.
    uint8_t configValue = ((uint8_t)gyroRange) << 3;
    _lastStatus = _i2c.writeRegister(_address, REG_GYRO_CONFIG, configValue);
    if (_lastStatus != 0)
    {
        return false;
    }

    _gyroRange = gyroRange;
    return true;
}

bool MPU6050::readRawGyroZ(int16_t *rawZ)
{
    if (!_initialized || rawZ == 0)
    {
        // Нельзя читать до успешной инициализации или в нулевой указатель.
        _lastStatus = 4;
        return false;
    }

    // Значение Z хранится в двух соседних регистрах, старший байт идет первым.
    _lastStatus = _i2c.readInt16BE(_address, REG_GYRO_ZOUT_H, rawZ);
    return _lastStatus == 0;
}

bool MPU6050::readGyroZDegreesPerSecond(float *gyroZ)
{
    if (gyroZ == 0)
    {
        _lastStatus = 4;
        return false;
    }

    int16_t rawZ = 0;
    if (!readRawGyroZ(&rawZ))
    {
        return false;
    }

    // Переводим сырое значение в градусы/с и вычитаем калибровочное смещение.
    *gyroZ = ((float)rawZ / getGyroScale()) - _gyroZOffset;
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

bool MPU6050::isInitialized() const
{
    return _initialized;
}

uint8_t MPU6050::getLastStatus() const
{
    return _lastStatus;
}

float MPU6050::getGyroScale() const
{
    // Чувствительность из документации MPU6050: LSB на 1 градус/с.
    switch (_gyroRange)
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
        return 131.0f;
    }
}
