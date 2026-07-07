#include "I2C.h"

I2C::I2C(TwoWire *wire) : _wire(wire)
{
}

void I2C::begin()
{
    // Wire может быть не задан, поэтому защищаемся от нулевого указателя.
    if (_wire != 0)
    {
        _wire->begin();
    }
}

void I2C::setWire(TwoWire *wire)
{
    _wire = wire;
}

TwoWire *I2C::getWire() const
{
    return _wire;
}

bool I2C::isDeviceConnected(uint8_t address) const
{
    if (_wire == 0)
    {
        return false;
    }

    // Пустая передача показывает, отвечает ли устройство на адрес.
    _wire->beginTransmission(address);
    return _wire->endTransmission() == 0;
}

uint8_t I2C::writeRegister(uint8_t address, uint8_t reg, uint8_t value) const
{
    return writeRegister(_wire, address, reg, value);
}

uint8_t I2C::writeRegisters(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length) const
{
    return writeRegisters(_wire, address, reg, data, length);
}

uint8_t I2C::readRegister(uint8_t address, uint8_t reg, uint8_t *value) const
{
    return readRegister(_wire, address, reg, value);
}

uint8_t I2C::readRegisters(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length) const
{
    return readRegisters(_wire, address, reg, data, length);
}

uint8_t I2C::readInt16BE(uint8_t address, uint8_t reg, int16_t *value) const
{
    return readInt16BE(_wire, address, reg, value);
}

uint8_t I2C::writeBits(uint8_t address,
                       uint8_t reg,
                       uint8_t bitStart,
                       uint8_t length,
                       uint8_t value) const
{
    return writeBits(_wire, address, reg, bitStart, length, value);
}

uint8_t I2C::writeRegister(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t value)
{
    if (wire == 0)
    {
        // 4 используем как общий код ошибки, аналогично Wire.
        return 4;
    }

    // Для записи регистра сначала отправляется адрес регистра, затем значение.
    wire->beginTransmission(address);
    wire->write(reg);
    wire->write(value);
    return wire->endTransmission();
}

uint8_t I2C::writeRegisters(TwoWire *wire, uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length)
{
    if (wire == 0 || (data == 0 && length > 0))
    {
        return 4;
    }

    wire->beginTransmission(address);
    wire->write(reg);

    // Последовательно отправляем все байты данных после адреса регистра.
    for (uint8_t i = 0; i < length; i++)
    {
        wire->write(data[i]);
    }

    return wire->endTransmission();
}

uint8_t I2C::readRegister(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t *value)
{
    return readRegisters(wire, address, reg, value, 1);
}

uint8_t I2C::readRegisters(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (wire == 0 || data == 0 || length == 0)
    {
        return 4;
    }

    wire->beginTransmission(address);
    wire->write(reg);
    uint8_t status = wire->endTransmission();

    if (status != 0)
    {
        return status;
    }

    // После выбора регистра запрашиваем нужное количество байт.
    uint8_t received = wire->requestFrom(address, length);
    for (uint8_t i = 0; i < received && i < length; i++)
    {
        data[i] = wire->read();
    }

    return received == length ? 0 : 4;
}

uint8_t I2C::readInt16BE(TwoWire *wire, uint8_t address, uint8_t reg, int16_t *value)
{
    if (value == 0)
    {
        return 4;
    }

    uint8_t data[2] = {0, 0};
    uint8_t status = readRegisters(wire, address, reg, data, 2);
    if (status != 0)
    {
        return status;
    }

    // Датчики MPU6050 и VL53L0X отдают 16-битные значения старшим байтом вперед.
    *value = ((int16_t)data[0] << 8) | data[1];
    return 0;
}

uint8_t I2C::writeBits(TwoWire *wire,
                       uint8_t address,
                       uint8_t reg,
                       uint8_t bitStart,
                       uint8_t length,
                       uint8_t value)
{
    if (length == 0 || length > 8 || bitStart > 7 || bitStart + 1 < length)
    {
        return 4;
    }

    uint8_t currentValue = 0;
    uint8_t status = readRegister(wire, address, reg, &currentValue);
    if (status != 0)
    {
        return status;
    }

    uint8_t shift = bitStart - length + 1;
    uint8_t mask = ((1 << length) - 1) << shift;

    // Сохраняем остальные биты регистра и заменяем только выбранное поле.
    value = (value << shift) & mask;
    currentValue = (currentValue & ~mask) | value;

    return writeRegister(wire, address, reg, currentValue);
}
