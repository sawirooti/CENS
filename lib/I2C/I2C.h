#pragma once

#include <Arduino.h>
#include <Wire.h>

// Небольшая обертка над Arduino Wire для работы с I2C-устройствами.
// Возвращаемые статусы совпадают со статусами Wire.endTransmission():
// 0 — успех, ненулевое значение — ошибка обмена.
class I2C
{
public:
    // По умолчанию используется стандартная шина Wire.
    explicit I2C(TwoWire *wire = &Wire);

    // Инициализация выбранной I2C-шины.
    void begin();

    // Замена используемой I2C-шины, если датчик подключен не к Wire.
    void setWire(TwoWire *wire);
    TwoWire *getWire() const;

    // Быстрая проверка, отвечает ли устройство по указанному адресу.
    bool isDeviceConnected(uint8_t address) const;

    // Запись одного байта или блока байт, начиная с указанного регистра.
    uint8_t writeRegister(uint8_t address, uint8_t reg, uint8_t value) const;
    uint8_t writeRegisters(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length) const;

    // Чтение одного байта, блока байт или 16-битного значения в формате Big Endian.
    uint8_t readRegister(uint8_t address, uint8_t reg, uint8_t *value) const;
    uint8_t readRegisters(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length) const;
    uint8_t readInt16BE(uint8_t address, uint8_t reg, int16_t *value) const;

    // Изменение только выбранного диапазона битов внутри регистра.
    uint8_t writeBits(uint8_t address,
                      uint8_t reg,
                      uint8_t bitStart,
                      uint8_t length,
                      uint8_t value) const;

    // Статические версии нужны, когда удобнее передать Wire явно.
    static uint8_t writeRegister(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t value);
    static uint8_t writeRegisters(TwoWire *wire, uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length);

    static uint8_t readRegister(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t *value);
    static uint8_t readRegisters(TwoWire *wire, uint8_t address, uint8_t reg, uint8_t *data, uint8_t length);
    static uint8_t readInt16BE(TwoWire *wire, uint8_t address, uint8_t reg, int16_t *value);

    static uint8_t writeBits(TwoWire *wire,
                             uint8_t address,
                             uint8_t reg,
                             uint8_t bitStart,
                             uint8_t length,
                             uint8_t value);

private:
    TwoWire *_wire;
};
