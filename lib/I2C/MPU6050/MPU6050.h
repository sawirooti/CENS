#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../I2C.h"

// Минимальная библиотека для MPU6050.
// В проекте используется только гироскоп по оси Z, поэтому акселерометр
// и остальные оси намеренно не вынесены в интерфейс.
class MPU6050
{
public:
    // Диапазон измерения гироскопа. Чем больше диапазон, тем меньше чувствительность.
    enum GyroRange
    {
        GYRO_RANGE_250_DPS = 0,
        GYRO_RANGE_500_DPS = 1,
        GYRO_RANGE_1000_DPS = 2,
        GYRO_RANGE_2000_DPS = 3
    };

    explicit MPU6050(uint8_t address = DEFAULT_ADDRESS);

    // Инициализация датчика, проверка WHO_AM_I и настройка фильтра/гироскопа.
    bool begin(TwoWire *wire = &Wire,
               GyroRange gyroRange = GYRO_RANGE_250_DPS,
               uint8_t address = DEFAULT_ADDRESS);

    // Настройка диапазона гироскопа.
    bool setGyroRange(GyroRange gyroRange);

    // Чтение сырого значения и значения в градусах в секунду по оси Z.
    bool readRawGyroZ(int16_t *rawZ);
    bool readGyroZDegreesPerSecond(float *gyroZ);

    // Смещение нуля гироскопа по Z. Вычитается из результата в dps.
    void setGyroZOffset(float offsetDegreesPerSecond);
    float getGyroZOffset() const;

    // Состояние датчика и последний статус обмена по I2C.
    bool isInitialized() const;
    uint8_t getLastStatus() const;

    // Основной и альтернативный адреса MPU6050 зависят от уровня на пине AD0.
    static const uint8_t DEFAULT_ADDRESS = 0x68;
    static const uint8_t ALTERNATIVE_ADDRESS = 0x69;

private:
    // Используемые регистры MPU6050.
    static const uint8_t REG_CONFIG = 0x1A;
    static const uint8_t REG_GYRO_CONFIG = 0x1B;
    static const uint8_t REG_GYRO_ZOUT_H = 0x47;
    static const uint8_t REG_PWR_MGMT_1 = 0x6B;
    static const uint8_t REG_WHO_AM_I = 0x75;

    static const uint8_t CLOCK_PLL_XGYRO = 0x01;
    static const uint8_t DLPF_44HZ = 0x03;
    static const uint8_t WHO_AM_I_VALUE = 0x68;

    // Коэффициент перевода сырого значения гироскопа в градусы в секунду.
    float getGyroScale() const;

    I2C _i2c;
    uint8_t _address;
    GyroRange _gyroRange = GYRO_RANGE_250_DPS;
    float _gyroZOffset = 0.0f;
    bool _initialized = false;
    uint8_t _lastStatus = 0;
};
