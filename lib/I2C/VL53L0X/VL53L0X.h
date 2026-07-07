#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../I2C.h"

// Минимальная библиотека для датчика дальности VL53L0X.
// Оставлено только то, что нужно проекту: измерение дальности по запросу
// и получение результата в миллиметрах.
class VL53L0X
{
public:
    VL53L0X() = default;

    // Инициализация датчика.
    bool begin();

    // Выполняет одно измерение и возвращает дальность в миллиметрах.
    bool readDistanceMillimeters(uint16_t *distanceMm);

    // Обновляет сохраненное значение дальности. Оставлено для простого старого стиля вызова.
    bool update();

    // Последнее успешно прочитанное значение дальности в миллиметрах.
    uint16_t getDistanceMillimeters() const;
    bool isInitialized() const;

private:
    // В этом проекте адрес и базовые ограничения фиксированы, чтобы библиотека была маленькой.
    static const uint8_t ADDRESS = 0x29;
    static const uint16_t TIMEOUT_MS = 100;
    static const uint8_t EXPECTED_MODEL_ID = 0xEE;

    // Используемые регистры VL53L0X.
    static const uint8_t REG_SYSRANGE_START = 0x00;
    static const uint8_t REG_SYSTEM_SEQUENCE_CONFIG = 0x01;
    static const uint8_t REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    static const uint8_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0B;
    static const uint8_t REG_RESULT_INTERRUPT_STATUS = 0x13;
    static const uint8_t REG_RESULT_RANGE_STATUS = 0x14;
    static const uint8_t REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    static const uint8_t REG_MSRC_CONFIG_TIMEOUT_MACROP = 0x46;
    static const uint8_t REG_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
    static const uint8_t REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
    static const uint8_t REG_MSRC_CONFIG_CONTROL = 0x60;
    static const uint8_t REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
    static const uint8_t REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
    static const uint8_t REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    static const uint8_t REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89;
    static const uint8_t REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
    static const uint8_t REG_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
    static const uint8_t REG_IDENTIFICATION_MODEL_ID = 0xC0;
    static const uint8_t REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
    static const uint8_t REG_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;

    // Включенные шаги внутренней измерительной последовательности датчика.
    struct SequenceStepEnables
    {
        bool tcc;
        bool msrc;
        bool dss;
        bool preRange;
        bool finalRange;
    };

    // Таймауты отдельных шагов измерения.
    struct SequenceStepTimeouts
    {
        uint16_t preRangeVcselPeriodPclks;
        uint16_t finalRangeVcselPeriodPclks;
        uint16_t msrcDssTccMclks;
        uint16_t preRangeMclks;
        uint16_t finalRangeMclks;
        uint32_t msrcDssTccUs;
        uint32_t preRangeUs;
        uint32_t finalRangeUs;
    };

    // Внутренняя инициализация, настройка таймингов и одиночное измерение.
    bool initializeSensor();
    bool setMeasurementTimingBudget(uint32_t budgetUs);
    bool startSingleMeasurement();
    uint16_t readRangeMillimeters();

    // Низкоуровневое чтение/запись регистров датчика.
    uint8_t readRegister(uint8_t reg);
    uint16_t readRegister16(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t *data, uint8_t length);

    void writeRegister(uint8_t reg, uint8_t value);
    void writeRegister16(uint8_t reg, uint16_t value);
    void writeRegisters(uint8_t reg, const uint8_t *data, uint8_t length);

    // Контроль зависания при ожидании готовности измерения.
    bool timeoutOccurred();
    void startTimeout();
    bool checkTimeoutExpired() const;

    // Служебные функции инициализации VL53L0X.
    bool getSpadInfo(uint8_t *count, bool *typeIsAperture);
    void getSequenceStepEnables(SequenceStepEnables *enables);
    void getSequenceStepTimeouts(const SequenceStepEnables *enables, SequenceStepTimeouts *timeouts);
    bool performSingleRefCalibration(uint8_t vhvInitByte);

    // Перевод таймаутов между форматом регистров, MCLK и микросекундами.
    static uint16_t decodeTimeout(uint16_t value);
    static uint16_t encodeTimeout(uint32_t timeoutMclks);
    static uint32_t timeoutMclksToMicroseconds(uint16_t timeoutPeriodMclks, uint8_t vcselPeriodPclks);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeoutPeriodUs, uint8_t vcselPeriodPclks);
    static uint8_t decodeVcselPeriod(uint8_t value);
    static uint32_t calcMacroPeriod(uint8_t vcselPeriodPclks);

    I2C _i2c;
    uint8_t _lastStatus = 0;
    uint32_t _timeoutStartMs = 0;
    bool _didTimeout = false;

    uint8_t _stopVariable = 0;
    uint16_t _distanceMm = 0;
    bool _initialized = false;
};
