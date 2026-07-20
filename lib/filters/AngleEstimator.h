#pragma once

#include <stdint.h>
#include "Defenition.hpp"

class AngleEstimator
{
public:
    explicit AngleEstimator(float initialAngleDeg = PSI_START);

    // Собирает показания неподвижного гироскопа и вычисляет его нулевое смещение.
    // Возвращает true, когда заданное время калибровки закончилось.
    bool calibrate(float gyroZDps,
                   uint32_t nowMs,
                   uint32_t durationMs = 1000,
                   uint16_t sampleRateHz = 100);

    // Интегрирует скорость MPU6050 по оси Z; dtSeconds задаётся в секундах.
    void update(float gyroZDps, float dtSeconds);

    float getAngle() const;
    bool setAngle(float angle);

private:
    static float normalize(float angleDeg);

    float _angleDeg;
    float _gyroBiasDps;
    float _calibrationSum;
    uint32_t _calibrationStartMs;
    uint32_t _lastCalibrationSampleMs;
    uint16_t _calibrationSampleCount;
    bool _calibrationDone;
};
