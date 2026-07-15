#include "AngleEstimator.h"

#include <math.h>

AngleEstimator::AngleEstimator(float initialAngleDeg)
    : _angleDeg(normalize(initialAngleDeg)),
      _gyroBiasDps(0.0f),
      _calibrationSum(0.0f),
      _calibrationStartMs(0),
      _lastCalibrationSampleMs(0),
      _calibrationSampleCount(0),
      _calibrationDone(false)
{
}

bool AngleEstimator::calibrate(float gyroZDps,
                               uint32_t nowMs,
                               uint32_t durationMs,
                               uint16_t sampleRateHz)
{
    if (_calibrationDone)
    {
        return true;
    }
    if (durationMs == 0 || sampleRateHz == 0)
    {
        return false;
    }

    uint32_t sampleIntervalMs = 1000UL / sampleRateHz;
    if (sampleIntervalMs == 0)
    {
        sampleIntervalMs = 1;
    }

    if (_calibrationSampleCount == 0)
    {
        _calibrationStartMs = nowMs;
        _lastCalibrationSampleMs = nowMs;
        _calibrationSum = gyroZDps;
        _calibrationSampleCount = 1;
    }
    else if ((uint32_t)(nowMs - _lastCalibrationSampleMs) >= sampleIntervalMs)
    {
        _calibrationSum += gyroZDps;
        ++_calibrationSampleCount;
        _lastCalibrationSampleMs = nowMs;
    }

    if ((uint32_t)(nowMs - _calibrationStartMs) < durationMs)
    {
        return false;
    }

    _gyroBiasDps = _calibrationSum / _calibrationSampleCount;
    _calibrationDone = true;
    return true;
}

void AngleEstimator::update(float gyroZDps, float dtSeconds)
{
    // Следующий вызов calibrate() начнёт новую калибровочную сессию.
    _calibrationSampleCount = 0;
    _calibrationDone = false;

    if (dtSeconds > 0.0f)
    {
        _angleDeg = normalize(_angleDeg + (gyroZDps - _gyroBiasDps) * dtSeconds);
    }
}

float AngleEstimator::getAngle() const
{
    return _angleDeg;
}

float AngleEstimator::normalize(float angleDeg)
{
    angleDeg = fmodf(angleDeg, 360.0f);
    return angleDeg < 0.0f ? angleDeg + 360.0f : angleDeg;
}
