#include <Arduino.h>
#include <math.h>

#include "AngleEstimator.h"
#include "MPU6050/MPU6050.h"
#include "Movement.h"
#include "VL53L0X/VL53L0X.h"
#include "testfilters.hpp"

enum class TestStage
{
    Calibration,
    Forward3s,
    Turn360,
    Forward1s,
    Done,
    Error
};

struct FilterValues
{
    float median;
    float ksimple;
    float peak;
    float minquad;
};

class FilterBank
{
public:
    void init(float value, float ksimpleNoise, float peakThreshold)
    {
        _median.init(value);
        _ksimple.init(value);
        _ksimple.config(ksimpleNoise);
        _peak.init(value);
        _peak.config(peakThreshold, 3);

        _sampleIndex = 0.0f;
        _count = 0;
        for (uint8_t i = 0; i < WINDOW; ++i)
        {
            _x[i] = 0.0f;
            _y[i] = value;
        }
        pushMinQuad(value);
    }

    FilterValues filter(float value)
    {
        FilterValues result;
        result.median = _median.filter(value);
        result.ksimple = _ksimple.filter(value, 0.3f);
        result.peak = _peak.filter(value);
        result.minquad = pushMinQuad(value);
        return result;
    }

private:
    static const uint8_t WINDOW = 8;

    float pushMinQuad(float value)
    {
        if (_count < WINDOW)
        {
            _x[_count] = _sampleIndex;
            _y[_count] = value;
            ++_count;
        }
        else
        {
            for (uint8_t i = 1; i < WINDOW; ++i)
            {
                _x[i - 1] = _x[i];
                _y[i - 1] = _y[i];
            }
            _x[WINDOW - 1] = _sampleIndex;
            _y[WINDOW - 1] = value;
        }
        _sampleIndex += 1.0f;

        _minquad.compute<float>(_x, _y, _count);
        if (_count < 2)
        {
            return value;
        }
        return _minquad.getA() * _x[_count - 1] + _minquad.getB();
    }

    Median3<float> _median;
    KSimple _ksimple;
    PeakFilter<float> _peak;
    MinQuad _minquad;
    float _x[WINDOW];
    float _y[WINDOW];
    float _sampleIndex;
    uint8_t _count;
};

const uint32_t SAMPLE_PERIOD_MS = 50;
const uint32_t CALIBRATION_TIME_MS = 1000;
const uint16_t CALIBRATION_RATE_HZ = 100;
const uint32_t FORWARD_3S_MS = 2000;
const uint32_t FORWARD_1S_MS = 1000;
const uint32_t TURN_TIMEOUT_MS = 2000;
const float TURN_TARGET_DEG = 360.0f;

MPU6050 gyro;
VL53L0X distanceSensor;
Movement car;
AngleEstimator angleEstimator(0.0f);

FilterBank distanceFilters;
FilterBank gyroFilters;
FilterBank angleFilters;

TestStage stage = TestStage::Calibration;
uint32_t stageStartMs = 0;
uint32_t lastSampleMs = 0;
uint32_t lastMicros = 0;

float gyroBiasSum = 0.0f;
uint16_t gyroBiasCount = 0;
float gyroBiasDps = 0.0f;
float integratedAngleDeg = 0.0f;
float turnStartAngleDeg = 0.0f;
bool filtersReady = false;

const char* stageName(TestStage value)
{
    switch (value)
    {
    case TestStage::Calibration: return "CALIBRATION";
    case TestStage::Forward3s: return "FORWARD_3S";
    case TestStage::Turn360: return "TURN_360";
    case TestStage::Forward1s: return "FORWARD_1S";
    case TestStage::Done: return "DONE";
    case TestStage::Error: return "ERROR";
    }
    return "UNKNOWN";
}

void printCsvHeader()
{
    Serial.println(
        "time_ms,stage,"
        "dist_raw,dist_median,dist_ksimple,dist_peak,dist_minquad,"
        "gyro_raw,gyro_median,gyro_ksimple,gyro_peak,gyro_minquad,"
        "angle_raw,angle_median,angle_ksimple,angle_peak,angle_minquad");
}

void printCsv(uint32_t nowMs,
              uint16_t rawDistance,
              float rawGyro,
              float rawAngle,
              const FilterValues& distanceValues,
              const FilterValues& gyroValues,
              const FilterValues& angleValues)
{
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(stageName(stage));
    Serial.print(',');
    Serial.print(rawDistance);
    Serial.print(',');
    Serial.print(distanceValues.median);
    Serial.print(',');
    Serial.print(distanceValues.ksimple);
    Serial.print(',');
    Serial.print(distanceValues.peak);
    Serial.print(',');
    Serial.print(distanceValues.minquad);
    Serial.print(',');
    Serial.print(rawGyro);
    Serial.print(',');
    Serial.print(gyroValues.median);
    Serial.print(',');
    Serial.print(gyroValues.ksimple);
    Serial.print(',');
    Serial.print(gyroValues.peak);
    Serial.print(',');
    Serial.print(gyroValues.minquad);
    Serial.print(',');
    Serial.print(rawAngle);
    Serial.print(',');
    Serial.print(angleValues.median);
    Serial.print(',');
    Serial.print(angleValues.ksimple);
    Serial.print(',');
    Serial.print(angleValues.peak);
    Serial.print(',');
    Serial.println(angleValues.minquad);
}

void enterStage(TestStage nextStage)
{
    car.Stop();
    stage = nextStage;
    stageStartMs = millis();

    Serial.print("# STAGE ");
    Serial.println(stageName(stage));

    if (stage == TestStage::Forward3s)
    {
        car.Forward();
    }
    else if (stage == TestStage::Turn360)
    {
        turnStartAngleDeg = integratedAngleDeg;
        car.TurnLeft();
    }
    else if (stage == TestStage::Forward1s)
    {
        car.Forward();
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("# BOOT test filters");

    car.init_motors();
    car.Stop();

    const bool gyroOk = gyro.begin();
    Serial.print("# MPU6050 ");
    Serial.println(gyroOk ? "OK" : "ERROR");
    if (!gyroOk)
    {
        Serial.print("# MPU6050 status ");
        Serial.println(gyro.lastError());
    }

    const bool distanceOk = distanceSensor.begin();
    Serial.print("# VL53L0X ");
    Serial.println(distanceOk ? "OK" : "ERROR");
    if (!distanceOk)
    {
        Serial.print("# VL53L0X status ");
        Serial.println(distanceSensor.lastError());
    }

    if (!gyroOk || !distanceOk)
    {
        stage = TestStage::Error;
        return;
    }

    float firstGyro = 0.0f;
    uint16_t firstDistance = 0;
    if (!gyro.readGyroZDegreesPerSecond(&firstGyro) ||
        !distanceSensor.readDistanceMillimeters(&firstDistance))
    {
        stage = TestStage::Error;
        Serial.println("# ERROR first sensor read");
        return;
    }

    distanceFilters.init((float)firstDistance, 20.0f, 60.0f);
    gyroFilters.init(firstGyro, 1.0f, 5.0f);
    angleFilters.init(0.0f, 2.0f, 10.0f);
    filtersReady = true;

    printCsvHeader();
    stageStartMs = millis();
    lastSampleMs = 0;
    lastMicros = micros();
    Serial.println("# STAGE CALIBRATION");
}

void loop()
{
    if (stage == TestStage::Error)
    {
        car.Stop();
        delay(100);
        return;
    }

    if (stage == TestStage::Done)
    {
        car.Stop();
        delay(100);
        return;
    }

    float rawGyro = 0.0f;
    uint16_t rawDistance = 0;
    if (!gyro.readGyroZDegreesPerSecond(&rawGyro))
    {
        stage = TestStage::Error;
        car.Stop();
        Serial.print("# ERROR gyro read ");
        Serial.println(gyro.lastError());
        return;
    }
    if (!distanceSensor.readDistanceMillimeters(&rawDistance))
    {
        stage = TestStage::Error;
        car.Stop();
        Serial.print("# ERROR distance read ");
        Serial.println(distanceSensor.lastError());
        return;
    }

    const uint32_t nowMicros = micros();
    const float dtSeconds = (uint32_t)(nowMicros - lastMicros) / 1000000.0f;
    lastMicros = nowMicros;

    if (stage == TestStage::Calibration)
    {
        car.Stop();
        angleEstimator.calibrate(rawGyro, millis(), CALIBRATION_TIME_MS, CALIBRATION_RATE_HZ);
        gyroBiasSum += rawGyro;
        ++gyroBiasCount;
    }
    else
    {
        angleEstimator.update(rawGyro, dtSeconds);
        integratedAngleDeg += (rawGyro - gyroBiasDps) * dtSeconds;
    }

    const FilterValues distanceValues = distanceFilters.filter((float)rawDistance);
    const FilterValues gyroValues = gyroFilters.filter(rawGyro);
    const FilterValues angleValues = angleFilters.filter(integratedAngleDeg);

    const uint32_t nowMs = millis();
    if (filtersReady && (lastSampleMs == 0 || (uint32_t)(nowMs - lastSampleMs) >= SAMPLE_PERIOD_MS))
    {
        lastSampleMs = nowMs;
        printCsv(nowMs,
                 rawDistance,
                 rawGyro,
                 integratedAngleDeg,
                 distanceValues,
                 gyroValues,
                 angleValues);
    }

    switch (stage)
    {
    case TestStage::Calibration:
        if ((uint32_t)(nowMs - stageStartMs) >= CALIBRATION_TIME_MS)
        {
            if (gyroBiasCount > 0)
            {
                gyroBiasDps = gyroBiasSum / gyroBiasCount;
            }
            integratedAngleDeg = 0.0f;
            angleEstimator.setAngle(0.0f);

            Serial.print("# CALIBRATION gyro_bias_dps ");
            Serial.println(gyroBiasDps);
            enterStage(TestStage::Forward3s);
        }
        break;

    case TestStage::Forward3s:
        car.Forward();
        if ((uint32_t)(nowMs - stageStartMs) >= FORWARD_3S_MS)
        {
            enterStage(TestStage::Turn360);
        }
        break;

    case TestStage::Turn360:
        car.TurnRight();
        if (fabsf(integratedAngleDeg - turnStartAngleDeg) >= TURN_TARGET_DEG)
        {
            enterStage(TestStage::Forward1s);
        }
        else if ((uint32_t)(nowMs - stageStartMs) >= TURN_TIMEOUT_MS)
        {
            Serial.println("# TURN timeout, continue to FORWARD_1S");
            enterStage(TestStage::Forward1s);
        }
        break;

    case TestStage::Forward1s:
        car.Forward();
        if ((uint32_t)(nowMs - stageStartMs) >= FORWARD_1S_MS)
        {
            enterStage(TestStage::Done);
            Serial.println("# DONE");
        }
        break;

    case TestStage::Done:
    case TestStage::Error:
        break;
    }

    delay(10);
}