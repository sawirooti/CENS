#include <Arduino.h>
#include "MPU6050/MPU6050.h"
#include "VL53L0X/VL53L0X.h"
#include "filter.h"

MPU6050 gyro;
VL53L0X distance;

static float teta = 0;
static uint32_t lastMicros = 0;

void setup()
{
    Serial.begin(9600);

    if (!gyro.begin())
    {
        Serial.print("MPU6050 init error: ");
        Serial.println(gyro.lastError());
    }

    if (!distance.begin())
    {
        Serial.print("VL53L0X init error: ");
        Serial.println(distance.lastError());
    }
}

void loop()
{
    float gyroZ = 0.0f;
    uint16_t distanceMm = 0;
    const uint32_t nowMicros = micros();

    const bool gyroOk = gyro.readGyroZDegreesPerSecond(&gyroZ);
    const bool distanceOk = distance.readDistanceMillimeters(&distanceMm);
    const float dt = (uint32_t)(nowMicros - lastMicros) / 1000000.0f;

    Serial.print("Gyro Z: ");
    if (gyroOk)
    {
        Serial.print(gyroZ);
        Serial.print(" deg/s");
    }
    else
    {
        Serial.print("error ");
        Serial.print(gyro.lastError());
    }
    if (lastMicros != 0 && gyroOk)
    {
        teta += gyroZ * dt;
    }

    Serial.print(" | Teta: ");
    Serial.print(teta);
    Serial.print(" | dt: ");
    if (lastMicros == 0)
    {
        Serial.print(0.0f);
    }
    else
    {
        Serial.print((uint32_t)(nowMicros - lastMicros) / 1000000.0f, 4);
    }

    lastMicros = nowMicros;

    Serial.print(" | Distance: ");
    if (distanceOk)
    {
        Serial.print(distanceMm);
        Serial.print(" mm");
    }
    else
    {
        Serial.print(distanceMm);
        Serial.print(" mm");
        Serial.print(" error ");
        Serial.print(distance.lastError());
    }

    Serial.print(" | Median Distance: ");
    Serial.println(getMedianFilter(distanceMm));

    delay(100);
}
