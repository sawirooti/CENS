#include <Arduino.h>
#include "Map.h"
#include "CENS.h"
#include "filter.h"
#include "MPU6050/MPU6050.h"
#include "VL53L0X/VL53L0X.h"
#include "AngleEstimator.h"
#include "Movement.h"

RobotState start(X_START, Y_START, PSI_START);
Map karta;
CENS cens(karta);
KensKalmanFilter3D Kalman(start, 1, 1, 1, 1);
MPU6050 gyro;
VL53L0X dist;
AngleEstimator teta(PSI_START); 
Movement Car;

static float distanceFront = LENGTH - Y_START * DELTA;
static float distanceRight = WIDTH - X_START * DELTA;
static Position pos;
static float d;
static float S;
static float teta_zad; //radian
static float half_S;
static uint32_t lastMicros = micros();
static uint32_t Micros = micros();
static float gyroZ = 0;
static RobotState position;
static float last_d;

void setup()
{
    gyro.begin();
    dist.begin();
    Car.init_motors();
}

void loop()
{
    pos = cens.calculatePosition(distanceFront, distanceRight);

    position.x = pos.x;
    position.y = pos.y;
    position.theta = teta.getAngle();
    Kalman.predict();
    Kalman.update(position);
    Kalman.getState();

    S = sqrtf((X_GOAL - pos.x) * (X_GOAL - pos.y) + (Y_GOAL - pos.y) * (Y_GOAL - pos.y)) * DELTA;
    half_S = d - S / 2.0f;

    if (S < DELTA) {
        Car.Stop();
        //Lampa
        while(1);
    }

    teta_zad = atan2f((Y_GOAL - pos.y), (X_GOAL - pos.x)) - PI / 2;

    while (abs(teta_zad * RAD_TO_DEG - teta.getAngle() + 180) >= 5)
    {   
        Micros = micros();
        gyro.readGyroZDegreesPerSecond(&gyroZ);
        teta.update(gyroZ, (Micros - lastMicros) / 1000000.0f);
        lastMicros = Micros;

        if ((teta_zad * RAD_TO_DEG - teta.getAngle() + 180) > 0)
        {
            Car.TurnLeft();
        }
        else {
            Car.TurnRight();
        }
    }
    
    //TODO: Добавить перерасчет дальностей после поворота

    last_d = getMedianFilter(dist.lastDistanceMillimeters());
    d = getMedianFilter(dist.lastDistanceMillimeters());

    while (d >= half_S)
    {   
        Micros = micros();
        gyro.readGyroZDegreesPerSecond(&gyroZ);
        teta.update(gyroZ, (Micros - lastMicros) / 1000000.0f);
        float tet = teta.getAngle();
        distanceFront += (d - last_d) * sin(tet); //тут надо подумать
        distanceRight += (d - last_d) * cos(tet);

        lastMicros = Micros;
        last_d = d;
        Car.Forward();
        delay(100);
        d = getMedianFilter(dist.lastDistanceMillimeters());
    }

    Car.Stop();

    uint32_t end = micros() + 3000;
    while (micros() < end) { 
        gyro.readGyroZDegreesPerSecond(&gyroZ);
        teta.calibrate(gyroZ, micros());
    }
}
