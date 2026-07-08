// #include <Arduino.h>
// #include "Movement.h"
// #include "CENS.h"
// #include "Map.hpp"
// #include "MPU6050/MPU6050.h"
// #include "VL53L0X/VL53L0X.h"

// Map myMap;
// CENS cens(myMap);
// Movement Car;
// MPU6050 gyro;
// VL53L0X distance;

// void setup() {
//   myMap.calculateMap();
//   Car.init_motors();

//   // distance.begin();
//   // gyro.begin();
// }

// void loop() {
//   // uint16_t d;
//   // float g;

//   // distance.readDistanceMillimeters(&d);
//   // gyro.readGyroZDegreesPerSecond(&g);

//   Car.Forward();
//   delay(10000);

//   Car.Back();
//   delay(10000);

//   Car.TurnRight();
//   delay(10000);

//   Car.TurnLeft();
//   delay(10000);

//   Car.Stop();
//   delay(20000);
// }


#include <Arduino.h>
#include "Movement.h"
#include "CENS.h"
#include "Map.hpp"
#include "MPU6050/MPU6050.h"
#include "VL53L0X/VL53L0X.h"
#include "CRSF.h"

Map myMap;
CENS cens(myMap);
Movement Car;
MPU6050 gyro;
VL53L0X distance;
CRSF rc;

// Каналы в порядке AETR (стандарт для ExpressLRS/Betaflight):
// CH0 = Roll (влево/вправо), CH1 = Pitch, CH2 = Throttle (газ), CH3 = Yaw
const uint8_t CH_STEER = 0;
const uint8_t CH_THROTTLE = 2;

// Диапазон стика CRSF: 172..1811, центр 992
const int16_t CRSF_MIN = 172;
const int16_t CRSF_MAX = 1811;
const int16_t CRSF_CENTER = 992;

// "Мёртвая зона" вокруг центра стика в единицах CRSF, чтобы машина не "ползла"
// от дрейфа/шума стика в нейтрали
const int16_t DEADZONE = 40;

// Насколько сильно steer урезает влияние на скорость поворота относительно throttle.
// 1.0 = полный вклад, 0.5 = более плавные повороты и т.п.
const float STEER_SCALE = 1.0f;

int16_t applyDeadzone(int16_t raw) {
    int16_t v = raw - CRSF_CENTER;
    if (abs(v) < DEADZONE) return 0;
    return v;
}

void setup() {
    myMap.calculateMap();
    Car.init_motors();

    rc.begin(Serial1, 115200); // RX1/TX1 на Mega, стандартный ELRS baud rate

    // distance.begin();
    // gyro.begin();
}

void loop() {
    rc.update();

    if (!rc.isLinkUp()) {
        Car.Stop();
        return;
    }

    int16_t steerRaw = applyDeadzone(rc.getChannel(CH_STEER));
    int16_t throttleRaw = applyDeadzone(rc.getChannel(CH_THROTTLE));

    // приводим из диапазона CRSF (после вычитания центра) к -255..255 (ШИМ)
    int16_t throttle = map(throttleRaw, -(CRSF_CENTER - CRSF_MIN), (CRSF_MAX - CRSF_CENTER), -255, 255);
    int16_t steer = map(steerRaw, -(CRSF_CENTER - CRSF_MIN), (CRSF_MAX - CRSF_CENTER), -255, 255);
    steer = (int16_t)(steer * STEER_SCALE);
 
    // arcade-микширование (танковый стиль): одновременно газ и поворот.
    // Внутренняя по повороту сторона может уходить в реверс при резком руле -
    // это даёт более резкие развороты вплоть до разворота на месте.
    int16_t left = constrain(throttle + steer, -255, 255);
    int16_t right = constrain(throttle - steer, -255, 255);

    Car.Drive(left, right);
}