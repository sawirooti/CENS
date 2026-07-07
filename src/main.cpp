#include <Arduino.h>
#include "Movement.h"
#include "CENS.h"
#include "Map.hpp"
#include "MPU6050/MPU6050.h"
#include "VL53L0X/VL53L0X.h"

Map myMap;
CENS cens(myMap);
Movement Car;
MPU6050 gyro;
VL53L0X distance;

void setup() {
  myMap.calculateMap();
  Car.init_motors();

  distance.begin();
  gyro.begin();
}

void loop() {
  uint16_t d;
  float g;

  distance.readDistanceMillimeters(&d);
  gyro.readGyroZDegreesPerSecond(&g);
}
