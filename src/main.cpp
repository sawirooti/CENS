#include <Arduino.h>
#include "Movement.h"
#include "CENS.h"
#include "map.hpp"

Map myMap;
CENS cens(myMap);
Movement Car;

void setup() {
  myMap.calculateMap();
  Car.init_motors();
}

void loop() {

}
