#pragma once

#define SPEED 200

// Размеры карты
#define LENGTH 200
#define WIDTH 100
#define DELTA 10     // шаг карты

// Пины моторов
// ВАЖНО: на Arduino Mega настоящий (переменный) ШИМ доступен только на пинах 2-13 и 44-46.
// A0-A3 такого не поддерживают - analogWrite() на них работает только как HIGH/LOW с порогом 128,
// без плавного регулирования скорости. Поэтому *_PWM перенесены на пины 10-13.
#define FL_PWM 10
#define FL_CW 3
#define FL_CCW 2

#define FR_PWM 11
#define FR_CW 5
#define FR_CCW 4

#define BL_PWM 12
#define BL_CW 9
#define BL_CCW 8

#define BR_PWM 13
#define BR_CW 7
#define BR_CCW 6

// Пины дальномера

// Пины гироскопа