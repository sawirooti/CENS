#include "Movement.h"

void Motor::init()
{
    pinMode(_pin_pwm, OUTPUT);
    pinMode(_pin_cw, OUTPUT);
    pinMode(_pin_ccw, OUTPUT);
};

void Motor::Move_CW()
{
    Move_CW(_vel);
};

void Motor::Move_CCW()
{
    Move_CCW(_vel);
};

void Motor::Move_CW(uint8_t speed)
{
    digitalWrite(_pin_cw, HIGH);
    digitalWrite(_pin_ccw, LOW);

    analogWrite(_pin_pwm, speed);
};

void Motor::Move_CCW(uint8_t speed)
{
    digitalWrite(_pin_cw, LOW);
    digitalWrite(_pin_ccw, HIGH);

    analogWrite(_pin_pwm, speed);
};

void Motor::Stop()
{
    digitalWrite(_pin_cw, HIGH);
    digitalWrite(_pin_ccw, HIGH);
};

void Motor::Neutral()
{
    digitalWrite(_pin_cw, LOW);
    digitalWrite(_pin_ccw, LOW);
};

void Movement::init_motors()
{
    FL.init();
    FR.init();
    BL.init();
    BR.init();
};

void Movement::Forward()
{
    FL.Move_CCW();
    FR.Move_CW();
    BL.Move_CCW();
    BR.Move_CW();
};

void Movement::Back()
{
    FL.Move_CW();
    FR.Move_CCW();
    BL.Move_CW();
    BR.Move_CCW();    
};

void Movement::TurnRight()
{
    FL.Move_CCW();
    FR.Move_CCW();
    BL.Move_CCW();
    BR.Move_CCW();
};

void Movement::TurnLeft()
{
    FL.Move_CW();
    FR.Move_CW();
    BL.Move_CW();
    BR.Move_CW();
};

void Movement::Stop()
{
    FL.Stop();
    FR.Stop();
    BL.Stop();
    BR.Stop();
};

void Movement::Neutral()
{
    FL.Neutral();
    FR.Neutral();
    BL.Neutral();
    BR.Neutral();
};

void Movement::Drive(int16_t left, int16_t right)
{
    // ограничиваем диапазон на случай, если извне придёт что-то за пределами -255..255
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    uint8_t leftSpeed = (uint8_t)abs(left);
    uint8_t rightSpeed = (uint8_t)abs(right);

    // левая сторона: вперёд = CCW (см. Forward())
    if (left > 0) {
        FL.Move_CCW(leftSpeed);
        BL.Move_CCW(leftSpeed);
    } else if (left < 0) {
        FL.Move_CW(leftSpeed);
        BL.Move_CW(leftSpeed);
    } else {
        FL.Stop();
        BL.Stop();
    }

    // правая сторона: вперёд = CW (см. Forward())
    if (right > 0) {
        FR.Move_CW(rightSpeed);
        BR.Move_CW(rightSpeed);
    } else if (right < 0) {
        FR.Move_CCW(rightSpeed);
        BR.Move_CCW(rightSpeed);
    } else {
        FR.Stop();
        BR.Stop();
    }
};