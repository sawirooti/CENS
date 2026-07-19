#include "Movement.h"

void Motor::init()
{
    pinMode(_pin_pwm, OUTPUT);
    pinMode(_pin_cw, OUTPUT);
    pinMode(_pin_ccw, OUTPUT);
};

void Motor::Move_CW()
{
    digitalWrite(_pin_cw, HIGH);
    digitalWrite(_pin_ccw, LOW);

    analogWrite(_pin_pwm, _vel);
};

void Motor::Move_CCW()
{
    digitalWrite(_pin_cw, LOW);
    digitalWrite(_pin_ccw, HIGH);

    analogWrite(_pin_pwm, _vel);
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
    // FR.Move_CCW();
    BL.Move_CCW();
    // BR.Move_CCW();
    FR.Stop();
    BR.Stop();
};

void Movement::TurnLeft()
{
    // FL.Move_CW();
    FR.Move_CW();
    // BL.Move_CW();
    BR.Move_CW();
    FL.Stop();
    BL.Stop();
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