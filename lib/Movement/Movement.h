#pragma once

#include <Arduino.h>
#include "Defenition.hpp"

class Motor 
{
    private:

    int _pin_pwm = 0;
    int _pin_cw = 0;
    int _pin_ccw = 0;
    int _vel = SPEED;

    public:

    Motor(int pin_pwm, int pin_cw, int pin_ccw) : _pin_pwm(pin_pwm), _pin_cw(pin_cw), _pin_ccw(pin_ccw) {}
    
    Motor() = default;
    ~Motor() = default;

    void init();

    void Move_CW();
    void Move_CCW();
    void Stop();
    void Neutral();
};

class Movement
{   
    private:

    Motor FL{FL_PWM, FL_CW, FL_CCW};
    Motor FR{FR_PWM, FR_CW, FR_CCW};
    Motor BL{BL_PWM, BL_CW, BL_CCW};
    Motor BR{BR_PWM, BR_CW, BR_CCW};

    public:

    Movement() = default;
    ~Movement() = default;
    
    void init_motors();

    void Forward();
    void Back();
    void TurnRight();
    void TurnLeft();
    void Stop();
    void Neutral();
};