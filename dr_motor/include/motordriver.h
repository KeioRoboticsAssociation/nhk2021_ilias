  
#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"

class Motor
{
public:
    Motor(PwmOut &pwm, DigitalOut &dir, double pulse, bool brakeable);
    void speed(double speed);
    void stop(float duty);

protected:
    PwmOut &_pwm;
    DigitalOut &_dir;
    double Pulse;
    bool Brakeable; // can the motor driver break
    int sign;       //prevents throwing the motor from full foward to full reverse and stuff melting.//まだ実装してない
};

#endif