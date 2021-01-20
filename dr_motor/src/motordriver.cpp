//sign機能(high speed forwardからhigh speed backしてモーターイカれるのを防ぎませう)をつけよう
//dir 1が正転
#include "motordriver.h"
#include "mbed.h"
#include "PID.h"

Motor::Motor(PwmOut &pwm, DigitalOut &dir, double pulse, bool brakeable) : _pwm(pwm), _dir(dir), Pulse(pulse), Brakeable(brakeable)
{
    // Set initial condition of PWM
    _pwm.period_us(pulse);
    _pwm.pulsewidth_us(0);
}

void Motor::speed(double speed)
{
    if (speed > 0){
        if(speed>=Pulse*0.50) speed=Pulse*0.50;//セッター
        _dir=1;
        _pwm.pulsewidth_us(speed);
    }
    else if(speed < 0){
        if(speed<=-1.0*Pulse*0.50) speed=-1*Pulse*0.50;//セッター
        _dir=0;
        _pwm.pulsewidth_us(-1*speed);
    }
    else{
        _pwm.pulsewidth_us(0);
    }
}