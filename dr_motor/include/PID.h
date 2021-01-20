#ifndef MBED_PID_H
#define MBED_PID_H

#include "mbed.h"

class PID
{
public:
    PID(double,double,double,int);//Kp,Ki,Kd,周期
    double Kp,Ki,Kd;//各定数
    double diff[2];//差分を管理
    int Delta_T;//周期
    int prespeed;
    double pid(double,double);
    void set_K(double,double,double);
};

#endif