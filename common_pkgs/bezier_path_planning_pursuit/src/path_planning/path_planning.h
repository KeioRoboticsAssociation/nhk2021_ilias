// path_planning.h

#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include "matrix.h"

//parameter
#define PURSUIT_AIM 0.5F;

#ifndef PI
#define PI 3.141592F
#endif

/******************** class ************************/
class Path
{
private:
    float max_accel = 1.2, max_vel = 1.0; // m/(s^2), m/s
    float max_initial_speed = 0.1;        // [m/s]
    float position[2] = {0, 0};           // [x,y]
    float ref_t = 1;
    Matrix control_point; // [ctrl_point][x,y]
    Matrix point;         // [point][x,y,theta,vel,length]
    void bezier();
    void set_vel();
    float costfunc(float t);
    float length_converge(float px_[4], float py_[4], float converge);
    float linear_search(float min, float max, float a, float b, float converge = 1e-5);

protected:
    Matrix solve_equations(Matrix M_, Matrix y_); // return [x](size*1)
    float bezier_length(float px[4], float py[4], float converge = 1);
    float linear_search(float (*cost)(float), float min, float max, float a, float b, float converge = 1e-5);

public:
    int pnum = 1;
    Path(){};
    Path(std::string filename);
    Path(std::string filename, float accel, float vel, float init_vel);
    void set_point_csv(std::string filename);
    Matrix path_func(float t);                                                    // return [x,y,theta,vel](4*1), 1<t<pnum
    Matrix pure_pursuit(float posx, float posy, bool foward, float reset_t = -1); // return [velx,vely,theta,ref_t](4*1)
};

#endif