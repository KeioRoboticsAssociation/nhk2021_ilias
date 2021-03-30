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
    float corner_speed_rate = 0.8;
    float max_initial_speed = 0.1;        // [m/s]
    float position[2] = {0, 0};           // [x,y]
    float ref_t = 1;
    float path_granularity = 0.01;
    const float gravitational_acceleration = 9.80665;
    Matrix control_point; // [ctrl_point][x,y]
    Matrix point;         // [point][x,y,theta,length]
    float *target_vel;
    int *waypoint_num;
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
    ~Path()
    {
        delete[] target_vel;
        delete[] waypoint_num;
    };
    Path(std::string filename);
    void load_config(std::string filename, float accel, float vel, float acc_lim_theta, float max_vel_theta, float init_vel, float speed_rate, float path_granularity_);
    void set_point_csv(std::string filename);
    void listen_goal_position(float &x, float &y, const bool &forward);
    Matrix path_func(float t);                                                    // return [x,y,theta,vel](4*1), 1<t<pnum
    Matrix pure_pursuit(float posx, float posy, float body_theta, bool foward, float reset_t = -1); // return [velx,vely,theta,ref_t](4*1)
};

#endif