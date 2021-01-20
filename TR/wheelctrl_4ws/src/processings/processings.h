#ifndef PROCESSINGS_H_
#define PROCESSINGS_H_

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

#define PI 3.141592F

namespace processings {
    void PI2INF(float &, const float &);
    void AdjustDirection(float &angle, const float &former_angle, int &speed_flag);
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
} // namespace processings

#endif
