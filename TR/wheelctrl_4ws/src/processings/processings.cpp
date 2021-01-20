#include "processings.h"

namespace processings {
    void PI2INF(float &angle, const float &former_angle)
    {
        if (former_angle - angle > PI){
            while (abs(former_angle - angle) > PI)
            {
                angle += 2 * PI;
            }
        }
        else if (former_angle - angle < -1*PI)
        {
            while (abs(former_angle - angle) > PI)
            {
                angle -= 2 * PI;
            }
        }
    }

    void AdjustDirection(float &angle, const float &former_angle, int &speed_flag)
    {
        if (former_angle - angle > 1e-7f)
        {
            if (former_angle - angle < PI / 2.0f)
            {
                if (speed_flag == -1)
                    speed_flag *= -1;
            }
            else if (former_angle - angle > PI / 2.0f)
            {
                angle += PI;
                if (speed_flag == 1)
                    speed_flag *= -1;
            }
            else
            {
                if (speed_flag == 1);
                else
                    speed_flag *= -1;
            }
        }
        else if (former_angle - angle < -1e-7f)
        {
            if (former_angle - angle > -1 * PI / 2.0f)
            {
                if (speed_flag == -1)
                    speed_flag *= -1;
            }
            else if (former_angle - angle < -1 * PI / 2.0f)
            {
                angle -= PI;
                if (speed_flag == 1)
                    speed_flag *= -1;
            }
            else
            {
                if (speed_flag == 1);
                else
                    speed_flag *= -1;
            }
        }
        else
        {
            if (speed_flag == 1);
            else
                speed_flag *= -1;
        }
    }

    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Passed by Reference
    }
}