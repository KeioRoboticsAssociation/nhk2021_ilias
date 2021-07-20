#ifndef JOY_COMMANDER_H_
#define JOY_COMMANDER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

/***************** joystick number ********************/
#define JOY_X 1
#define JOY_Y 0
#define JOY_OMEGA 3
#define JOY_RESET 11
/******************************************************/

#define PI 3.141592f

class JOYSTICK
{
public:
    JOYSTICK(ros::NodeHandle &nh, const int &loop_rate, const float &acc_lim_xy, const float &max_vel_xy, const float &max_vel_theta, const float &acc_lim_theta);
    ~JOYSTICK(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher cmd_pub;
    ros::Publisher init_angle_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber teleopflag_sub;

    //Configurations
    int loop_rate_;
    float acc_lim_xy_;
    float max_vel_xy_;
    float acc_lim_theta_;
    float max_vel_theta_;

    //variables
    geometry_msgs::Twist cmd_vel;
    bool teleop_flag;
    float old_omega;
    float omega;
    float old_vx;
    float vx;
    float old_vy;
    float vy;

    //Methods
    float AdjustVelocity(const float &ref, float &old_v, const float &max_v, const float &acc_lim);
    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void teleopflag_callback(const std_msgs::Bool::ConstPtr &joy_msg);
    float roundoff(const float &value, const float &epsilon);
    void update();
};

#endif
