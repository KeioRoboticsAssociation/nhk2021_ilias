#ifndef JOY_COMMANDER_TR_H_
#define JOY_COMMANDER_TR_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Joy.h>


/***************** joystick number ********************/
#define JOY_X 1
#define JOY_Y 0
#define JOY_OMEGA 3
/******************************************************/

#define PI 3.141592f

class JOYSTICK_TR
{
public:
    JOYSTICK_TR(ros::NodeHandle &nh, const int &loop_rate);
    ~JOYSTICK_TR(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher cmd_pub;
    ros::Subscriber joy_sub;

    //Configurations
    int loop_rate_;

    //variables
    geometry_msgs::Twist cmd_vel;

    //Methods
    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void update();
};

#endif
