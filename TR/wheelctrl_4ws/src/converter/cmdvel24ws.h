#ifndef CMDVEL24WS_H_
#define CMDVEL24WS_H_

#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class VelConverter
{
public:
    VelConverter(ros::NodeHandle &nh, const int &body_width, const int &loop_rate);
    ~VelConverter(){};
    void update();

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher pub_RF;
    ros::Publisher pub_LF;
    ros::Publisher pub_LB;
    ros::Publisher pub_RB;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber bno_sub_;

    //Configurations
    int hoge;
    int loop_rate_;
    float BODY_WIDTH;

    //variables
    float theta;
    float vx;
    float vy;
    float omega;
    float target_speed[4];
    float target_theta[4];

    //Methods
    void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    void bnoCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);
    void publishMsg();
    void cmdvel24ws(const float &theta_body, const float &vx, const float &vy, const float &omega);
};

#endif
