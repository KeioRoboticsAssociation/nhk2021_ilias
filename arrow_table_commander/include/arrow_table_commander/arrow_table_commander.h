#ifndef Arrow_Table_Commander_H
#define Arrow_Table_Commander_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <cmath>
#include <string>

/***************** joystick number ********************/
#define JOY_PLUS_POT_NUMBER 7
#define JOY_MINUS_POT_NUMBER 6
#define JOY_RUN 0
/******************************************************/

class Arrow_Table_Commander
{
public:
    Arrow_Table_Commander(ros::NodeHandle &nh, const int &loop_rate, const std::string &base_frame_id);
    ~Arrow_Table_Commander(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher arrow_table_angle_pub;
    ros::Publisher thrower_flag_pub;
    ros::Publisher throwing1_power_pub;
    ros::Publisher throwing2_power_pub;
    ros::Publisher throwing3_power_pub;
    ros::Publisher throwing4_power_pub;
    ros::Publisher throwing5_power_pub;
    ros::Subscriber joy_sub;

    //Configurations
    int loop_rate_;
    std::string base_frame_id_;
    float thrower_position[5] = {0.4155, 0.2155, 0.0155, -0.1845, -0.3845};

    //variables
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    float table_angle; // rad
    float pot_distance;
    int pot_number;
    int thrower_number;
    int arrow_table_mode;

    //Methods
    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw);
    void BroadcastThrowerTF();
    void CalculateTableAngleAndDistance();
    void catch_arrow();
    void swing_arrow();
    void throw_arrow(int thrower_number);
    float CalculatePower(float distance);
};

#endif