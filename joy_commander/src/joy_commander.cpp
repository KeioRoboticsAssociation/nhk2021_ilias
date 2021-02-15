#include "joy_commander/joy_commander.h"

JOYSTICK::JOYSTICK(ros::NodeHandle &nh, const int &loop_rate): nh_(nh), loop_rate_(loop_rate){
    ROS_INFO("Creating joy_commander");

    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);

    cmd_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    joy_sub = nh_.subscribe("/joy", 1,
                                 &JOYSTICK::joy_callback, this);

    update();
}

void JOYSTICK::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  cmd_vel.linear.x = joy_msg->axes[JOY_X];
  cmd_vel.linear.y = joy_msg->axes[JOY_Y];
  cmd_vel.angular.z = joy_msg->axes[JOY_OMEGA];
}

void JOYSTICK::update()
{
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        cmd_pub.publish(cmd_vel);
        ros::spinOnce();
        r.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_commander");

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");
    int looprate = 30; // Hz
    arg_n.getParam("controol_frequency", looprate);

    JOYSTICK joystick(nh,looprate);
    return 0;
}
