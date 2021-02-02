#include "joy_commander_tr.h"

JOYSTICK_TR::JOYSTICK_TR(ros::NodeHandle &nh, const int &loop_rate): nh_(nh), loop_rate_(loop_rate){
    ROS_INFO("Creating joy_commander_tr");

    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);

    cmd_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    joy_sub = nh_.subscribe("/joy", 1,
                                 &JOYSTICK_TR::joy_callback, this);

    update();
}

void JOYSTICK_TR::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  cmd_vel.linear.x = joy_msg->axes[JOY_X];
  cmd_vel.linear.y = joy_msg->axes[JOY_Y];
  cmd_vel.angular.z = joy_msg->axes[JOY_OMEGA];
}

void JOYSTICK_TR::update()
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
    ros::init(argc, argv, "joy_commander_tr");

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");
    int looprate = 30; // Hz
    arg_n.getParam("controol_frequency", looprate);

    JOYSTICK_TR joystick_tr(nh,looprate);
    return 0;
}
