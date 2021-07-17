#include "joy_commander/joy_commander.h"

JOYSTICK::JOYSTICK(ros::NodeHandle &nh, const int &loop_rate, const float &acc_lim_xy, const float &max_vel_xy, const float &max_vel_theta, const float &acc_lim_theta)
: nh_(nh), loop_rate_(loop_rate), acc_lim_xy_(acc_lim_xy), max_vel_xy_(max_vel_xy), max_vel_theta_(max_vel_theta), acc_lim_theta_(acc_lim_theta){
    ROS_INFO("Creating joy_commander");

    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("acc_lim_xy [m/s^2]: " << acc_lim_xy);
    ROS_INFO_STREAM("max_vel_xy [m/s]: " << max_vel_xy_);
    ROS_INFO_STREAM("acc_lim_theta [rad/s^2]: " << acc_lim_theta_);
    ROS_INFO_STREAM("max_vel_theta [rad/s]: " << max_vel_theta_);

    cmd_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    init_angle_pub = nh_.advertise<std_msgs::Empty>("/init_angle_flag", 1);
    joy_sub = nh_.subscribe("/joy", 1,
                                 &JOYSTICK::joy_callback, this);
    teleopflag_sub = nh_.subscribe("teleopflag", 1,
                                 &JOYSTICK::teleopflag_callback, this);
    teleop_flag = true;
    old_omega = 0;
    omega = 0;
    old_vx = 0;
    vx = 0;
    old_vy = 0;
    vy = 0;
    update();
}

float JOYSTICK::AdjustVelocity(const float &ref, float &old_v, const float &max_v, const float &acc_lim)
{
    float v = ref;
    // max_acc limit
    if (v - old_v >= 0)
    {
        if ((v - old_v) * (float)loop_rate_ > acc_lim)
        {
            v = old_v + acc_lim / (float)loop_rate_;
        }
    }
    else
    {
        if ((v - old_v) * (float)loop_rate_ < -1 * acc_lim)
        {
            v = old_v - acc_lim / (float)loop_rate_;
        }
    }
    // max_vel limit
    if (v >= 0)
    {
        if (v > max_v)
        {
            v = max_v;
        }
    }
    else
    {
        if (v < -1 * max_v)
        {
            v = -1 * max_v;
        }
    }

    old_v = v;
    return v;
}

float JOYSTICK::roundoff(const float &value, const float &epsilon)
{
    float ans = value;
    if(abs(ans) < epsilon){
        ans = 0.0;
    }
    return ans;
}

void JOYSTICK::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  vx = roundoff(joy_msg->axes[JOY_X], 1e-4)*max_vel_xy_;
  vy = roundoff(joy_msg->axes[JOY_Y], 1e-4)*max_vel_xy_;
  omega = roundoff(joy_msg->axes[JOY_OMEGA], 1e-4)*max_vel_theta_;
  if (joy_msg->buttons[JOY_RESET]){
      static std_msgs::Empty topic;
      init_angle_pub.publish(topic);
  }
}

void JOYSTICK::teleopflag_callback(const std_msgs::Bool::ConstPtr &joy_msg){
    teleop_flag = joy_msg->data;
}

void JOYSTICK::update()
{
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        if(teleop_flag){
            cmd_vel.linear.x = AdjustVelocity(vx, old_vx, max_vel_xy_, acc_lim_xy_);
            cmd_vel.linear.y = AdjustVelocity(vy, old_vy, max_vel_xy_, acc_lim_xy_);
            cmd_vel.angular.z = AdjustVelocity(omega, old_omega, max_vel_theta_, acc_lim_theta_);
            cmd_pub.publish(cmd_vel);
        }
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
    
    float acc_lim_xy = 2.5;
    float max_vel_xy = 1.5;
    float acc_lim_theta = 3.2;
    float max_vel_theta = 1.57;

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("acc_lim_xy", acc_lim_xy);
    arg_n.getParam("acc_lim_theta", acc_lim_theta);
    arg_n.getParam("max_vel_xy", max_vel_xy);
    arg_n.getParam("max_vel_theta", max_vel_theta);

    JOYSTICK joystick(nh,looprate, acc_lim_xy, max_vel_xy, max_vel_theta, acc_lim_theta);
    return 0;
}
