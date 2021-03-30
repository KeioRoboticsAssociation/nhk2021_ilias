#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "path_planning.h"

#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>

#include <actionlib/server/simple_action_server.h>          // action Library Header File
#include <bezier_path_planning_pursuit/PursuitPathAction.h> // PursuitPathAction Action File Header

using namespace bezier_path_planning_pursuit;

class Path_Planner
{
public:
    Path_Planner(ros::NodeHandle &nh, const int &loop_rate, const bool &use_tf, const std::string &data_path, const float &max_accel, const float &max_vel, const float &corner_speed_rate, const std::string &global_frame_id, const std::string &base_frame_id, const float &initial_vel, const float &xy_goal_tolerance, const float &yaw_goal_tolerance, const std::string &angle_source, const float &max_vel_theta, const float &acc_lim_theta, const float &fix_angle_gain, const int &LINE_NUMBER, const float &path_granularity);
    ~Path_Planner()
    {
        delete[] path;
        delete[] path_ros;
    };

private:
    //Handlers
    ros::NodeHandle &nh_;

    actionlib::SimpleActionServer<PursuitPathAction> as_; // Action server declaration, NodeHandle instance must be created before this line. Otherwise strange error occurs.

    ros::Publisher cmd_pub;
    ros::Publisher path_pub;
    ros::Subscriber bno_sub;
    ros::Subscriber odom_sub;

    PursuitPathFeedback feedback_;
    PursuitPathResult result_;

    //Configurations
    int loop_rate_;
    int LINE_NUM;
    bool use_tf_;
    std::string data_path_;
    std::string global_frame_id_;
    std::string base_frame_id_;
    std::string angle_source_;

    float max_accel_;
    float max_vel_;
    float max_vel_theta_;
    float acc_lim_theta_;
    float initial_vel_;
    float corner_speed_rate_;
    float xy_goal_tolerance_;
    float yaw_goal_tolerance_;
    float fix_angle_gain_;
    float goal_position_x;
    float goal_position_y;
    float path_granularity_;

    Path *path;
    nav_msgs::Path *path_ros;

    //variables
    tf::TransformListener listener;
    Matrix control;
    float body_theta = 0;
    float position[2] = {0}; // [x,y]
    float _old_omega_;
    float _omega_;
    float _old_vx_;
    float _vx_;
    float _old_vy_;
    float _vy_;

    int path_mode = 1; // 0:none, 1:path1, 2:path2, ...LINE_NUM
    int forwardflag = 1;

    //Methods
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void bnoCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &imu);
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
    geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw);
    void setPoseTopic(const int &path_num);

    void setup(float accel, float max_vel, float acc_lim_theta, float max_vel_theta, float init_vel, float corner_speed_rate, float path_granularity);
    void init_variables();
    void executeCB(const PursuitPathGoalConstPtr &goal);
    void AdjustVelocity(float &v, float &old_v, const float &max_v, const float &acc_lim);
    bool reachedxyGoal();
    void terminate(const float &target_angle);
    void publishMsg(const float &vx, const float &vy, const float &omega);
};

#endif