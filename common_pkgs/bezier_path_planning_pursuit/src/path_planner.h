#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "path_planning.h"

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <actionlib/server/simple_action_server.h>          // action Library Header File
#include <bezier_path_planning_pursuit/PursuitPathAction.h> // PursuitPathAction Action File Header

using namespace bezier_path_planning_pursuit;

#define LINE_NUM 2

class Path_Planner
{
public:
    Path_Planner(ros::NodeHandle &nh, const int &loop_rate, const std::string &zonename, const bool &use_odom_tf, const std::string &data_path, const float &max_accel, const float &max_vel, const float &corner_speed_rate, const std::string &global_frame_id, const float &initial_vel, const float &goal_tolerance);
    ~Path_Planner(){};

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
    std::string zonename_;
    bool use_odom_tf_;
    std::string data_path_;
    std::string global_frame_id_;

    float max_accel_;
    float max_vel_;
    float initial_vel_;
    float corner_speed_rate_;
    float goal_tolerance_;

    Path path[LINE_NUM];
    nav_msgs::Path path_ros[LINE_NUM];

    //variables
    tf::TransformListener listener;
    Matrix control;
    float body_theta = 0;
    float position[2] = {0}; // [x,y]

    int path_mode = 1; // 0:none, 1:path1, 2:path2, ...LINE_NUM
    int forwardflag = 1;

    //Methods
    void odomCallback(const nav_msgs::Odometry &msg);
    void bnoCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
    geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw);
    void setPoseTopic(const int &path_num);

    void setup(std::string zone, float accel, float max_vel, float init_vel, float corner_speed_rate);
    void executeCB(const PursuitPathGoalConstPtr &goal);
    bool reachedGoal();
    void publishMsg(const float &vx, const float &vy, const float &omega);
    void update();
};

#endif