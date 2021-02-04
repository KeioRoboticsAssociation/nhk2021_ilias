#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

#include <actionlib/server/simple_action_server.h>          // action Library Header File
#include <bezier_path_planning_pursuit/PursuitPathAction.h> // PursuitPathAction Action File Header

#include "path_planning.h"

using namespace bezier_path_planning_pursuit;

#define LINE_NUM 1
/*
+++++ KEY +++++
0: start
1: receive
2~6: try1~5
7~9: kick1~3
m: manual_mode
p: show_path
F1: change_Red<->Blue
F2: create csv Blue -> Red

+++++ path_mode +++++
0: none
1: start -> recieve
2~6: receive -> try1~5
7: try1,2 -> kick1
8: try3~5 -> kick1
9: try1,2 -> kick2
10: try3~5 -> kick2
11: try1 -> kick3
12: try2~5 -> kick3
13: kick1 -> receive
14: kick2 -> receive
15: kick3 -> receive
16~20: start -> try1~5
21~23: start -> kick1~3
24: kick1 -> kick2
25: kick2 -> kick3
26: kick1 -> kick3
*/

class Path_Planner
{
public:
    Path_Planner(ros::NodeHandle &nh, const int &loop_rate, const std::string &zonename, const bool &use_odom_tf, const std::string &data_path, const float &max_accel, const float &max_vel, const float &corner_speed_rate);
    ~Path_Planner(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    actionlib::SimpleActionServer<PursuitPathAction> as_; // Action server declaration, NodeHandle instance must be created before this line. Otherwise strange error occurs.

    ros::Publisher cmd_pub;
    ros::Subscriber bno_sub;
    ros::Subscriber odom_sub;

    PursuitPathFeedback feedback_;
    PursuitPathResult result_;

    //Configurations
    int loop_rate_;
    std::string zonename_;
    bool use_odom_tf_;
    std::string data_path_;

    float max_accel_;
    float max_vel_;
    float corner_speed_rate_;

    Path path[LINE_NUM];

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

    void setup(std::string zone, float accel, float max_vel, float init_vel);
    void executeCB(const PursuitPathGoalConstPtr &goal);
    bool reachedGoal();
    void publishMsg(const float &vx, const float &vy, const float &omega);
    void update();
};