#ifndef Arrow_Table_Commander_H
#define Arrow_Table_Commander_H

#include <ros/ros.h>
#include <tf/transform_listenerx.h>
#include <arrow_table_commander_msgs/ArrowTableCommander.h>

#include <cmath>
#include <string>

class Arrow_Table_Commander
{
public:
    Arrow_Table_Commander(ros::NodeHandle &nh, const int &loop_rate, const std::string &base_frame_id);
    ~Arrow_Table_Commander(){};

private:
    //Handlers
    ros::NodeHandle &nh_;

    ros::Publisher arrow_table_command_pub;

    //Configurations
    int loop_rate_;
    std::string base_frame_id_;

    //variables
    tf::TransformListener listener;

    //Methods
    void ListenTF();
};

#endif