#include "arrow_table_commander/arrow_table_commander.h"

std::string node_name = "arrow_table_commander";

Arrow_Table_Commander::Arrow_Table_Commander(ros::NodeHandle &nh, const int &loop_rate, const std::string &base_frame_id)
    : nh_(nh), loop_rate_(loop_rate), base_frame_id_(base_frame_id)
{ //constructer, define pubsub
    ROS_INFO("Creating arrow_table_commander");
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("base_frame_id: " << base_frame_id_);

    arrow_table_command_pub = nh_.advertise<arrow_table_commander_msgs::ArrowTableCommander>("pot_position", 1);   
    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        ListenTF();
        ros::spinOnce();
        r.sleep();
    }
}

void Arrow_Table_Commander::ListenTF()
{
    arrow_table_commander_msgs::ArrowTableCommander msg;

    for (int i = 0; i < 5; i++){
        // tf_listner
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/" + base_frame_id_, "/pot" + std::to_string(i+1), ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        msg.pot_angle[i] = atan2(transform.getOrigin().y(),transform.getOrigin().x()); // rad
        msg.pot_distance[i] = sqrt(transform.getOrigin().x()*transform.getOrigin().x() + transform.getOrigin().y()*transform.getOrigin().y());

    }

    arrow_table_command_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");

    int looprate = 30; // Hz
    std::string base_frame_id = "base_link";

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("base_frame_id", base_frame_id);

    Arrow_Table_Commander commander(nh, looprate, base_frame_id);
    return 0;
}
