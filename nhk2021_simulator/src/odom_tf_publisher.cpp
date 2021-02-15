#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>

float position[2]={};
geometry_msgs::Quaternion odom_quat;

void odomCallback(const nav_msgs::Odometry &msg)
{
    position[0] = msg.pose.pose.position.x;
    position[1] = msg.pose.pose.position.y;
    odom_quat = msg.pose.pose.orientation;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_tf_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");

    std::string base_frame_id = "base_link";
    arg_n.getParam("base_frame_id", base_frame_id);

    odom_quat.x = 0;
    odom_quat.y = 0;
    odom_quat.z = 0;
    odom_quat.w = 1;

    ros::Subscriber sub = nh.subscribe("odom", 1, odomCallback);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();

    ros::Rate r(30.0);

    while (nh.ok())
    {
        current_time = ros::Time::now();

        //tf odom->base_link
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = base_frame_id;

        odom_trans.transform.translation.x = position[0];
        odom_trans.transform.translation.y = position[1];
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        ros::spinOnce();
        r.sleep();
    }
}