#include "path_planner.h"

std::string node_name = "path_planning_pursuit";

Path_Planner::Path_Planner(ros::NodeHandle &nh, const int &loop_rate, const std::string &zonename, const bool &use_odom_tf, const std::string &data_path)
    : nh_(nh), loop_rate_(loop_rate), zonename_(zonename), use_odom_tf_(use_odom_tf), data_path_(data_path)
{ //constructer, define pubsub
    ROS_INFO("Creating path_planning_pursuit");
    ROS_INFO_STREAM("zonename: " << zonename_);
    ROS_INFO_STREAM("use_odom_tf: " << use_odom_tf_);
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);

    cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    bno_sub = nh_.subscribe("pose", 1, &Path_Planner::bnoCallback, this);
    if(!use_odom_tf_){// don't use odom tf. instead, use odom topic for localization
        odom_sub = nh_.subscribe("odom", 1, &Path_Planner::odomCallback, this);
    }

    control.change_size(4,1);
    change_RB(zonename_);
    update();
}

void Path_Planner::odomCallback(const nav_msgs::Odometry &msg)
{
    position[0] = msg.pose.pose.position.x;
    position[1] = msg.pose.pose.position.y;
}


void Path_Planner::geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Passed by Reference
}

void Path_Planner::bnoCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_DEBUG("Received pose");
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, msg->pose.orientation);
    body_theta = yaw;
}

void Path_Planner::change_RB(std::string zone)
{
    using namespace std;
    for (int i = 0; i < LINE_NUM; i++)
    {
        // "data_path" has data path
        stringstream ss;
        if (zone == "blue"){
            ss << data_path_ + "/path_point_blue" << i + 1 << ".csv";
            string str = "Found " + ss.str() + " successfully";
            ROS_INFO("%s", str.c_str());
        }
        else if (zone == "red")
        {
            ss << data_path_ + "/path_point_red" << i + 1 << ".csv";
            string str = "Found " + ss.str() + " successfully";
            ROS_INFO("%s", str.c_str());
        }
        else
        {
            cerr << "err change_RB / filename" << endl;
            exit(1);
        }
        path[i].set_point_csv(ss.str());
    }
}

void Path_Planner::update(){
    ros::Rate r(loop_rate_);

    while (ros::ok()){
        if (use_odom_tf_){
            // tf_listner
            tf::StampedTransform transform;
            try
            {
                listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            position[0] = transform.getOrigin().x() * 1000.0f; // m -> mm
            position[1] = transform.getOrigin().y() * 1000.0f;
            //body_theta = tf::getYaw(transform.getRotation());
        }

        if(path_mode==0){
            ROS_WARN("no paths are selected");
            control[1][1] = 0;
            control[2][1] = 0;
            control[3][1] = 0;
        }
        else
        {
            ROS_INFO("pursuit path %d", path_mode - 1);
            //pure_pursuit
            // return [velx,vely,theta,ref_t](4*1)
            // forwardflag = 1 when move forward ,0 when move backward
            control = path[path_mode - 1].pure_pursuit(position[0], position[1], forwardflag);
            control[3][1] -= body_theta;
            // control[4][1]で追従時に参照した点番号がわかり、遷移先の検討に使える
            // pure_pursuitの4つ目の引数は開始時の点番号、底周辺から線形探索が始まる
        }

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = control[1][1];
        cmd_vel.linear.y = control[2][1];
        cmd_vel.angular.z = control[3][1];

        cmd_pub.publish(cmd_vel);

        //pathを遷移させる
        //publish path(nav_msgs/Path)(for visualization)

        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");

    int looprate = 30; // Hz
    std::string zonename = "blue";
    std::string data_path = "";
    bool use_odom_tf = false;

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("zone", zonename);
    arg_n.getParam("use_odom_tf", use_odom_tf);
    arg_n.getParam("data_path", data_path);

    Path_Planner planner(nh,looprate, zonename, use_odom_tf, data_path);
    return 0;
}
