#include "path_planner.h"

std::string node_name = "path_planning_pursuit";

using namespace bezier_path_planning_pursuit;

Path_Planner::Path_Planner(ros::NodeHandle &nh, const int &loop_rate, const std::string &zonename,
                           const bool &use_odom_tf, const std::string &data_path, const float &max_accel,
                           const float &max_vel, const float &corner_speed_rate, const std::string &global_frame_id)
    : nh_(nh), loop_rate_(loop_rate), zonename_(zonename), use_odom_tf_(use_odom_tf),
      data_path_(data_path), max_accel_(max_accel), max_vel_(max_vel), corner_speed_rate_(corner_speed_rate), global_frame_id_(global_frame_id),
      as_(nh, node_name, boost::bind(&Path_Planner::executeCB, this, _1), false)
{ //constructer, define pubsub
    ROS_INFO("Creating path_planning_pursuit");
    ROS_INFO_STREAM("zonename: " << zonename_);
    ROS_INFO_STREAM("use_odom_tf: " << use_odom_tf_);
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("max_accel [m/s^2]: " << max_accel_);
    ROS_INFO_STREAM("max_vel [m/s]: " << max_vel_);
    ROS_INFO_STREAM("corner_speed_rate: " << corner_speed_rate_);
    ROS_INFO_STREAM("global_frame_id: " << global_frame_id_);

    cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("path", 1);
    bno_sub = nh_.subscribe("pose", 1, &Path_Planner::bnoCallback, this);
    if(!use_odom_tf_){// don't use odom tf. instead, use odom topic for localization
        odom_sub = nh_.subscribe("odom", 1, &Path_Planner::odomCallback, this);
    }

    control.change_size(4,1);
    setup(zonename_, max_accel_, max_vel_, corner_speed_rate_);

    as_.start();
    //update();
}

void Path_Planner::odomCallback(const nav_msgs::Odometry &msg)
{
    position[0] = msg.pose.pose.position.x * 1000.0f; // m -> mm
    position[1] = msg.pose.pose.position.y * 1000.0f;
}


void Path_Planner::geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Passed by Reference
}

geometry_msgs::Quaternion Path_Planner::rpy_to_geometry_quat(double roll, double pitch, double yaw)
{
    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

void Path_Planner::bnoCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //ROS_INFO("Received pose bezier");
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, msg->pose.orientation);
    body_theta = yaw;
}

void Path_Planner::setPoseTopic(const int &path_num)
{
    std_msgs::Header h;
    h.seq = path_num;
    h.stamp = ros::Time::now();
    h.frame_id = global_frame_id_;
    path_ros[path_num].header = h;

    int index = 0;
    for (float t = 1; t <= path[path_num].pnum; t += 0.1)
    {
        Matrix pose_unit(path[path_num].path_func(t));
        float x = pose_unit[1][1];
        float y = pose_unit[2][1];
        float theta = pose_unit[3][1];

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.pose.position.x = x;
        temp_pose.pose.position.y = y;
        temp_pose.pose.position.z = 0;

        geometry_msgs::Quaternion quat = rpy_to_geometry_quat(0, 0, theta);

        temp_pose.pose.orientation = quat;
        temp_pose.header = h;
        temp_pose.header.seq = index;
        index++;
        path_ros[path_num].poses.push_back(temp_pose);
    }
}

void Path_Planner::setup(std::string zone, float max_accel, float max_vel, float corner_speed_rate)
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
        float init_vel = max_accel / loop_rate_;
        path[i].load_config(ss.str(), max_accel, max_vel, init_vel, corner_speed_rate);
        setPoseTopic(i);
    }
}

void Path_Planner::publishMsg(const float &vx, const float &vy, const float &omega){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = omega;

    cmd_pub.publish(cmd_vel);
}

bool Path_Planner::reachedGoal(){
    if (forwardflag)
    {
        if (control[4][1] >= path[path_mode - 1].pnum - 0.05) // if the reference point almost reached goal
            return true;
    }
    else
    {
        if (control[4][1] <= 1.05) // if the reference point almost reached goal
            return true;
    }
    return false;
}

void Path_Planner::executeCB(const PursuitPathGoalConstPtr &goal) // if use action communication
{
    ros::Rate r(loop_rate_);

    bool success = true; // Used as a variable to store the success or failure of an action

    forwardflag = goal->direction;
    path_mode = goal->pathmode;

    ROS_INFO("%s: Executing, pathmode: %d, direction: %d", node_name.c_str(), goal->pathmode, goal->direction);

    while (ros::ok())
    {
        if (use_odom_tf_)
        {
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

        // Confirm action cancellation from action client
        if (as_.isPreemptRequested())
        {
            // Notify action cancellation
            ROS_INFO("%s: Preempted", node_name.c_str());
            publishMsg(0,0,0);
            // Action cancellation and consider action as failure and save to variable
            as_.setPreempted();
            success = false;
            break;
        }

        if (path_mode == 0)
        {
            ROS_WARN("no paths are selected, aborting %s", node_name.c_str());
            control[1][1] = 0;
            control[2][1] = 0;
            control[3][1] = 0;

            publishMsg(control[1][1], control[2][1], control[3][1]);

            as_.setPreempted();
            success = false;
            break;
        }
        else
        {
            //ROS_INFO("pursuit path %d", path_mode - 1);
            //pure_pursuit
            // return [velx,vely,theta,ref_t](4*1)
            // forwardflag = 1 when move forward ,0 when move backward
            control = path[path_mode - 1].pure_pursuit(position[0], position[1], forwardflag);
            control[3][1] -= body_theta;

            if(reachedGoal()){
                publishMsg(0,0,0);
                break;
            }
            // control[4][1]で追従時に参照した点番号がわかり、遷移先の検討に使える
            // pure_pursuitの4つ目の引数は開始時の点番号、点周辺から線形探索が始まる
        }

        publishMsg(control[1][1], control[2][1], control[3][1]);
        path_pub.publish(path_ros[path_mode-1]);
        feedback_.reference_point = control[4][1];
        as_.publishFeedback(feedback_);

        //publish path(nav_msgs/Path)(for visualization)

        ros::spinOnce();
        r.sleep();
    }

    // If the action target value is reached,
    // transmit current Fibonacci sequence as the result value.
    if (success)
    {
        result_.result = true;
        ROS_INFO("%s: Succeeded", node_name.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}

void Path_Planner::update(){ // if use topic communication
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
    float max_accel = 2.5;
    float max_vel = 1.0;
    float corner_speed_rate = 0.8;
    std::string global_frame_id = "odom";

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("zone", zonename);
    arg_n.getParam("use_odom_tf", use_odom_tf);
    arg_n.getParam("data_path", data_path);
    arg_n.getParam("max_accel", max_accel);
    arg_n.getParam("max_vel", max_vel);
    arg_n.getParam("corner_speed_rate", corner_speed_rate);
    arg_n.getParam("global_frame_id", global_frame_id);

    Path_Planner planner(nh, looprate, zonename, use_odom_tf, data_path, max_accel, max_vel, corner_speed_rate, global_frame_id);
    ros::spin(); // Wait to receive action goal
    return 0;
}
