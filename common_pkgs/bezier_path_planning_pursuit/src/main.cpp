#include "path_planner.h"

std::string node_name = "path_planning_pursuit";

using namespace bezier_path_planning_pursuit;

Path_Planner::Path_Planner(ros::NodeHandle &nh, const int &loop_rate, const std::string &zonename,
                           const bool &use_odom_tf, const std::string &data_path, const float &max_accel,
                           const float &max_vel, const float &corner_speed_rate, const std::string &global_frame_id,
                           const float &initial_vel, const float &xy_goal_tolerance, const float &yaw_goal_tolerance,
                           const std::string &angle_source, const float &max_vel_theta, const float &acc_lim_theta)
    : nh_(nh), loop_rate_(loop_rate), zonename_(zonename), use_odom_tf_(use_odom_tf),
      data_path_(data_path), max_accel_(max_accel), max_vel_(max_vel), corner_speed_rate_(corner_speed_rate),
      global_frame_id_(global_frame_id), initial_vel_(initial_vel), xy_goal_tolerance_(xy_goal_tolerance), yaw_goal_tolerance_(yaw_goal_tolerance), 
      angle_source_(angle_source), max_vel_theta_(max_vel_theta), acc_lim_theta_(acc_lim_theta),
      as_(nh, node_name, boost::bind(&Path_Planner::executeCB, this, _1), false)
{ //constructer, define pubsub
    ROS_INFO("Creating path_planning_pursuit");
    ROS_INFO_STREAM("zonename: " << zonename_);
    ROS_INFO_STREAM("use_odom_tf: " << use_odom_tf_);
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("acc_lim_xy [m/s^2]: " << max_accel_);
    ROS_INFO_STREAM("max_vel_xy [m/s]: " << max_vel_);
    ROS_INFO_STREAM("acc_lim_theta [rad/s^2]: " << acc_lim_theta_);
    ROS_INFO_STREAM("max_vel_theta [rad/s]: " << max_vel_theta_);
    ROS_INFO_STREAM("initial_vel [m/s]: " << initial_vel_);
    ROS_INFO_STREAM("corner_speed_rate: " << corner_speed_rate_);
    ROS_INFO_STREAM("global_frame_id: " << global_frame_id_);
    ROS_INFO_STREAM("xy_goal_tolerance: " << xy_goal_tolerance_);
    ROS_INFO_STREAM("yaw_goal_tolerance [rad]: " << yaw_goal_tolerance_);
    ROS_INFO_STREAM("angle_source: " << angle_source_);

    cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("path", 1);

    if (angle_source_ == "pose"){
        bno_sub = nh_.subscribe("pose", 1, &Path_Planner::bnoCallback, this);
    }
    else if (angle_source_ == "imu"){
        bno_sub = nh_.subscribe("imu", 1, &Path_Planner::imuCallback, this);
    }
    else{
        ROS_ERROR("Failed to define angle source. Aborting...");
        ros::shutdown();
    }

    if(!use_odom_tf_){// don't use odom tf. instead, use odom topic for localization
        odom_sub = nh_.subscribe("odom", 1, &Path_Planner::odomCallback, this);
    }

    control.change_size(5,1);
    setup(zonename_, max_accel_, max_vel_, acc_lim_theta_, max_vel_theta_, initial_vel_, corner_speed_rate_);

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
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, msg->pose.orientation);
    body_theta = yaw;
}

void Path_Planner::imuCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, imu->orientation);
    body_theta = yaw;
    //ROS_INFO("%f", body_theta*180.0/PI);
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
        float x = pose_unit[1][1] / 1000; // mm -> m
        float y = pose_unit[2][1] / 1000;
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

void Path_Planner::setup(std::string zone, float max_accel, float max_vel, float acc_lim_theta, float max_vel_theta, float init_vel, float corner_speed_rate)
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
        path[i].load_config(ss.str(), max_accel, max_vel, acc_lim_theta, max_vel_theta, init_vel, corner_speed_rate);
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
        if (control[4][1] >= path[path_mode - 1].pnum - xy_goal_tolerance_ && body_theta > control[5][1] - yaw_goal_tolerance_ && body_theta < control[5][1] + yaw_goal_tolerance_) // if the reference point almost reached goal
            return true;
    }
    else
    {
        if (control[4][1] <= 1.00 + xy_goal_tolerance_ && body_theta > control[5][1] - yaw_goal_tolerance_ && body_theta < control[5][1] + yaw_goal_tolerance_) // if the reference point almost reached goal
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
            control = path[path_mode - 1].pure_pursuit(position[0], position[1], body_theta, loop_rate_, forwardflag);
            //control[3][1] -= body_theta;

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
            control = path[path_mode - 1].pure_pursuit(position[0], position[1], body_theta, loop_rate_, forwardflag);
            //control[3][1] -= body_theta;
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
    float max_vel = 1.5;
    float corner_speed_rate = 0.8;
    float xy_goal_tolerance = 0.05;
    float yaw_goal_tolerance = 0.01;
    std::string global_frame_id = "odom";
    std::string angle_source = "pose";

    float acc_lim_theta = 3.2;
    float max_vel_theta = 1.57;

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("zone", zonename);
    arg_n.getParam("use_odom_tf", use_odom_tf);
    arg_n.getParam("data_path", data_path);
    arg_n.getParam("acc_lim_xy", max_accel);
    arg_n.getParam("acc_lim_theta", acc_lim_theta);
    arg_n.getParam("max_vel_xy", max_vel);
    arg_n.getParam("max_vel_theta", max_vel_theta);

    float initial_vel = max_accel / looprate;

    arg_n.getParam("initial_vel", initial_vel);
    arg_n.getParam("corner_speed_rate", corner_speed_rate);
    arg_n.getParam("global_frame_id", global_frame_id);
    arg_n.getParam("xy_goal_tolerance", xy_goal_tolerance);
    arg_n.getParam("yaw_goal_tolerance", yaw_goal_tolerance);
    arg_n.getParam("angle_source", angle_source);

    Path_Planner planner(nh, looprate, zonename, use_odom_tf, data_path, max_accel, max_vel, corner_speed_rate, global_frame_id, initial_vel, xy_goal_tolerance, yaw_goal_tolerance, angle_source, max_vel_theta, acc_lim_theta);
    ros::spin(); // Wait to receive action goal
    return 0;
}
