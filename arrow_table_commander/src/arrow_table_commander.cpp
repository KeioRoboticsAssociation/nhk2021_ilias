#include "arrow_table_commander/arrow_table_commander.h"

std::string node_name = "arrow_table_commander";

Arrow_Table_Commander::Arrow_Table_Commander(ros::NodeHandle &nh, const int &loop_rate, const std::string &base_frame_id)
    : nh_(nh), loop_rate_(loop_rate), base_frame_id_(base_frame_id)
{ //constructer, define pubsub
    ROS_INFO("Creating arrow_table_commander");
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate_);
    ROS_INFO_STREAM("base_frame_id: " << base_frame_id_);

    arrow_table_angle_pub = nh_.advertise<std_msgs::Float32MultiArray>("arrow_table_angle", 1);
    thrower_flag_pub = nh_.advertise<std_msgs::Int32MultiArray>("thrower_flag", 1);
    throwing1_power_pub = nh_.advertise<std_msgs::Float32MultiArray>("thrower1_power", 1);   
    throwing2_power_pub = nh_.advertise<std_msgs::Float32MultiArray>("thrower2_power", 1);   
    throwing3_power_pub = nh_.advertise<std_msgs::Float32MultiArray>("thrower3_power", 1);   
    throwing4_power_pub = nh_.advertise<std_msgs::Float32MultiArray>("thrower4_power", 1);   
    throwing5_power_pub = nh_.advertise<std_msgs::Float32MultiArray>("thrower5_power", 1);   
    joy_sub = nh_.subscribe("/joy", 1, &Arrow_Table_Commander::joy_callback, this);

    pot_number = 0;
    table_angle = 0.0;
    pot_distance = 0.0;
    thrower_number = 1;
    arrow_table_mode = 0;

    ros::Rate r(loop_rate_);

    while (ros::ok())
    {
        if (pot_number == 0){
            pot_distance = 0.0;
            table_angle = 0.0;
        }
        else{
            CalculateTableAngleAndDistance();
        }

        std_msgs::Float32MultiArray msg;
        msg.data.resize(1);
        msg.data[0] = table_angle;
        arrow_table_angle_pub.publish(msg);
        
        // std::cout << "table_angle " << table_angle * 180.0 / M_PI << std::endl;
        BroadcastThrowerTF();
        ros::spinOnce();
        r.sleep();
    }
}

geometry_msgs::Quaternion Arrow_Table_Commander::rpy_to_geometry_quat(double roll, double pitch, double yaw){
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

void Arrow_Table_Commander::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  std::cout << "arrow_table_mode " <<  arrow_table_mode << std::endl;
    std::cout << "thrower_number " << thrower_number << std::endl;
  if (joy_msg->buttons[JOY_PLUS_POT_NUMBER]){
      if(pot_number == 5){
          pot_number = 0;
      }
      else pot_number++;
  }
  else if (joy_msg->buttons[JOY_MINUS_POT_NUMBER]){
      if (pot_number == 0){
          pot_number = 5;
      }
      else pot_number--;
  }
  else if (joy_msg->buttons[JOY_RUN]){
      if(arrow_table_mode == 0){
          catch_arrow();
      }
      else if(arrow_table_mode == 1){
          swing_arrow();
      }
      else{
          throw_arrow(thrower_number);
          thrower_number++;
      }

      if (arrow_table_mode == 6){
          arrow_table_mode = 0;
          thrower_number = 1;
      }
      else arrow_table_mode++;
  }
}

void Arrow_Table_Commander::catch_arrow()
{
    std_msgs::Int32MultiArray msg;
    msg.data.resize(1);
    msg.data[0] = 0;
    thrower_flag_pub.publish(msg);
}

void Arrow_Table_Commander::swing_arrow()
{
    std_msgs::Int32MultiArray msg;
    msg.data.resize(1);
    msg.data[0] = 1;
    thrower_flag_pub.publish(msg);
}

void Arrow_Table_Commander::throw_arrow(int thrower_number)
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(1);
    msg.data[0] = CalculatePower(pot_distance);
    switch(thrower_number){
        case 1:
            throwing1_power_pub.publish(msg);
            break;
        case 2:
            throwing2_power_pub.publish(msg);
            break;
        case 3:
            throwing3_power_pub.publish(msg);
            break;
        case 4:
            throwing4_power_pub.publish(msg);
            break;
        case 5:
            throwing5_power_pub.publish(msg);
            break;
        default:
            ROS_ERROR("invalid thrower number");
    }
}

float Arrow_Table_Commander::CalculatePower(float distance){
    return distance; // TODO
}

void Arrow_Table_Commander::BroadcastThrowerTF()
{    
    ros::Time current_time; 
    current_time = ros::Time::now();

    //tf
    geometry_msgs::TransformStamped pot_table_trans;
    pot_table_trans.header.stamp = current_time;
    pot_table_trans.header.frame_id = base_frame_id_;
    pot_table_trans.child_frame_id = "pot_table";

    pot_table_trans.transform.translation.x = -0.1315;
    pot_table_trans.transform.translation.y = 0.0;
    pot_table_trans.transform.translation.z = 0.2;
    pot_table_trans.transform.rotation = rpy_to_geometry_quat(0.0, 0.0, table_angle);
    broadcaster.sendTransform(pot_table_trans);

    for (int i = 0; i < 5; i++){
        current_time = ros::Time::now();

        //tf
        geometry_msgs::TransformStamped thrower_trans;
        thrower_trans.header.stamp = current_time;
        thrower_trans.header.frame_id = "pot_table";
        thrower_trans.child_frame_id = "thrower" + std::to_string(i+1);

        thrower_trans.transform.translation.x = -0.35325;
        thrower_trans.transform.translation.y = -thrower_position[i];
        thrower_trans.transform.translation.z = 0.0;
        thrower_trans.transform.rotation = rpy_to_geometry_quat(0.0, 0.0, 0.0);
        broadcaster.sendTransform(thrower_trans);    
    }
}

void Arrow_Table_Commander::CalculateTableAngleAndDistance()
{    
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/pot_table_central", "/pot" + std::to_string(pot_number), ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    float pot_distance_from_table = sqrt(transform.getOrigin().x()*transform.getOrigin().x() + transform.getOrigin().y()*transform.getOrigin().y());
    table_angle = M_PI / 2 + atan2(transform.getOrigin().x(),-transform.getOrigin().y()) - asin(thrower_position[thrower_number-1] / pot_distance_from_table);

    pot_distance = sqrt(pot_distance_from_table * pot_distance_from_table - thrower_position[thrower_number-1] * thrower_position[thrower_number-1]) - 0.1315;

    // std::cout << "x " << transform.getOrigin().x() << std::endl;
    // std::cout << "y " << transform.getOrigin().y() << std::endl;
    // std::cout << "atan2 " << atan2(transform.getOrigin().x(),transform.getOrigin().y()) * 180.0 / M_PI << std::endl;
    // std::cout << "acos " << acos(thrower_position[thrower_number-1] / pot_distance_from_table) * 180.0 / M_PI << std::endl;

    if (table_angle < 0){
        table_angle += 2 * M_PI;
    }
    if (table_angle > 4.9){
        table_angle = 4.9;
    }

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
