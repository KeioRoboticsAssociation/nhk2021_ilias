#! /usr/bin/env python

import rospy

import bezier_path_planning_pursuit.msg
from std_msgs.msg import Int32MultiArray

class Baffle_Over():
    def __init__(self):
        self.planner_status_sub = rospy.Subscriber("/bezier_path_planning_pursuit/goal", bezier_path_planning_pursuit.msg.PursuitPathActionGoal, self.Statuscallback)
        self.baffle_over_flag_pub = rospy.Publisher('/baffle_over_flag', Int32MultiArray, queue_size=1)

    def Statuscallback(self, msg):
        if msg.goal.pathmode == 1:
            array = [1]
            array_forPublish = Int32MultiArray(data=array)
            self.baffle_over_flag_pub.publish(array_forPublish)
        


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('baffle_over_flag_pub')
        print("create baffle_over_flag_pub")

        baffle_over = Baffle_Over()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
