#! /usr/bin/env python

import rospy

from std_msgs.msg import String
from std_msgs.msg import Empty

class ArrowThrowerEmergencyStop():
    def __init__(self):
        self.emergensy_sub = rospy.Subscriber("/emergency_stop_flag", Empty, self.emergencycallback)
        self.emergency_flag_pub = rospy.Publisher('/arrow_thrower_emergency_stop', String, queue_size=1)

    def emergencycallback(self, msg):
        self.emergency_flag_pub.publish("s")

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('arrow_thrower_emergency_stopper')
        print("create arrow_thrower_emergency_stopper")

        arrow_thrower_emergency_stoper = ArrowThrowerEmergencyStop()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
