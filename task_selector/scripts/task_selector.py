#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
import bezier_path_planning_pursuit.msg
from sensor_msgs.msg import Joy

class Task_Selector():
    def __init__(self):
        self.pathmode_ = 0
        self.direction_ = 1
        self.send_goal = False
        self.enable_button = True # 配列にする
        self.client = actionlib.SimpleActionClient('/bezier_path_planning_pursuit', bezier_path_planning_pursuit.msg.PursuitPathAction)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.Joycallback)

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

    def sendgoal(self):
        # Creates a goal to send to the action server.
        goal = bezier_path_planning_pursuit.msg.PursuitPathGoal(pathmode=self.pathmode_, direction=self.direction_)

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()


    def Joycallback(self, msg):
        print("a")
        
        if self.push_button(msg.axes[1], self.enable_button[2]):
            self.pathmode_ += 1
        if self.push_button(msg.axes[1], self.enable_button[2]):
            self.pathmode_ -= 1
        
        if self.push_button(msg.axes[1], self.enable_button[2]):
            self.direction_=1
            self.sendgoal()

    def push_button(self, value, enable_button):
        if value == 1 and enable_button:
            enable_button = False
            return True
        else:
            if value == 0:
                enable_button = True
            return False


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('task_selector')
        print("create task_selector")

        task_selector = Task_Selector()
        task_selector.sendgoal()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
