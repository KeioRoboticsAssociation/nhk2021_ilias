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
        '''
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        '''

    def Joycallback(self, msg):
        if msg.buttons[10] == 1:
            self.client.cancel_goal()
        elif msg.buttons[5] == 1:
            self.pathmode_ += 1
        elif msg.buttons[4] == 1:
            self.pathmode_ -= 1
        elif msg.buttons[2] == 1:
            self.direction_ = 1
            self.sendgoal()
        elif msg.buttons[1] == 1:
            self.direction_ = 0
            self.sendgoal()
        
        print(self.pathmode_)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('task_selector')
        print("create task_selector")

        task_selector = Task_Selector()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
