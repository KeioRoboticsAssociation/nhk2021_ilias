#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
import bezier_path_planning_pursuit.msg
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from arrow_table_commander_msgs.msg import ArrowTableCommander

class Task_Selector():
    def __init__(self):
        self.pathmode_ = 0
        self.direction_ = 1
        self.pot_number = 0
        self.send_goal = False
        self.pot_position = ArrowTableCommander()
        self.client = actionlib.SimpleActionClient('/bezier_path_planning_pursuit', bezier_path_planning_pursuit.msg.PursuitPathAction)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.Joycallback)
        self.pot_position_sub = rospy.Subscriber("pot_position", ArrowTableCommander, self.PotPositioncallback)
        self.pathmode_pub = rospy.Publisher('/task_selector/joy_pathmode', Int32, queue_size=1)
        self.arrow_table_angle_pub = rospy.Publisher('arrow_table_angle', Float32MultiArray, queue_size=1)
        self.pot_distance_pub = rospy.Publisher('pot_distance', Float32MultiArray, queue_size=1)
        self.teleopflag_pub = rospy.Publisher('/task_selector/teleop_mode', Bool, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop_flag', Empty, queue_size=1)
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.publish_pot_position()
            r.sleep()

    def sendgoal(self):
        teleop_mode = Bool()
        teleop_mode.data = False
        self.teleopflag_pub.publish(teleop_mode)

        # Creates a goal to send to the action server.
        goal = bezier_path_planning_pursuit.msg.PursuitPathGoal(pathmode=self.pathmode_, direction=self.direction_)

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        '''
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        '''
    def PotPositioncallback(self, msg):
        self.pot_position = msg

    def Joycallback(self, msg):
        if msg.buttons[10] == 1:
            self.client.cancel_goal()
            teleop_mode = Bool()
            teleop_mode.data = True
            self.teleopflag_pub.publish(teleop_mode)
        elif msg.buttons[5] == 1:
            self.pathmode_ += 1
        elif msg.buttons[4] == 1:
            if self.pathmode_ > 0:
                self.pathmode_ -= 1
        elif msg.buttons[2] == 1:
            self.direction_ = 1
            self.sendgoal()
        elif msg.buttons[1] == 1:
            self.direction_ = 0
            self.sendgoal()
        elif msg.buttons[8] == 1:
            self.client.cancel_goal()
            teleop_mode = Bool()
            teleop_mode.data = True
            self.teleopflag_pub.publish(teleop_mode)
            emergency_msg = Empty()
            self.emergency_stop_pub.publish(emergency_msg)
        elif msg.buttons[5] == 1:
            self.pot_number += 1
            if self.pot_number > 5:
                self.pot_number = 0
        elif msg.buttons[4] == 1:
            self.pot_number -= 1
            if self.pot_number < 0:
                self.pot_number = 5
        
        message = Int32()
        message.data = self.pathmode_
        self.pathmode_pub.publish(message)

    def publish_pot_position(self):
        if self.pot_number != 0:
            pot_table_angle = self.pot_position.pot_angle[pot_number - 1]
            pot_distance = self.pot_position.pot_distance[pot_number - 1]
            if abs(pot_table_angle) > 0.9:
                pot_table_angle = pot_table_angle / abs(pot_table_angle) * 0.9
        else:
            pot_table_angle = 0.0
            pot_distance = 0.0

        angle_array = Float32MultiArray(data = [pot_table_angle])
        distance_array = Float32MultiArray(data = [pot_distance])

        self.arrow_table_angle_pub.publish(angle_array)
        self.pot_distance_pub.publish(distance_array)

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
