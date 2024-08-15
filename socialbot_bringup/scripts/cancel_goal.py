#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
import actionlib


from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16, Int64
from people_msgs.msg import Person
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseActionResult, MoveBaseActionGoal
from std_msgs.msg import Bool,Float32,Int16,Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray, GoalID
from std_msgs.msg import String

def general_process():
    rospy.init_node('cancel_move_base')
    rospy.Subscriber("/neuro_gest_kinect/gesture", String, gesturecallback)
    rospy.Subscriber("mensajedato",String, datacallback)
    rospy.spin()

def datacallback(data):
    mensaje=data.data


def gesturecallback(data):
    lectura_kinect = data.data
    if lectura_kinect == "para":
        rospy.loginfo("paro general")
        cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)
        cmd_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.linear.y = 0.0
        move_msg.linear.z = 0.0
        move_msg.angular.x = 0.0
        move_msg.angular.y = 0.0
        move_msg.angular.z = 0.0
        cmd_pub.publish(move_msg)

if __name__ == '__main__':
    try:
        socialbot_robot1_goal = MoveBaseGoal()
        general_process()
    except rospy.ROSInterruptException:
        pass