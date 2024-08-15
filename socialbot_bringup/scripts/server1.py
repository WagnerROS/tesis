#!/usr/bin/env python
import rospy
import sys
import numpy as np
# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseActionResult, MoveBaseActionGoal
from std_msgs.msg import Bool,Float32,Int16,Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray


global vel_x
global checker
vel_x = 5.0
# Brings in the messages used by the MoveBase action, including the
# goal message and the result message.


def process():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("move_base/status", GoalStatusArray, goalcallback)
    rospy.Subscriber("cmd_vel", Twist, cmdcallback)
    process.pub = rospy.Publisher('Confirm', Bool, queue_size=10)
    rospy.spin()
    
    

def cmdcallback(data):
    vel_x = data.linear.x
    vel_y = data.linear.y
    #rospy.loginfo(vel_x)
    if vel_x == 0.0 and vel_y == 0.0:
        checker = True
        rospy.loginfo('quieto')
    else:
        checker = False
        rospy.loginfo('moviendose')
    process.pub.publish(checker)
    return vel_x  
       

def goalcallback(status):
    length = len(status.status_list)
    goalcallback.goalmed = status.status_list[length - 1].status

    return goalcallback.goalmed 
    

    

if __name__ == '__main__':
    checker = False
    velx = 0.0
    vely = 0.0
    process()