#!/usr/bin/env python
import rospy
import sys
import numpy as np
import actionlib
import math 
import time

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
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

roll = pitch = yaw = 0.0
kP = 0.5

def general_process():
    rospy.init_node('kinect_response')
    rospy.Subscriber("/neuro_gest_kinect/gesture", String, gesturecallback)
    rospy.Subscriber('/odom', Odometry, get_rotation)
    rospy.Subscriber("x_stream", Float32, xdatacallback)
    rospy.Subscriber("y_stream", Float32, ydatacallback)
    rospy.Subscriber("z_stream", Float32, zdatacallback)
    rospy.Subscriber("w_stream", Float32, wdatacallback)
    rospy.Subscriber("mensajedato",String, datacallback)
    rospy.spin()


def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    

def datacallback(data):
    mensaje=data.data
    auxiliar = 0
    #si la comparacion es avanzar va a un punto
    
    if mensaje == "avanza":
        rospy.loginfo("EL robot va a avanzar")
        socialbot_navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server to come up...")
        socialbot_navigation_client.wait_for_server()
        rospy.loginfo("move_base action server is available...")
        socialbot_robot1_goal.target_pose.header.stamp = rospy.Time.now()
        socialbot_robot1_goal.target_pose.header.frame_id = "map"
        socialbot_robot1_goal.target_pose.header.seq = 1 
        socialbot_robot1_goal.target_pose.pose.position.x = 1.5
        socialbot_robot1_goal.target_pose.pose.position.y = 0.0
        socialbot_robot1_goal.target_pose.pose.orientation.w = 1
        socialbot_robot1_goal.target_pose.pose.orientation.z = 0
        socialbot_robot1_goal.target_pose.pose.position.z = 0.0
        socialbot_navigation_client.send_goal(socialbot_robot1_goal)
        socialbot_navigation_client.wait_for_result()

    if mensaje == "izquierda":
        target_angle_left = 90.0
        pub = rospy.Publisher('/cmd_kinect',Twist,queue_size=1)
        command = Twist()
        while True:
            target_rad = target_angle_left * math.pi/180
            command.angular.z = kP * (target_rad - yaw)
            if (np.isclose(command.angular.z, 0,rtol= 0.01, atol= 0.01)):
                command.angular.z = 0.0
                pub.publish(command)
                break
            pub.publish(command)
            auxiliar = 1

    if mensaje =="derecha":
        target_angle_rigth = -90.0
        pub = rospy.Publisher('/cmd_kinect',Twist,queue_size=1)
        command = Twist()
        while True:
            target_rad = target_angle_rigth * math.pi/180
            command.angular.z = kP * (target_rad - yaw)
            if (np.isclose(command.angular.z, 0,rtol= 0.01, atol= 0.01)):
                command.angular.z = 0.0
                pub.publish(command)
                break
            pub.publish(command)
            auxiliar = 1

    if auxiliar == 1:
        target_angle_rigth = 0
        pub = rospy.Publisher('/cmd_kinect',Twist,queue_size=1)
        command = Twist()
        while True:
            target_rad = target_angle_rigth * math.pi/180
            command.angular.z = kP * (target_rad - yaw)
            if (np.isclose(command.angular.z, 0,rtol= 0.01, atol= 0.01)):
                command.angular.z = 0.0
                pub.publish(command)
                break
            pub.publish(command)
            auxiliar = 0


def gesturecallback(data):
    lectura_kinect = data.data
    auxiliar = 0
    if lectura_kinect == "avanzar":
            rospy.loginfo("EL robot va a avanzar")
            socialbot_navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo("Waiting for move_base action server to come up...")
            socialbot_navigation_client.wait_for_server()
            rospy.loginfo("move_base action server is available...")
            socialbot_robot1_goal.target_pose.header.stamp = rospy.Time.now()
            socialbot_robot1_goal.target_pose.header.frame_id = "map"
            socialbot_robot1_goal.target_pose.header.seq = 1 
            socialbot_robot1_goal.target_pose.pose.position.x = 1.50
            socialbot_robot1_goal.target_pose.pose.position.y = 0.0
            socialbot_robot1_goal.target_pose.pose.orientation.w = 1
            socialbot_robot1_goal.target_pose.pose.orientation.z = 0
            socialbot_robot1_goal.target_pose.pose.position.z = 0.0
            socialbot_navigation_client.send_goal(socialbot_robot1_goal)
            socialbot_navigation_client.wait_for_result()

    if lectura_kinect == "izquierda":
        target_angle_left = 90.0
        pub = rospy.Publisher('/cmd_kinect',Twist,queue_size=1)
        command = Twist()
        while True:
            target_rad = target_angle_left * math.pi/180
            command.angular.z = kP * (target_rad - yaw)
            if (np.isclose(command.angular.z, 0,rtol= 0.05, atol= 0.05)):
                command.angular.z = 0.0
                pub.publish(command)
                break
            pub.publish(command)
            auxiliar = 1

    if lectura_kinect =="derecha":
        target_angle_rigth = -90.0
        pub = rospy.Publisher('/cmd_kinect',Twist,queue_size=1)
        command = Twist()
        while True:
            target_rad = target_angle_rigth * math.pi/180
            command.angular.z = kP * (target_rad - yaw)
            if (np.isclose(command.angular.z, 0,rtol= 0.05, atol= 0.05)):
                command.angular.z = 0.0
                pub.publish(command)
                break
            pub.publish(command)
            auxiliar = 1
            
    if auxiliar == 1:
        target_angle_rigth = 0
        pub = rospy.Publisher('/cmd_kinect',Twist,queue_size=1)
        command = Twist()
        while True:
            target_rad = target_angle_rigth * math.pi/180
            command.angular.z = kP * (target_rad - yaw)
            if (np.isclose(command.angular.z, 0,rtol= 0.05, atol= 0.05)):
                command.angular.z = 0.0
                pub.publish(command)
                break
            pub.publish(command)
            auxiliar = 0
def xdatacallback(data):
    socialbot_robot1_goal.target_pose.pose.position.x = data.data

def ydatacallback(data):
    socialbot_robot1_goal.target_pose.pose.position.y = data.data

def zdatacallback(data):
    socialbot_robot1_goal.target_pose.pose.orientation.z = data.data
    
def wdatacallback(data):
    socialbot_robot1_goal.target_pose.pose.orientation.w = data.data
   
if __name__ == '__main__':
    try:
        socialbot_robot1_goal = MoveBaseGoal()
        general_process()
    except rospy.ROSInterruptException:
        pass