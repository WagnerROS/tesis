#!/usr/bin/python3
import rospy
import sys
import numpy as np
import math
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Bool,Float32,Int16,Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
xodom = 0.0
yodom = 0.0
wodom = 0.0
zodom = 0.0
medida = 0.5
straigth = 0.15
turnrigth = 0.15
turnleft = -0.25
move = Twist()
move.linear.x = 0.0
move.linear.z = 0.0
estado = 0

def odomcallback(data):
    medida = 0.5
    limiter = 0.01
    limitea = 0.005 
    xodom = data.pose.pose.position.x
    yodom = data.pose.pose.position.y
    zodom = data.pose.pose.orientation.z
    wodom = data.pose.pose.orientation.w
    estado = 10
    if (np.isclose(yodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(xodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(zodom, 0,rtol= limitea , atol= limitea )) and (np.isclose(wodom, 0.99,rtol= limitea , atol= limitea )):
        move.angular.z =0.00
        move.linear.x =0.15
        estado = 0
        rospy.loginfo("1")
    if (np.isclose(yodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(xodom, medida,rtol= limiter , atol= limiter )) and (np.isclose(zodom, 0,rtol= limitea , atol= limitea )) and (np.isclose(wodom, 0.99,rtol= limitea , atol= limitea )):
        move.angular.z= -0.25
        move.linear.x = 0.00
        rospy.loginfo("2")
    if (np.isclose(yodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(xodom, medida,rtol= limiter , atol= limiter )) and (np.isclose(zodom, -0.707,rtol= limitea , atol= limitea )) and (np.isclose(wodom, 0.707,rtol= limitea , atol= limitea )):
        move.angular.z= 0.00
        move.linear.x =0.15
        rospy.loginfo("3")
    if (np.isclose(yodom, medida,rtol= limiter , atol= limiter )) and (np.isclose(xodom, medida,rtol= limiter , atol= limiter )) and (np.isclose(zodom, -0.707,rtol= limitea , atol= limitea )) and (np.isclose(wodom, 0.707,rtol= limitea , atol= limitea )):
        move.angular.z= -0.25
        move.linear.x = 0.00
        rospy.loginfo("4")
    if (np.isclose(yodom, medida,rtol= limiter , atol= limiter )) and (np.isclose(xodom, medida,rtol= limiter , atol= limiter )) and (np.isclose(zodom, -0.99,rtol= limitea , atol= limitea )) and (np.isclose(wodom, 0.0,rtol= limitea , atol= limitea )):
        move.angular.z= 0.00
        move.linear.x =0.15
        rospy.loginfo("5")
    if (np.isclose(yodom, medida,rtol= limiter , atol= limiter )) and (np.isclose(xodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(zodom, -0.99,rtol= limitea , atol= limitea )) and (np.isclose(wodom, 0.0,rtol= limitea , atol= limitea )):
        move.angular.z= -0.25
        move.linear.x = 0.00
        rospy.loginfo("6")
    if (np.isclose(yodom, medida,rtol= limiter , atol= limiter )) and (np.isclose(xodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(zodom, -0.707,rtol= limitea , atol= limitea )) and (np.isclose(wodom, -0.707,rtol= limitea , atol= limitea )):
        move.angular.z= 0.00
        move.linear.x =0.15
        estado = 1
        rospy.loginfo("7")
    if (np.isclose(yodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(xodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(zodom, -0.707,rtol= limitea , atol= limitea )) and (np.isclose(wodom, -0.707,rtol= limitea , atol= limitea )):
        move.angular.z= -0.25
        move.linear.x = 0.00
        rospy.loginfo("8")
    if (estado == 1) and (np.isclose(yodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(xodom, 0,rtol= limiter , atol= limiter )) and (np.isclose(zodom, 0.0,rtol= limitea , atol= limitea )) and (np.isclose(wodom, 0.99,rtol= limitea , atol= limitea )):
        move.angular.z= 0.00
        move.linear.x = 0.00
        rospy.loginfo("9")
    

def process():
    rospy.init_node('odom_test1', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("odom", Odometry, odomcallback)
    rate = rospy.Rate(1000000)

    while not rospy.is_shutdown():
        rate.sleep()
        pub.publish(move)





if __name__=="__main__":
    try:
        process()
    except rospy.ROSInterruptException:
        pass
