#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

import roslib

from math import pow
from math import radians, degrees
import sys
import tf
import math

class ControlBot:
    def __init__(self):
        rospy.init_node('control_bot')
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose = PoseStamped()
        self.rate = rospy.Rate(10)

    def update_pose(self, msg):
        self.pose.pose = msg.pose.pose

    def move_to_goal(self, goal_pose):
        vel_msg = Twist()

        while not rospy.is_shutdown():
            # Calculating error terms
            dx = goal_pose.pose.position.x - self.pose.pose.position.x
            dy = goal_pose.pose.position.y - self.pose.pose.position.y
            distance = sqrt(dx**2 + dy**2)
            desired_angle = atan2(dy, dx)

            # Angular velocity
            vel_msg.angular.z = 0.5 * (desired_angle - self.yaw)
            # Linear velocity
            vel_msg.linear.x = 0.5 * distance

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            # Stopping condition
            if distance < 0.1:
                rospy.loginfo("Goal reached!")
                break

    def quaternion_to_yaw(self, quaternion):
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

if __name__ == '__main__':
    try:
        bot = ControlBot()

        while not rospy.is_shutdown():
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = float(input("Ingrese la coordenada x del objetivo: "))
            goal_pose.pose.position.y = float(input("Ingrese la coordenada y del objetivo: "))
            goal_pose.pose.orientation.w = 1.0  # Orientación por defecto

            bot.move_to_goal(goal_pose)

            answer = input("¿Desea ingresar otro punto? (y/n): ")
            if answer.lower() != 'y':
                break

    except rospy.ROSInterruptException:
        pass  
