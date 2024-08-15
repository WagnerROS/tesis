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
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalID


# Brings in the messages used by the MoveBase action, including the
# goal message and the result message.


def process():
    checker = 0
    
    rospy.init_node('move_socialbot')

    #pub = rospy.Publisher('Confirm', Bool, queue_size=10)
    #rate = rospy.Rate(100)
    rospy.Subscriber("x_stream", Float32, xdatacallback)
    rospy.Subscriber("y_stream", Float32, ydatacallback)
    rospy.Subscriber("z_stream", Float32, zdatacallback)
    rospy.Subscriber("w_stream", Float32, wdatacallback)
    rospy.Subscriber("odom", Odometry, odomcallback)
    rospy.Subscriber("move_base/status", GoalStatusArray, goalcallback)
    pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    auxiliar = 1
    # Send the goal to the action server.
    rospy.spin()



def xdatacallback(data):
    socialbot_robot1_goal.target_pose.pose.position.x = data.data
    rospy.loginfo('recibe datos')
    # Create a SimpleActionClient for the move_base action server.
    socialbot_navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait until the move_base action server becomes available.
    rospy.loginfo("Waiting for move_base action server to come up...")
    socialbot_navigation_client.wait_for_server()
    rospy.loginfo("move_base action server is available...")
    #x_pos = rospy.get_param('~x_position')
    #y_pos = rospy.get_param('~y_position')
    # Construct the target pose for the turtlebot in the "map" frame.
    socialbot_robot1_goal.target_pose.header.stamp = rospy.Time.now()
    socialbot_robot1_goal.target_pose.header.frame_id = "map"
    socialbot_robot1_goal.target_pose.header.seq = 1 
    socialbot_robot1_goal.target_pose.pose.position.z = 0.0
    socialbot_robot1_goal.target_pose.pose.orientation.x = 0.0
    socialbot_robot1_goal.target_pose.pose.orientation.y = 0.0
    socialbot_robot1_goal.target_pose.pose.orientation.w = 1
    socialbot_robot1_goal.target_pose.pose.orientation.z = 0
    socialbot_navigation_client.send_goal(socialbot_robot1_goal)
    #rospy.loginfo("Goal sent to move_base action server.")
    #rospy.loginfo("Goal position x: %s  y: %s", socialbot_robot1_goal.target_pose.pose.position.x, socialbot_robot1_goal.target_pose.pose.position.y)
    # Wait for the server to finish performing the action.
    socialbot_navigation_client.wait_for_result()
    # Display a log message depending on the navigation result.
    navigation_result_status = socialbot_navigation_client.get_state()
    #if GoalStatus.SUCCEEDED != navigation_result_status:
        #rospy.logerr('Navigation to the desired goal failed :( -- Sorry, try again!)')
    #else:
        #rospy.loginfo('Hooray! Successfully reached the desired 
    #rospy.logerr('Navigation to the desired goal failed :( -- Sorry, try again!)')
    #pub.publish(checker)

def ydatacallback(data):
    socialbot_robot1_goal.target_pose.pose.position.y = data.data

def zdatacallback(data):
    socialbot_robot1_goal.target_pose.pose.orientation.z = data.data
    
def wdatacallback(data):
    socialbot_robot1_goal.target_pose.pose.orientation.w = data.data

def odomcallback(data):
    odomcallback.odomx = data.pose.pose.position.x
    odomcallback.odomy = data.pose.pose.position.y
   
def goalcallback(status):
    length = len(status.status_list)

    

if __name__ == '__main__':
    socialbot_robot1_goal = MoveBaseGoal()
    process()