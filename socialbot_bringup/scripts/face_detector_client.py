#!/usr/bin/env python

import rospy
from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import People 
from people_msgs.msg import Person 
from geometry_msgs.msg import Point



class FaceDetectClient(object):

	def __init__(self):
		self.face_detect_subs = rospy.Subscriber("/face_detector/people_tracker_measurements_array",PositionMeasurementArray,self.face_detect_subs_callback)
		self.pos_mesurement_array = PositionMeasurementArray()

	def face_detect_subs_callback(self,msg):
		self.pos_mesurement_array = msg

def Face_Detection_CLient_Start():
	rospy.init_node("face_detection_client_start_node")

	while not rospy.Time.now():
		pass

	face_detector_client = FaceDetectClient()
	rospy.spin()

if __name__ == "__main__":
	Face_Detection_CLient_Start()


