#!/usr/bin/env python


import rospy
#import roslib
#roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16, Int64
from cob_perception_msgs.msg import DetectionArray
from people_msgs.msg import People
from people_msgs.msg import Person

#############################################################################
class BridgePd:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("bridge_pd_to_social")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',20.0)  # the rate at which to publish the transform
        """self.ticks_meter = float(rospy.get_param('ticks_meter', 649))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.406)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta"""
        
        self.i = 0
        self.inic = Person()
        self.persona = People()
        self.detection = DetectionArray()
        # subscriptions
        rospy.Subscriber("/object_detection/detections", DetectionArray, self.detectCallback)
        self.PeoplePub = rospy.Publisher("people", People, queue_size=10) #original queue_size=10 , demasiado para arduino si se reduce el valor se pierde presicion
        #self.odomBroadcaster = TransformBroadcaster()
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        self.i = 0 
        self.u = 0
        #rospy.loginfo("longitud del vector %i " % len(self.detection.detections))
        

        for k in range(len(self.detection.detections)):
            #rospy.loginfo("k = %i" % k)
            #rospy.loginfo("i = %i" % self.i)
            if (self.detection.detections[k].label == "person"):
                self.i =self.i +1

        while (len(self.persona.people) < (self.i)):
            #rospy.loginfo("append")
            self.persona.people.append(self.inic)

        while (len(self.persona.people) > (self.i)):
            #rospy.loginfo("pop")
            self.persona.people.pop(0)    

        #rospy.loginfo("persona len = %i" % len(self.persona.people))
        for j in range(len(self.detection.detections)):
            if (self.detection.detections[j].label == "person"): 
                self.persona.header = self.detection.header
                self.persona.people[self.u].position.x = self.detection.detections[j].mask.roi.x +(self.detection.detections[j].mask.roi.width/2)
                #rospy.loginfo("x = %f" % self.persona.people[self.u].position.x)
                self.persona.people[self.u].position.y = self.detection.detections[j].mask.roi.y +(self.detection.detections[j].mask.roi.height/2)
                #rospy.loginfo("y = %i" % self.persona.people[self.u].position.y)
                self.u = self.u + 1
                #rospy.loginfo("j = %i" % j)
                #rospy.loginfo("u = %i" % self.u)
                if self.u == self.i:
                    self.PeoplePub.publish(self.persona)
           
    #############################################################################
    def detectCallback(self, msg):
    #############################################################################
        self.detection = msg                                 
        
    #############################################################################

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    Bridge = BridgePd()
    Bridge.spin()
    
    
   
