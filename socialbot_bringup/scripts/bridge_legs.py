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
from leg_tracker.msg import PersonArray
from people_msgs.msg import PositionMeasurementArray
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
        self.rate = rospy.get_param('~rate',6.0)  # the rate at which to publish the message
        self.i = 0
        self.inic = Person()
        self.persona = People()
        self.detection = PositionMeasurementArray()
        # subscriptions
        rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.detectCallback)
        self.PeoplePub = rospy.Publisher("people", People, queue_size=10) 
        
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
        #rospy.loginfo("longitud del vector %i " % len(self.detection.people))
    
        while (len(self.persona.people) > 0):
            #rospy.loginfo("pop")
            self.persona.people.pop(0)    

        self.persona.header = self.detection.header
        self.persona.header.frame_id = "map"

        for j in range(0,len(self.detection.people)):
            if j <= (len(self.detection.people)-1):
                new_person = Person() 
                new_person.position.x = self.detection.people[j].pos.x 
                new_person.position.y = self.detection.people[j].pos.y
                new_person.name = str(j)
                self.persona.people.append(new_person)
                #rospy.loginfo("x = %f" % self.persona.people[0].position.x)
                #rospy.loginfo("y = %f" % self.persona.people[0].position.y)
                #rospy.loginfo("j = %i" % j)
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
    
    
   
